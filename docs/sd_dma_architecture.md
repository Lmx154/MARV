# SD Logging & DMA Architecture Plan

This document specifies the planned architecture for high-throughput, low-jitter logging to an SD card while coexisting with time-sensitive sensor and CAN operations. It is a forward-looking design to guide implementation after core drivers are completed.

> Status: DESIGN (not yet implemented)

---
## 1. DMA-Friendly Architecture Choice
We adopt the Hybrid Bus Strategy:
- SPI1: Dedicated to MCP2515 CAN controller (interrupt-driven, latency sensitive).
- SPI0: Shared between BMI088 (IMU) and SD card logging.
- I2C0 / I2C1: Managed via a bus manager for multi-client sensor access.

Rationale:
- BMI088 transfers are short & predictable (register reads, burst sensor frames) and can be interleaved between SD write bursts.
- SD writes are buffered and chunked to avoid starving the IMU or CAN subsystems.
- CAN is isolated on its own hardware SPI port for maximum ISR responsiveness.

Key Architectural Pieces:
- `Spi0Bus` abstraction with push-down arbitration & explicit SD DMA session state.
- Double-buffered log aggregator feeding block-aligned DMA writes.
- RTIC task priority levels ensuring time-critical tasks preempt logging scheduling (not mid-DMA, but between bounded DMA chunks).

---
## 2. SPI Bus Allocation Strategy
| Bus  | Devices            | Purpose                                 | Notes |
|------|--------------------|-----------------------------------------|-------|
| SPI0 | SD Card + BMI088   | Logging (bulk) + IMU (periodic sampling)| Shared w/ arbitration |
| SPI1 | MCP2515             | CAN frames (interrupt driven)           | Exclusive |

Chip Select & Proposed Pins (See `PINOUT.md`):
- BMI088: CS_ACCEL=GP17, CS_GYRO=GP22 (shared SCK=GP18, MOSI=GP19, MISO=GP16)
- SD: CS=GP9 (separate microSD SPI signal set re-mapped onto SPI0 lines through board routing assumption) *NOTE: Align routing so SD shares SPI0 wires physically; currently doc lists SD on GP6–GP9, but for a unified SPI0 physical bus we may reroute—see Section 7.*
- MCP2515: CS=GP13, SCK=GP10, MOSI=GP11, MISO=GP12 (SPI1)

Arbitration Rules:
1. SPI0 starts in `Idle`.
2. BMI088 operations acquire short-term exclusive access (`with_gyro`, `with_accel`).
3. SD DMA write requests transition SPI0 to `SdDmaPending` then `SdDmaActive`.
4. No BMI088 operation starts while `SdDmaActive`.
5. DMA bursts are limited in length (block count) to guarantee latency bounds (< IMU period slack).

---
## 3. Does SD Need Its Own Bus?
Conclusion: Not at present.
- Sharing SPI0 keeps pin usage efficient and avoids dedicating SPI1 (needed for CAN).
- Latency impact is bounded by chunking DMA transfers (e.g., 4–8 blocks per burst).
- Multi-block write efficiencies retained while not monopolizing the bus beyond an acceptable scheduling window.

Edge Case Where Separate Bus Might Be Required:
- Logging throughput target rises above ~2–3 MB/s sustained.
- IMU sampling increases to very high rates (e.g., >3 kHz) with tight jitter constraints.

Mitigation Strategies Before Adding a Bus:
- Increase SPI clock for SD only (post-init up to 25–50 MHz if signal integrity holds).
- Adaptive chunk sizing: shrink multi-block window if IMU overruns detected.
- Opportunistic writes during low sensor activity windows.

---
## 4. Reliability Impact of Sharing
Reliability is not degraded electrically by sharing so long as:
- Distinct chip-select lines with proper pull-ups/pull-downs avoid spurious selection.
- All devices release MISO line (high-Z) when CS inactive (both BMI088 & SD meet this).

Main Risk: Timing contention leading to missed IMU sample windows or CAN ISR latency (only if we misplaced CAN onto the shared bus; we avoided this).

Controls:
- Bounded SD DMA chunk duration target: < 5 ms per burst.
- Schedule logger flush at a lower RTIC priority; high-priority sensor task sets a defer flag if it detects SPI0 busy near its next deadline.
- Optionally insert a time guard before launching another chunk (e.g., ensure > X µs until next IMU read).

---
## 5. RTIC + DMA Integration Pattern
### Resource Overview
```
#[resources]
struct Resources {
    spi0_bus: Spi0Bus,         // Owns SPI0 peripheral + CS pins
    spi1_can: Spi1Can,         // MCP2515 driver
    dma_log: DmaChannels,      // (tx, rx optional) for SD
    log_mgr: LogManager,       // Buffers + state machine
    i2c0_mgr: &'static BusManagerSimple<I2c0>,
    i2c1_mgr: &'static BusManagerSimple<I2c1>,
    bmi_state: BmiState,
    gps_state: GpsState,
    can_state: CanState,
}
```

### Task Sketch
```
#[task(priority=5, resources=[spi0_bus, bmi_state])]
fn imu_sample(cx: imu_sample::Context) {
    cx.resources.spi0_bus.with_gyro(|spi, cs| bmi088_read_gyro(spi, cs));
    cx.resources.spi0_bus.with_accel(|spi, cs| bmi088_read_accel(spi, cs));
}

#[task(priority=3, resources=[log_mgr, spi0_bus, dma_log])]
fn log_flush(cx: log_flush::Context) {
    if let Some(chunk) = cx.resources.log_mgr.dequeue_chunk() {
        if cx.resources.spi0_bus.try_start_sd_dma(chunk, &mut cx.resources.dma_log).is_ok() {
            // DMA started; completion handled in dma_irq task
        } else {
            // Requeue / retry later
        }
    }
}

#[task(binds=DMA_IRQ_0, priority=4, resources=[spi0_bus, log_mgr, dma_log])]
fn dma_irq(cx: dma_irq::Context) {
    if cx.resources.spi0_bus.complete_sd_dma(&mut cx.resources.dma_log) {
        cx.resources.log_mgr.mark_chunk_done();
        log_flush::spawn().ok(); // kick next chunk
    }
}
```

### State Machine (Spi0Bus)
```
Idle -> BmiShortOp -> Idle
Idle -> SdDmaPending -> SdDmaActive -> Idle
(BmiShortOp blocked while SdDmaActive)
```

---
## 6. Practical Buffer + Locking Design
### Buffers
- Two large sector-aligned buffers: `BUF_A` & `BUF_B` (e.g., 4096 bytes each = 8 sectors @ 512B).
- Producers write variable-length log records into `active_fill` buffer.
- When threshold reached (e.g., >= 3072 bytes or forced flush), swap and enqueue a `ChunkDescriptor { ptr, len, sectors }`.

### LogManager API
```
fn push_record(&mut self, rec: &[u8]) -> Result<(), Overflow>;
fn force_flush(&mut self);
fn dequeue_chunk(&mut self) -> Option<ChunkDescriptor>;
fn mark_chunk_done(&mut self);
```
Guarantees records are not split across DMA boundaries (record truncated or buffer flushed early).

### Concurrency Control
- `Spi0Bus` internal enum tracks ownership.
- `try_start_sd_dma` returns Err(Busy) if not Idle.
- BMI088 helpers early-return if `SdDmaActive`.
- Optional: Maintain a short IMU shadow queue; if IMU read skipped once, logger reduces next chunk size.

### DMA Setup Essentials
- Configure SPI TX FIFO + DMA channel for memory-to-peripheral.
- Optionally ignore RX (discard) or set a dummy buffer to keep DMA symmetrical.
- Use a preamble generator (CMD / tokens) before launching DMA for pure payload phases.
- For CMD25 multi-block writes: send CMD, then per-block token + payload; simplest is to DMA only payload segments and write tokens manually in between blocks (or build a contiguous block that includes tokens if timing allows).

---
## 7. Alternative (PIO / Re-Mapping) Options
### Option A: Keep Current Pinout (SD Separate SPI Signals)
Pros: Board already wired; no reroute needed.
Cons: Requires either software SPI or an additional SPI instance (not available) unless pins are remapped to hardware SPI0 lines.

### Option B: Re-route SD onto SPI0 Physical Lines
- Align SD SCK/MOSI/MISO to SPI0 (GP18/19/16) and choose a distinct CS (e.g., GP9 or GP6).
- Pros: Hardware SPI acceleration + DMA with no PIO.
- Cons: Hardware revision or rework required if PCB already fixed.

### Option C: PIO-Based Dedicated SD Bus
- Use PIO state machine to implement SPI for SD at custom clock.
- Pros: Offloads SPI0; IMU gets uncontested hardware SPI0.
- Cons: PIO program complexity, extra DMA choreography, higher maintenance.
- Apply only if IMU jitter becomes unacceptable with shared bus.

### Option D: PIO for IMU, SPI0 for SD
- Offload IMU (lower bandwidth) to a minimal PIO SPI; free SPI0 fully for high-speed SD multi-block DMA.
- Likely unnecessary unless SD throughput target escalates.

Decision Path:
1. Start with Option A (current doc). Abstract via `Spi0Bus` so migration to B/C/D only changes one module.
2. Measure contention (timestamp gaps in IMU sampling & flush duration metrics).
3. If >5% missed IMU windows or >X ms logger-induced latency observed, evaluate remap or PIO.

---
## 8. Implementation Roadmap (Deferred)
1. Implement BMI088 driver (accel + gyro) with blocking SPI helpers.
2. Introduce `Spi0Bus` skeleton (no DMA yet) + logging buffer mock (writes to UART).
3. Add SD block driver (CMD0..init..single-block write).
4. Add multi-block write + double buffer.
5. Integrate DMA channels & ISR.
6. Add metrics: max flush time, IMU jitter histogram.
7. Optimize / consider PIO only if metrics demand.

---
## 9. Risks & Mitigations
| Risk | Impact | Mitigation |
|------|--------|-----------|
| SD long busy times | IMU sample jitter | Limit blocks/flush, schedule between samples |
| DMA race / corrupt buffer | Data loss | Use state flags + chunk descriptors + assert alignment |
| Power dip during write | File system corruption | Periodic FAT sync or journaling format (LittleFS / custom) |
| CAN ISR starvation (if miswired) | Lost CAN frames | Keep CAN on SPI1 only |
| Token / CRC timing errors | Write failures | Start with single-block writes; add multi-block after validation |

---
## 10. Metrics to Collect Post-Implementation
- Average / max DMA flush duration (µs)
- IMU sampling jitter (µs deviation)
- Logger backlog depth (# records waiting)
- SD write error rate / retries
- Bus arbitration contention counts

---
## 11. Summary
A shared SPI0 with structured arbitration and bounded DMA bursts provides efficient SD logging without compromising sensor or CAN responsiveness. The design isolates critical real-time paths, offers clear migration levers (PIO, remap), and keeps the implementation incremental.

Prepared for: Future driver + RTIC integration phase.
Maintainer: (add name/contact)
