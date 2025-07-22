# Implementation Document for RP2350-Based UAV Flight Computer Firmware

## Document Overview

This implementation document serves as a detailed guideline for developing the firmware of an unmanned aerial vehicle (UAV) flight computer based on the RP2350 microcontroller. It translates the high-level design into actionable steps, providing pseudo code structures, feature checklists, and implementation strategies for each MVP checkpoint in the phased development roadmap. The focus is on creating modular, efficient code that adheres to Rust's no_std environment, utilizing only the RTIC crate for real-time task management and the rp235x-hal crate for hardware abstraction. All other components, such as sensor drivers, AHRS, EKF, and communication protocols, must be implemented natively in Rust without external libraries.

Pseudo code is used throughout to illustrate key structures and algorithms, emphasizing computational efficiency (e.g., preferring O(1) operations for real-time tasks, minimizing floating-point operations where possible, and leveraging hardware features like DMA and FPU for offloading). Modularity is maintained through trait-based abstractions, allowing for extensible components without runtime overhead. Developers should follow this as a checklist, implementing one MVP at a time, with testing and optimization integrated at each step.

Key principles:
- **Efficiency**: Use fixed-point arithmetic where precision allows (e.g., for sensor parsing) to reduce FPU dependency; ensure loops are bounded and avoid allocations.
- **Modularity**: Define traits for drivers and algorithms to enable swapping (e.g., PID vs. SMC) with compile-time resolution.
- **Safety**: Leverage RTIC's resource management for shared access; use atomic operations for inter-core communication.
- **Testing**: Each MVP includes a testable product and verification steps, focusing on bench tests with an ArduPilot drone for telemetry validation.

## System Requirements and Assumptions

Implementation assumes the hardware and software environment as specified in the high-level design. Developers must:
- Verify rp235x-hal compatibility with the RP2350 datasheet (August 8, 2024 version) for peripherals like I2C, SPI, UART, PIO, and DMA.
- Implement custom fixed-point math utilities (e.g., a struct for Q-format numbers) for DSP tasks, avoiding floating-point where sub-millisecond latency is critical.
- Ensure all code runs in a no_std context, with panic handling routed to a watchdog reset.

## Firmware Architecture Implementation

Implement the three-layer architecture as follows:

1. **Hardware Layer**: Use rp235x-hal to configure peripherals. Create manager structs for shared resources (e.g., I2C bus) wrapped in RTIC resources for late initialization.
   - Pseudo code for a bus manager (e.g., for I2C):
     ```
     struct I2CManager {
         i2c: I2CPeripheral,  // From rp235x-hal
         in_use: bool,        // Flag for exclusive access
     }

     impl I2CManager {
         fn acquire() -> Option<&mut I2CPeripheral> {
             if !self.in_use {
                 self.in_use = true;
                 return Some(&mut self.i2c);
             }
             return None;  // O(1) check, prevents conflicts
         }

         fn release() {
             self.in_use = false;
         }
     }
     ```
     - Efficiency: O(1) acquire/release; use in RTIC tasks to ensure single-threaded access.

2. **Driver Layer**: Define traits for sensor interactions to abstract borrowing. Each driver implements read/parse/calibrate methods.
   - Pseudo code for a sensor trait:
     ```
     trait SensorDriver {
         fn read_raw(&mut self, bus: &mut BusType) -> RawData;  // Borrow bus briefly, O(n) where n is data size (small)
         fn parse(&self, raw: RawData) -> ParsedData;           // Fixed-point ops, O(1)
         fn calibrate(&mut self, params: &Params) -> bool;      // Update biases, O(1)
     }
     ```
     - Modularity: Allows swapping sensors without changing application logic.

3. **Application Layer**: Use RTIC for tasks on dual cores. Core 0 for high-priority estimation/control; Core 1 for logging/CAN.
   - Shared resources: Use queues (e.g., fixed-size arrays with atomic indices) for inter-core data.
   - Pseudo code for a queue:
     ```
     struct FixedQueue<T, const SIZE: usize> {
         buffer: [T; SIZE],
         read_idx: AtomicUsize,
         write_idx: AtomicUsize,
     }

     impl FixedQueue {
         fn push(&self, item: T) -> bool {  // O(1), atomic ops
             let idx = self.write_idx.fetch_add(1, Ordering::Relaxed) % SIZE;
             if (idx + 1) % SIZE == self.read_idx.load(Ordering::Relaxed) { return false; }  // Full check
             self.buffer[idx] = item;
             true;
         }

         fn pop(&self) -> Option<T> {  // O(1)
             let idx = self.read_idx.load(Ordering::Relaxed);
             if idx == self.write_idx.load(Ordering::Relaxed) { return None; }
             let item = self.buffer[idx];
             self.read_idx.store((idx + 1) % SIZE, Ordering::Relaxed);
             Some(item);
         }
     }
     ```
     - Efficiency: Bounded size prevents growth; atomic for no locks.

## Implementation of Functionality and Features

- **Core Functionality**:
  - Sensor Acquisition: Use DMA for SPI/IMU reads to offload CPU; interrupt triggers RTIC task for parsing.
  - AHRS/EKF: Custom implementations using quaternion math (fixed-point where possible). AHRS fuses gyro/accel/mag in O(n) steps (n=small matrix sizes).
  - Logging: DMA writes to SD; format as binary with timestamps (O(1) per entry).
  - Actuators: PIO for PWM; configure state machines for parallel output.
  - CAN Reception: DMA RX with CRC-16-CCITT check (O(n) over packet, n<100 bytes).

- **Advanced Features**:
  - Flight Modes: Enum-based state machine; traits for controllers (e.g., PID: O(1) per update).
  - Parameter System: Struct with fixed fields; load from flash, validate on update (O(1) lookups).
  - Calibration: Accumulate samples (bounded buffer), compute averages/biases (O(n) with n=100-1000).
  - Telemetry-Only: Disable control tasks; queue data for CAN TX to Radio.

- **Safety and Reliability**:
  - Watchdog: RTIC monotonic timer resets.
  - CRC: Polynomial computation, precompute table for O(n) efficiency.

## Implementation of Error Prevention Measures

- **Ownership/Borrowing**: Use RTIC late resources; wrap mutables in UnsafeCell only if needed, but prefer task priorities.
- **I2C Conflicts**: Timeout in acquire (e.g., spinloop with counter); recovery: reset bus via rp235x-hal.
- **SPI/DMA**: Check status flags in interrupts; retry on overflow (rare, O(1)).
- **CAN**: Validate CRC-16-CCITT before queuing; baud rate: 115200 for balance.
- **Dual-Core**: Affinity in RTIC; atomics for shared params.
- **General**: Static pin assignments; health checks in idle task.

## Phased Development and MVP Checkpoint Implementation

Follow the phases sequentially. Each MVP includes:
- **Checklist**: Features to implement.
- **Pseudo Code Snippets**: Key structures/algorithms.
- **Testing and Optimization**: Steps and metrics.

### Phase 1: Basic Functionality and Optimization

**Objective**: Set up hardware/drivers and basic tasks.

- **MVP 1.1: Hardware and Driver Setup**
  - **Checklist**:
    - Configure peripherals (I2C, SPI, DMA) in init.
    - Implement bus managers for I2C/SPI.
    - Create trait-based drivers for BMP388, BMM350, PCF8563, BMI088, ICM-20948, Ublox NEO-M9N.
    - Parse raw data with fixed-point scaling.
    - Output parsed data via serial (for testing).
  - **Pseudo Code Snippet** (Driver Example for BMI088 IMU):
    ```
    struct BMI088Driver {
        accel_bias: Vector3,  // Fixed-point
        gyro_scale: Vector3,
    }

    impl SensorDriver for BMI088Driver {
        fn read_raw(&mut self, spi_bus: &mut SPIBus) -> RawData {
            spi_bus.select_chip();              // O(1)
            let data = spi_bus.transfer_dma([REG_ACCEL, 0; 12]);  // DMA offload
            spi_bus.deselect();
            data
        }

        fn parse(&self, raw: RawData) -> ParsedData {
            let accel_x = (raw[0] as i32 * self.accel_scale.x) - self.accel_bias.x;  // O(1) fixed-point mul/sub
            // Similarly for y,z, gyro
            ParsedData { accel: Vector3(accel_x, accel_y, accel_z), gyro: ... }
        }

        fn calibrate(&mut self, params: &Params) -> bool {
            self.accel_bias = params.get("accel_bias");  // O(1) lookup
            true;
        }
    }
    ```
  - **Testable Product**: Binary that reads and serial-prints sensor data at 10 Hz.
  - **Testing**: Connect to ArduPilot drone; verify accuracy (±1% error) via known references.
  - **Improvements**: Adjust traits for borrow errors; ensure no data races.

- **Optimization 1.1**: Profile DMA setup/teardown (target <100 µs); measure CPU usage during reads (target <5% at 100 Hz).

- **MVP 1.2: Basic Application Tasks**
  - **Checklist**:
    - Set up RTIC app with tasks for sensor reads (priority high).
    - Implement state machine enum (Idle, Armed, TelemetryOnly).
    - Enable/disable tasks based on mode.
    - Collect data in a struct for logging/serial.
  - **Pseudo Code Snippet** (RTIC Task for Sensor Read):
    ```
    #[task(priority = 3, resources = [i2c_manager, baro_driver])]
    fn read_baro(cx: read_baro::Context) {
        let bus = cx.resources.i2c_manager.acquire().unwrap();  // Exclusive
        let raw = cx.resources.baro_driver.read_raw(bus);
        cx.resources.i2c_manager.release();
        let parsed = cx.resources.baro_driver.parse(raw);
        // Store in shared struct or queue
    }
    ```
  - **Testable Product**: Binary with mode transitions via dummy input; consistent data collection.
  - **Testing**: Simulate modes; check log for no jitter (>99% on-time).
  - **Improvements**: Assign priorities to minimize preemption delays.

- **Optimization 1.2**: Balance tasks across cores (reads on Core 0); profile queue latencies (<10 µs).

### Phase 2: CAN Reception from Radio MC and Optimization

**Objective**: Add CAN RX with CRC-16-CCITT; integrate parameters.

- **MVP 2.1: CAN Setup**
  - **Checklist**:
    - Configure CAN DMA RX on Core 1.
    - Implement frame format: Header (2 bytes len/type) + Payload + CRC (2 bytes).
    - Validate CRC-16-CCITT in task.
    - Queue valid payloads (e.g., telemetry requests).
  - **Pseudo Code Snippet** (CAN Parse Task):
    ```
    #[task(priority = 2, core = 1, resources = [can_dma_buffer, queue])]
    fn parse_can(cx: parse_can::Context) {
        let buffer = cx.resources.can_dma_buffer;  // Fixed size
        let crc_calc = crc16_ccitt(buffer[0..len]);  // O(n), n small
        if crc_calc != buffer[len..len+2] { log_error(); return; }
        let payload = extract_payload(buffer);
        cx.resources.queue.push(payload);  // O(1)
    }
    ```
  - **Testable Product**: Receive and queue data from simulated Radio MC.
  - **Testing**: Send corrupted frames; verify rejection (<1% false positive).
  - **Improvements**: Optimize frame size (minimize headers).

- **Optimization 2.1**: Profile throughput (target 1 kB/s); buffer sizing to avoid overruns.

- **MVP 2.2: Parameter System Integration**
  - **Checklist**:
    - Define Params struct with fixed fields (e.g., array of key-value).
    - Load from flash on boot; update from CAN payloads.
    - Validate updates (e.g., range checks).
  - **Pseudo Code Snippet** (Params Update):
    ```
    struct Params {
        values: [f32; NUM_PARAMS],  // Or fixed-point
        keys: [&str; NUM_PARAMS],
    }

    impl Params {
        fn update(&mut self, key: &str, value: f32) -> bool {
            for i in 0..NUM_PARAMS {  // O(k), k small (~50)
                if self.keys[i] == key {
                    if value in MIN..MAX { self.values[i] = value; return true; }
                }
            }
            false;
        }
    }
    ```
  - **Testable Product**: Remote param changes affect drivers.
  - **Testing**: Update via sim; confirm no invalid states.
  - **Improvements**: Hash keys for O(1) if needed, but array sufficient.

- **Optimization 2.2**: Minimize flash writes (batch); test CRC with noise injection.

### Phase 3: Estimation and Basic Control with Optimization

**Objective**: Add AHRS/EKF and control.

- **MVP 3.1: AHRS and EKF Implementation**
  - **Checklist**:
    - Custom AHRS: Madgwick-like fusion (quaternions, O(1) per update).
    - Custom EKF: 15-state (pos/vel/att/bias), matrix ops with fixed-point.
    - Integrate CAN commands to toggle.
    - Log estimates.
  - **Pseudo Code Snippet** (AHRS Update):
    ```
    struct AHRS {
        quat: Quaternion,  // Fixed-point
    }

    impl AHRS {
        fn update(&mut self, gyro: Vector3, accel: Vector3, mag: Vector3, dt: f32) {
            let gyro_quat = integrate_gyro(gyro, dt);  // O(1) mul/add
            self.quat = normalize(self.quat * gyro_quat);
            let correction = gradient_descent(accel, mag, self.quat);  // O(1), small matrices
            self.quat = self.quat + correction * BETA;
        }
    }
    ```
  - **Testable Product**: Fused data in logs.
  - **Testing**: Bench vs. ArduPilot; drift <1°/min.
  - **Improvements**: Tune gains via params.

- **Optimization 3.1**: Use FPU for matrices; parallelize prediction/correction if possible.

- **MVP 3.2: Basic Control and Modes**
  - **Checklist**:
    - Trait for controllers (e.g., PID).
    - Link to state machine for mode enables.
    - Output to PIO actuators.
  - **Pseudo Code Snippet** (PID Controller Trait):
    ```
    trait Controller {
        fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32;  // O(1)
    }

    struct PID {
        kp: f32, ki: f32, kd: f32,
        integral: f32,
    }

    impl Controller for PID {
        fn update(&mut self, set: f32, meas: f32, dt: f32) -> f32 {
            let err = set - meas;
            self.integral += err * dt;  // Anti-windup if needed
            let deriv = (err - prev_err) / dt;
            kp * err + ki * self.integral + kd * deriv
        }
    }
    ```
  - **Testable Product**: Actuator responses in test rig.
  - **Testing**: Overshoot <5%; stability checks.
  - **Improvements**: Link calibrations to gains.

- **Optimization 3.2**: Profile control loop (<1 ms); test alternative controllers.

### Phase 4: Advanced Features and Final Optimization

**Objective**: Complete and validate.

- **MVP 4.1: Calibrations and Modes**
  - **Checklist**:
    - Implement calibration tasks (accumulate, compute).
    - Full modes (Stabilize, etc.) via traits.
    - Select algorithms via params.
  - **Pseudo Code Snippet** (Calibration):
    ```
    fn calibrate_accel(samples: [Vector3; 100]) -> Vector3 {
        let mut sum = Vector3::zero();
        for s in samples { sum += s; }  // O(n), n=100
        sum / 100.0
    }
    ```
  - **Testable Product**: Mode switches; calibrated ops.
  - **Testing**: Flight tests; evaluate smoothness.
  - **Improvements**: Test IEKF variant.

- **Optimization 4.1**: Balance core loads (<50% each); enable sleep modes.

- **MVP 4.2: End-to-End Validation**
  - **Checklist**:
    - Integrate all; add fault injection.
    - Full telemetry via Radio MC.
  - **Pseudo Code Snippet** (Fault Handling):
    ```
    fn health_check(params: &Params) {
        if sensor_error > params.threshold { state_machine.transition(Recovery); }  // O(1)
    }
    ```
  - **Testable Product**: Mission-ready firmware.
  - **Testing**: Compare to ArduPilot; RMS error <0.5 m.
  - **Improvements**: Refine from logs; iterate optimizations.
