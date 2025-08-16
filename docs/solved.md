Solved issues log

Template for future entries
- Title: <short summary>
- Date: <YYYY-MM-DD>
- Scope: <repo/branch/binary>
- Symptom: <what you observed>
- Root cause: <why it happened>
- Fix: <high-level change>
- Steps: <ordered steps you applied>
- Verification: <how you proved it works>
- Affected files: <key files edited>
- Notes/Gotchas: <things to remember>
- References: <links to docs/issues>

Entry: defmt RTT not showing logs in multi-binary workspace
- Title: defmt RTT silent when using multiple cargo bins (FC, GS, Radio)
- Date: 2025-08-16
- Scope: repo rustypico, branch main, all three binaries (FC, gs, radio)
- Symptom: Flashing worked via probe-rs, but no RTT/defmt logs appeared on probe console when running cargo run --bin <name> on the multi-bin branch. Same code printed logs on the single-bin branch.
- Root cause:
  - The workspace didn’t consistently link the defmt linker script and RTT sink for every bin. defmt needs:
    - Linker script defmt.x passed to the linker
    - The RTT sink (defmt-rtt) linked by each binary, and at least one defmt log executed so LTO doesn’t drop the sink
    - Runner attaching that can decode defmt (probe-rs run is fine)
  - In multi-bin setups it’s easy for one bin to miss the defmt_rtt linkage or for rustflags/runner env to be absent.
- Fix:
  - Centralize target/rustflags/runner and DEFMT_LOG in .cargo/config.toml at the repo root.
  - Ensure each binary links the RTT sink by adding use defmt_rtt as _; and emit an early defmt::info!.
  - Keep rp235x-hal built with feature "defmt" (already enabled).
- Steps:
  1) Create .cargo/config.toml with:
     [build]
     target = "thumbv8m.main-none-eabihf"

     [target.thumbv8m.main-none-eabihf]
     rustflags = [
         "-C", "link-arg=--nmagic",
         "-C", "link-arg=-Tlink.x",
         "-C", "link-arg=-Tdefmt.x",   # required for defmt symbol table
         "-C", "target-cpu=cortex-m33",
     ]

     runner = "probe-rs run --chip RP2350"

     [env]
     DEFMT_LOG = "debug"
  2) In each bin main:
     - Add: use defmt_rtt as _;  // link RTT sink
     - Prefer: defmt::info!("<boot banner>"); near startup so the sink isn’t optimized out.
  3) Build and run per bin (PowerShell):
     - cargo run --bin FC
     - cargo run --bin gs
     - cargo run --bin radio
  4) Observe probe-rs console showing decoded defmt logs.
- Verification:
  - Confirmed RTT logs appear after flashing FC; repeated for gs and radio with boot banners.
- Affected files:
  - .cargo/config.toml (added rustflags, runner, DEFMT_LOG)
  - fc/main.rs (already had defmt_rtt and defmt logs)
  - gs/main.rs (added defmt_rtt and boot defmt log)
  - radio/main.rs (added defmt_rtt and boot defmt log)
- Notes/Gotchas:
  - If you want panic messages over defmt, use panic-probe instead of panic-halt.
  - Only one active probe session can own the RTT; close other debug terminals.
  - Ensure dependent crates enabling their own "defmt" features when needed.
  - For other chips adjust runner --chip accordingly.
- References:
  - defmt book: https://defmt.ferrous-systems.com/
  - probe-rs: https://probe.rs/docs/tools/
