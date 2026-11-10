# qmi-psram

Driver for the ISSI IS66WVS16M8FBLL 16 MiB PSRAM in socket U24 of the MARV V2 Rev 1 flight controller (RP2354B), on the RP2350's second QSPI chip select. The chip knowledge, the timing calculation and the start-up sequence are hardware-independent. The RP2350 back-end (feature `rp2350`) uses raw registers, so it needs no HAL and works under embassy-rp, another HAL or none. The driver turns the PSRAM into 16 MiB of ordinary memory.

## Usage

```rust
let p = embassy_rp::init(Default::default());
let (_qmi_cs1, _pin) = (p.QMI_CS1, p.PIN_0);              // keep both unused
// SAFETY: QMI CS1 and GPIO0 belong to the driver; core 1 is not running from flash.
let qmi = unsafe { qmi_psram::rp2350::Rp2350Qmi::new(qmi_psram::rp2350::Cs1Pin::Gpio0) };
let mut psram = qmi_psram::Psram::new(qmi);
psram.init(&mut embassy_time::Delay, embassy_rp::clocks::clk_sys_freq())?;
let memory: &mut [u8] = psram.as_mut_slice();             // 16 MiB
```

- `init` works whatever state the chip was left in and does not write to the memory. Call it before starting the second core: the flash the program runs from is unreachable for a few microseconds at a time while it talks to the chip (interrupts are off for about 8 us at a time, measured).
- Programming the in-package flash resets the RP2350's set-up for the PSRAM. Call `remap()` afterwards; the contents are kept.
- The memory is handed out at 0x15000000, the uncached address. The cached one (0x11000000) also works but writes through it are slower (7.9 against 20.8 MiB/s).
- Compile this crate optimised in debug builds too (`[profile.dev.package.qmi-psram] opt-level = 3`). Unoptimised, its chip-select pulses last up to 13 us against the chip's 4 us limit.
- The timing is computed from clk_sys: 75 MHz at 150 MHz, with one step of sampling margin measured on each side on this board.

## What the application provides

- A `DelayNs` for `init`.
- clk_sys in hertz.
- A `critical-section` implementation, for example embassy-rp's `critical-section-impl`.

Dependencies: `embedded-hal` 1.0 and, optionally, `critical-section` and `defmt`.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |
| `rp2350` | the raw-register RP2350 back-end; the application provides the `critical-section` implementation |

## Measured on the MARV V2 Rev 1 board

- All 16 MiB verified at 75 MHz.
- 20.8 MiB/s write and 24.4 MiB/s read.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh psram_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `psram_smoke` | chip ID, start from either chip state, all 16 MiB written and read back twice, retention, chip against window, sampling margin, throughput, re-map, chip-select and interrupts-off times; never programs the flash |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: QMI CS1 on GPIO0 (10k pull-up R35). Clock and SD0-3 are the MCU's dedicated QSPI pins shared with the in-package flash.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `psram_smoke` exited 0 when last run.

Not done for this device:

- An allocator on top of the memory.
- The other two parts the socket accepts (APS6404L, W25Q32).
- Use while the second core runs from flash.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and psram_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   register, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`.
