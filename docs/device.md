# IS66WVS16M8FBLL PSRAM device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the datasheet listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## PSRAM (ISSI IS66WVS16M8FBLL, hand-fitted in socket U24; Rev A 01/2025)

Pins: see "Pins and bus" below, QMI CS1 / PSRAM.

- 2.7-3.6 V, 128 Mbit = 16 MiB, 24-bit address, two stacked 64 Mbit dies. A burst must not cross the die
  boundary. 1024-byte wrapped page.
- Powers up in SPI mode; wait >= 150 us after power-up. No recovery time is given for the 0x66/0x99 reset;
  150 us works. Measured: the reset kept the contents (64 KiB checked).
- Commands: 0x66 reset-enable then 0x99 reset (must be consecutive); 0x9F read ID (SPI: 24 address bits
  don't-care, no wait); 0x03 read (33 MHz in SPI mode); 0x0B fast read (8 wait cycles SPI, 4 QPI);
  0xEB quad IO read (6 wait cycles, 104 MHz); 0x02 write; 0x38 quad IO write (0 wait, 104 MHz);
  0x35 enter QPI; 0xF5 exit QPI.
- Read ID: manufacturer 0x9D, KGD 0x5D (0x55 = failed die), then 48-bit EID whose top three bits give the
  density (100 = 128 Mbit).
- CE# low time limit tCEM: 4 us up to 85 degC. CE# high time >= 1 clock. Max clock 104 MHz.
  Measured at 75 MHz: Read ID CE# low 1347 ns; window bursts end at 3.1 us with MAX_SELECT 7 (bound 3.85 us).
- Driver: this crate; test `tools/run.sh psram_smoke`. At clk_sys 150 MHz: QPI, EBh read with 24 dummy
  bits, 38h write, CLKDIV 2 (75 MHz), RXDELAY 2, COOLDOWN 1, PAGEBREAK 1024, MAX_SELECT 7, MIN_DESELECT 2,
  SELECT_SETUP 1. Measured RXDELAY pass window: 1-4 at CLKDIV 2, 0-5 at CLKDIV 3.
- Measured, 1 MiB sequential: uncached alias write 20.8 MiB/s, read 24.4 MiB/s; cached alias write 7.9 MiB/s
  (write-allocate), read 25.2 MiB/s. The driver hands out the uncached alias 0x15000000.
- Measured with the driver compiled at opt-level 0 with debug assertions: direct-mode CE# low up to 13.4 us (over
  tCEM); every data check still passed. Compile the driver optimised in debug builds too.
- Differences from the APS6404L that `embassy_rp::psram` targets: manufacturer ID 0x9D instead of 0x0D,
  16 MiB instead of 8 MiB, 104 MHz instead of 144 MHz, tCEM 4 us instead of 8 us, QPI fast read at 84 MHz.
- Socket U24 is one SOIC-8 land for either this PSRAM, an APS6404L-SQN-SN (8 MB QSPI PSRAM) or a W25Q32JVSNIQ
  (32 Mbit QSPI NOR). The PCB is assembled with U24 empty.
- Programming the in-package flash resets the QMI configuration; `Psram::remap()` re-programs the window
  without a chip command (verified with the M1 reset simulated).
- The QMI registers used to reach it are under RP2350 below.

## RP2350 / RP2354B (RP2350.pdf)

- QMI: base 0x400d0000. Direct mode: set DIRECT_CSR.EN, wait BUSY low, assert CS1N, push to DIRECT_TX, always
  drain DIRECT_RX. XIP (the running program's flash) is inaccessible while direct mode is enabled, so that code
  must execute from RAM with interrupts off. M1 format/command registers: RFMT/RCMD/WFMT/WCMD
  (p1236-1247). Writes through the window need CTRL.WRITABLE_M1. Verified: window 1 at 0x11000000 (cached), 0x15000000
  (uncached). Direct-mode FIFOs: faster via XIP_AUX 0x50500004/8 (p349). Cache clean by set/way: 0x1bffc001+8n (E11).

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 0 | FLASH_CS1 | QMI CS1n (F9) | PSRAM socket U24 chip select; 10k pull-up R35 |

| Peripheral | GPIOs | Device |
|---|---|---|
| QMI CS1 | 0 | PSRAM socket U24 |

| Item | Value |
|---|---|
| PSRAM socket U24 | CS GPIO0, 10k pull-up R35; clock and SD0-3 are the MCU's dedicated QSPI pins shared with the in-package flash; supply V3V3_SYS; no pull-ups on SD2/SD3 |

## Datasheet

| Device | URL |
|---|---|
| RP2350 / RP2354B | https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf |
| IS66WVS16M8FBLL PSRAM | https://cdn.sparkfun.com/assets/a/7/e/0/0/IS66WVS16M8FBLL-104NLI-COM-29556.pdf |
| APS6404L-SQN-SN PSRAM (socket option) | https://www.apmemory.com/tw/downloadFiles/0324112120g3605268 |
| W25Q32JVSNIQ NOR (socket option) | https://www.winbond.com/resource-files/W25Q32JV%20RevJ%2012242024%20Plus.pdf |
