//! RP2350 back-end: QMI chip select 1 through raw registers.
//!
//! Addresses and fields come from the RP2350 datasheet: QMI registers p1235-1247, XIP_CTRL p345-346,
//! XIP_AUX p349-351, XIP cache maintenance p341, erratum RP2350-E11 p1373-1374, IO_BANK0 and
//! PADS_BANK0 base addresses p31, GPIO_CTRL p609, pad registers p786, function table (QMI CS1n is
//! F9) p590-592.
//!
//! While direct mode is enabled the XIP window, and with it the flash the program runs from, is
//! unreachable (p1234). The code that runs in that time lives in RAM (`.data`), calls nothing,
//! and runs inside `critical_section::with`. Configuring the window enables direct mode for a
//! moment too, to wait until no XIP transfer is in flight (p1238). The direct-mode FIFO data goes
//! through the XIP_AUX AHB port, which is faster than the QMI's APB registers (p344-345), so the
//! chip select stays low for as short a time as possible.
//!
//! Compile this crate optimised, in debug builds too (`[profile.dev.package.qmi-psram]` with
//! `opt-level = 3`). How long the chip select stays low in a direct-mode transaction depends on how
//! fast this code runs: measured on the board, at most 2.1 us optimised and up to 13.4 us at
//! `opt-level = 0`, against the chip's 4 us limit. Unoptimised code with debug assertions also
//! calls the assertion checks of `read_volatile` and `write_volatile`, which live in flash; in
//! that test they ran from the XIP cache and every data check still passed.
#![allow(unsafe_code)]

use crate::bus::{BusError, QmiBus, Width};
use crate::timing::{DirectClock, WindowRegs};

/// Cached alias of QMI window 1 (p340, p1226).
pub const CACHED_BASE: usize = 0x1100_0000;
/// Uncached alias of QMI window 1, the one [`Rp2350Qmi`] hands out (p340).
pub const UNCACHED_BASE: usize = 0x1500_0000;

const QMI_BASE: usize = 0x400d_0000;
const DIRECT_CSR: *mut u32 = QMI_BASE as *mut u32;
/// QMI_DIRECT_TX and QMI_DIRECT_RX through the XIP_AUX port at 0x5050_0000.
const AUX_DIRECT_TX: *mut u32 = 0x5050_0004 as *mut u32;
const AUX_DIRECT_RX: *mut u32 = 0x5050_0008 as *mut u32;
const M1_TIMING: *mut u32 = (QMI_BASE + 0x20) as *mut u32;
const M1_RFMT: *mut u32 = (QMI_BASE + 0x24) as *mut u32;
const M1_RCMD: *mut u32 = (QMI_BASE + 0x28) as *mut u32;
const M1_WFMT: *mut u32 = (QMI_BASE + 0x2c) as *mut u32;
const M1_WCMD: *mut u32 = (QMI_BASE + 0x30) as *mut u32;
const ATRANS4: *mut u32 = (QMI_BASE + 0x44) as *mut u32;

const CSR_EN: u32 = 1 << 0;
const CSR_BUSY: u32 = 1 << 1;
const CSR_ASSERT_CS1N: u32 = 1 << 3;
const CSR_AUTO_CS1N: u32 = 1 << 7;
const CSR_RXEMPTY: u32 = 1 << 16;
const CSR_RXLEVEL_SHIFT: u32 = 18;
const TX_OE: u32 = 1 << 19;
const TX_DWIDTH_16: u32 = 1 << 18;
const TX_IWIDTH_QUAD: u32 = 2 << 16;
/// Entries the direct-mode TX and RX FIFOs each hold (Figure 129, p1226).
const FIFO_DEPTH: usize = 4;

const XIP_CTRL: *mut u32 = 0x400c_8000 as *mut u32;
const XIP_CTRL_WRITABLE_M1: u32 = 1 << 11;
/// Clean by set/way through the top 16 kB of the maintenance window: the RP2350-E11 workaround,
/// which re-tags the cleaned dirty lines outside the QMI address range. Clean lines stay.
const MAINT_CLEAN_E11: usize = 0x1bff_c001;
/// 16 kB of 8-byte lines.
const CACHE_LINES: usize = 2048;

const IO_BANK0_BASE: usize = 0x4002_8000;
const PADS_BANK0_BASE: usize = 0x4003_8000;
const FUNCSEL_QMI_CS1N: u32 = 9;
const PAD_ISO: u32 = 1 << 8;
const PAD_IE: u32 = 1 << 6;
const PAD_DRIVE_12MA: u32 = 3 << 4;
const PAD_SLEWFAST: u32 = 1 << 0;

/// Busy-wait budget, in loop iterations: about a millisecond at 150 MHz.
const SPIN_LIMIT: u32 = 100_000;
/// Longest transaction this back-end accepts.
const MAX_WRITE: usize = 8;
const MAX_READ: usize = 32;
/// FIFO entries of the longest transaction, two bytes per entry.
const MAX_ENTRIES: usize = MAX_WRITE / 2 + MAX_READ / 2;

/// The GPIOs that have the QMI CS1n function (F9, RP2350 datasheet Table 645, p590-592).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Cs1Pin {
    /// GPIO0.
    Gpio0 = 0,
    /// GPIO8.
    Gpio8 = 8,
    /// GPIO19.
    Gpio19 = 19,
    /// GPIO47, QFN-80 packages (RP2350B, RP2354B) only.
    Gpio47 = 47,
}

/// QMI chip select 1 of the RP2350, through raw registers.
pub struct Rp2350Qmi {
    pin: Cs1Pin,
}

impl Rp2350Qmi {
    /// Gives `pin` the QMI CS1n function: pad output 12 mA fast slew, input enabled, no pulls
    /// (the board has a pull-up), pad isolation removed.
    ///
    /// # Safety
    ///
    /// - QMI chip select 1 and `pin` belong to this value alone for as long as it, or the
    ///   [`crate::Psram`] holding it, is in use. Under embassy-rp, keep `p.QMI_CS1` and the
    ///   pin's peripheral unused.
    /// - While `init` or `remap` runs, nothing else executes from flash or touches the XIP
    ///   window: the other core is stopped or runs from RAM, and no DMA channel reads flash or
    ///   the window.
    pub unsafe fn new(pin: Cs1Pin) -> Self {
        let n = pin as usize;
        let pad = (PADS_BANK0_BASE + 0x04 + 4 * n) as *mut u32;
        let ctrl = (IO_BANK0_BASE + 0x04 + 8 * n) as *mut u32;
        let config = PAD_IE | PAD_DRIVE_12MA | PAD_SLEWFAST;
        // SAFETY: the caller hands over the pin; these are its pad and control registers.
        // Isolation stays on until the function is selected, so the pin does not glitch.
        unsafe {
            pad.write_volatile(config | PAD_ISO);
            ctrl.write_volatile(FUNCSEL_QMI_CS1N);
            pad.write_volatile(config);
        }
        Self { pin }
    }

    /// The chip select pin.
    pub fn pin(&self) -> Cs1Pin {
        self.pin
    }
}

// SAFETY: `window_base` is the uncached alias of window 1, which maps the device on CS1 through
// the identity translation `program_window` writes. Nothing else in Rust refers to it: the
// constructor's contract gives this value CS1 exclusively.
unsafe impl QmiBus for Rp2350Qmi {
    fn transfer(
        &mut self,
        clock: DirectClock,
        width: Width,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), BusError> {
        if write.len() > MAX_WRITE || read.len() > MAX_READ {
            return Err(BusError::TooLong);
        }
        // The FIFO entries, built on the stack before direct mode: two bytes per entry (the first
        // byte goes out first, p1238), one for an odd last byte. The caller's buffers may be in
        // flash or in the window itself.
        let (tx_write, tx_read) = match width {
            Width::Single => (0, 0),
            Width::Quad => (TX_OE | TX_IWIDTH_QUAD, TX_IWIDTH_QUAD),
        };
        let mut entries = [0u32; MAX_ENTRIES];
        let mut n = 0;
        for pair in write.chunks(2) {
            entries[n] = match *pair {
                [a, b] => tx_write | TX_DWIDTH_16 | u32::from(a) | (u32::from(b) << 8),
                _ => tx_write | u32::from(pair[0]),
            };
            n += 1;
        }
        let n_write = n;
        for pair in read.chunks(2) {
            entries[n] = if pair.len() == 2 {
                tx_read | TX_DWIDTH_16
            } else {
                tx_read
            };
            n += 1;
        }
        let mut input = [0u8; MAX_READ];
        let csr = clock.csr_bits();
        let ok = critical_section::with(|_| {
            // SAFETY: `entries` holds `n` entries, of which those from `n_write` on return the
            // bytes `input` has room for; both are on this stack in RAM. Interrupts are off and
            // the constructor's contract keeps every other XIP user away.
            unsafe { direct_transfer(csr, entries.as_ptr(), n, n_write, input.as_mut_ptr()) }
        });
        read.copy_from_slice(&input[..read.len()]);
        if ok { Ok(()) } else { Err(BusError::Timeout) }
    }

    fn program_window(&mut self, regs: &WindowRegs) -> Result<(), BusError> {
        let regs = *regs;
        let ok = critical_section::with(|_| {
            // SAFETY: `regs` is a copy on this stack in RAM; interrupts are off; the constructor's
            // contract keeps every other XIP user away.
            unsafe { write_window(&regs) }
        });
        if ok { Ok(()) } else { Err(BusError::Timeout) }
    }

    fn flush_cache(&mut self) {
        for line in 0..CACHE_LINES {
            // SAFETY: a byte write to the cache maintenance window performs the operation its
            // address selects and stores nothing (p341). Byte writes keep the operation bits.
            unsafe { ((MAINT_CLEAN_E11 + 8 * line) as *mut u8).write_volatile(0) };
        }
        barrier();
    }

    fn window_base(&self) -> usize {
        UNCACHED_BASE
    }
}

/// Data and instruction synchronisation barrier.
#[inline(always)]
fn barrier() {
    #[cfg(target_arch = "arm")]
    // SAFETY: barriers have no operands and no effect besides ordering.
    unsafe {
        core::arch::asm!("dsb", "isb", options(nostack, preserves_flags));
    }
}

/// One direct-mode transaction on CS1: pushes the `n` FIFO entries at `entries` and stores the
/// bytes received for the entries from `n_write` on (one or two per entry, as the entry's
/// DWIDTH says) at `input`. Returns false if a wait ran out. Leaves direct mode off and
/// DIRECT_CSR as it was.
///
/// Every entry returns one RX entry. At most `FIFO_DEPTH` entries are outstanding (pushed and not
/// popped), so the TX FIFO never overflows without polling TXFULL, and one DIRECT_CSR read per
/// round says how many entries to pop.
///
/// # Safety
///
/// Runs from RAM with direct mode on, so it must not be interrupted by anything that executes
/// from flash: call it in a critical section, with no other XIP user. `entries` must be valid for
/// `n` entries and `input` for the bytes they return; neither may be in flash or the XIP window.
#[unsafe(link_section = ".data.ram_func")]
#[inline(never)]
unsafe fn direct_transfer(
    csr: u32,
    entries: *const u32,
    n: usize,
    n_write: usize,
    input: *mut u8,
) -> bool {
    let mut ok = true;
    // SAFETY: QMI register accesses; the buffer accesses stay within the lengths the caller
    // vouches for.
    unsafe {
        let saved = DIRECT_CSR.read_volatile();
        DIRECT_CSR.write_volatile(csr | CSR_EN);
        // A transfer that was in flight when direct mode was enabled must finish first.
        let mut spin = SPIN_LIMIT;
        while DIRECT_CSR.read_volatile() & CSR_BUSY != 0 {
            if spin == 0 {
                ok = false;
                break;
            }
            spin -= 1;
        }
        if ok {
            DIRECT_CSR.write_volatile(csr | CSR_EN | CSR_ASSERT_CS1N);
            let mut pushed = 0usize;
            let mut popped = 0usize;
            let mut stored = 0usize;
            let mut spin = SPIN_LIMIT;
            while popped < n {
                while pushed < n && pushed - popped < FIFO_DEPTH {
                    AUX_DIRECT_TX.write_volatile(entries.add(pushed).read());
                    pushed += 1;
                }
                let mut level = (DIRECT_CSR.read_volatile() >> CSR_RXLEVEL_SHIFT) & 0x7;
                while level > 0 && popped < n {
                    let data = AUX_DIRECT_RX.read_volatile();
                    if popped >= n_write {
                        input.add(stored).write(data as u8);
                        stored += 1;
                        if entries.add(popped).read() & TX_DWIDTH_16 != 0 {
                            input.add(stored).write((data >> 8) as u8);
                            stored += 1;
                        }
                    }
                    popped += 1;
                    level -= 1;
                }
                if spin == 0 {
                    ok = false;
                    break;
                }
                spin -= 1;
            }
            // BUSY stays high half an SCK period after the last bit, which is the CE# hold.
            let mut spin = SPIN_LIMIT;
            while DIRECT_CSR.read_volatile() & CSR_BUSY != 0 {
                if spin == 0 {
                    ok = false;
                    break;
                }
                spin -= 1;
            }
            DIRECT_CSR.write_volatile(csr | CSR_EN);
        }
        let mut drain = 8;
        while drain > 0 && DIRECT_CSR.read_volatile() & CSR_RXEMPTY == 0 {
            let _ = AUX_DIRECT_RX.read_volatile();
            drain -= 1;
        }
        DIRECT_CSR.write_volatile(saved & !(CSR_EN | CSR_ASSERT_CS1N | CSR_AUTO_CS1N));
        barrier();
    }
    ok
}

/// Writes the window-1 registers once the QMI is idle. Returns false if it did not become idle.
///
/// # Safety
///
/// As for [`direct_transfer`]; `regs` must point to RAM.
#[unsafe(link_section = ".data.ram_func")]
#[inline(never)]
unsafe fn write_window(regs: *const WindowRegs) -> bool {
    let mut ok = true;
    // SAFETY: QMI and XIP_CTRL register accesses; `regs` is valid by the caller's contract.
    unsafe {
        let saved = DIRECT_CSR.read_volatile();
        // Direct mode blocks new XIP transfers; BUSY low means none is in flight (p1238), so the
        // window registers may change (p1241).
        DIRECT_CSR.write_volatile(saved | CSR_EN);
        let mut spin = SPIN_LIMIT;
        while DIRECT_CSR.read_volatile() & CSR_BUSY != 0 {
            if spin == 0 {
                ok = false;
                break;
            }
            spin -= 1;
        }
        if ok {
            M1_TIMING.write_volatile((*regs).timing);
            M1_RFMT.write_volatile((*regs).rfmt);
            M1_RCMD.write_volatile((*regs).rcmd);
            M1_WFMT.write_volatile((*regs).wfmt);
            M1_WCMD.write_volatile((*regs).wcmd);
            ATRANS4.write_volatile((*regs).atrans[0]);
            ATRANS4.add(1).write_volatile((*regs).atrans[1]);
            ATRANS4.add(2).write_volatile((*regs).atrans[2]);
            ATRANS4.add(3).write_volatile((*regs).atrans[3]);
            let ctrl = XIP_CTRL.read_volatile();
            if (*regs).writable {
                XIP_CTRL.write_volatile(ctrl | XIP_CTRL_WRITABLE_M1);
            } else {
                XIP_CTRL.write_volatile(ctrl & !XIP_CTRL_WRITABLE_M1);
            }
        }
        DIRECT_CSR.write_volatile(saved & !(CSR_EN | CSR_ASSERT_CS1N | CSR_AUTO_CS1N));
        barrier();
    }
    ok
}
