//! The seam between the driver and the hardware behind QMI chip select 1.
//!
//! A [`QmiBus`] knows nothing about the PSRAM. It clocks one direct-mode transaction on chip
//! select 1, writes the window registers it is handed, flushes the XIP cache and says where the
//! window is. Choosing commands, checking the ID, computing the timing and encoding the
//! registers all happen in the driver above, so that logic runs and is tested on the host.
//!
//! # Rules every implementation follows
//!
//! - [`QmiBus::transfer`] drives CS1 low for exactly that one transaction and returns with CS1
//!   high and the memory window usable again. The window is never unavailable outside a call.
//! - [`QmiBus::program_window`] waits until the QMI is idle before it writes, and touches only the
//!   registers of window 1 and the window-1 write enable.
//! - Neither call sends anything to the chip on its own account.
//! - A wait is never open-ended. Each one ends in success or in [`BusError::Timeout`].

use crate::timing::{DirectClock, WindowRegs};

/// Data width of a direct-mode transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Width {
    /// One bit per clock: out on SD0, in on SD1.
    Single,
    /// Four bits per clock on SD0-3, high nibble first.
    Quad,
}

/// Why a bus operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusError {
    /// The QMI did not become idle, or did not finish shifting, in the time allowed.
    Timeout,
    /// The transaction is longer than the implementation accepts.
    TooLong,
}

/// QMI chip select 1: direct-mode transactions and the memory window.
///
/// # Safety
///
/// [`crate::Psram`] hands out the window as a Rust slice, so an implementation promises: once
/// [`QmiBus::program_window`] has returned `Ok` with register values that suit the chip on CS1,
/// the 16 MiB starting at [`QmiBus::window_base`] behave as ordinary memory backed by that chip,
/// no other Rust reference covers them, and this lasts until the next call to `transfer` or
/// `program_window`. `window_base` returns the same address every time.
#[allow(unsafe_code)]
pub unsafe trait QmiBus {
    /// Runs one transaction in direct mode: CS1 low, `write` shifted out (outputs driven at quad
    /// width), then `read.len()` bytes shifted in (outputs released at quad width), CS1 high.
    ///
    /// Wait cycles before read data are read bytes the caller discards: at quad width one byte
    /// is two cycles.
    fn transfer(
        &mut self,
        clock: DirectClock,
        width: Width,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), BusError>;

    /// Writes the window-1 registers and the window-1 write enable.
    fn program_window(&mut self, regs: &WindowRegs) -> Result<(), BusError>;

    /// Writes back every dirty XIP cache line, so the chip holds everything written through the
    /// cached alias and no later write-back can reach the chip.
    fn flush_cache(&mut self);

    /// Address of window 1 as the driver hands it out.
    fn window_base(&self) -> usize;
}
