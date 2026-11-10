//! Driver for the ISSI IS66WVS16M8FBLL 16 MiB QSPI PSRAM on QMI chip select 1 of the RP2350.
//!
//! Four layers, one seam:
//!
//! - [`chip`]: opcodes, Read ID decoding and the datasheet limits ([`Chip::IS66WVS16M8FBLL`]).
//! - [`timing`]: the QMI window and direct-mode timing computed from clk_sys, and the register
//!   values that program the window. Pure code, tested on the host.
//! - [`Psram`]: the reset, identification and QPI sequence, and the memory window as a slice. It
//!   reaches hardware only through [`QmiBus`].
//! - A back-end that implements [`QmiBus`]: `rp2350::Rp2350Qmi` (feature `rp2350`), raw
//!   registers and no HAL.
//!
//! `init` works whether the chip is in SPI or QPI mode (it is not power-cycled between program
//! runs): it leaves QPI mode, resets the chip, waits, checks the ID in SPI mode and again in QPI
//! mode, and programs the window: EBh quad read with 6 wait cycles, 38h quad write, the fastest
//! SCK the chip allows (75 MHz at a 150 MHz clk_sys), bursts broken at the 1024-byte wrap page
//! (which also keeps them inside one 8 MiB die) and chip select limited to the 4 us tCEM. It
//! never writes the memory contents.
//!
//! The window is handed out through the uncached alias at 0x1500_0000. The cached alias at
//! 0x1100_0000 also takes writes (the XIP cache is write-back, RP2350 datasheet p340-341 and
//! p346), but on the board it is slower: 1 MiB sequential at 75 MHz writes at 7.9 MiB/s cached
//! against 20.8 MiB/s uncached (a write miss first reads the line), reads at 25.2 against 24.4
//! MiB/s, and random 4-byte accesses take longer. A cached write also stays in the cache until the
//! line is evicted or cleaned, so it is lost if the boot ROM's flash routines invalidate the
//! cache. Code that uses the cached alias anyway must clean the cache (RP2350-E11 workaround, see
//! `QmiBus::flush_cache`) before anything that resets QMI or invalidates the cache.
//!
//! ```ignore
//! let p = embassy_rp::init(Default::default());
//! // Keep these two unused for as long as the PSRAM is in use.
//! let (_qmi_cs1, _pin) = (p.QMI_CS1, p.PIN_0);
//! // SAFETY: QMI CS1 and GPIO0 are this driver's, core 1 is not running from flash.
//! let qmi = unsafe { qmi_psram::rp2350::Rp2350Qmi::new(qmi_psram::rp2350::Cs1Pin::Gpio0) };
//! let mut psram = qmi_psram::Psram::new(qmi);
//! let info = psram.init(&mut embassy_time::Delay, embassy_rp::clocks::clk_sys_freq())?;
//! let memory: &mut [u8] = psram.as_mut_slice(); // 16 MiB
//! ```
//!
//! After the QMI M1 registers were reset, for example by programming the in-package flash
//! through the boot ROM, re-program the window without touching the chip or its contents:
//!
//! ```ignore
//! psram.remap()?;
//! ```
#![cfg_attr(not(test), no_std)]
#![deny(unsafe_code)]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod bus;
pub mod chip;
#[cfg(feature = "rp2350")]
pub mod rp2350;
pub mod timing;

use embedded_hal::delay::DelayNs;

pub use bus::{BusError, QmiBus, Width};
pub use chip::{Chip, Id, IdError, opcode};
pub use timing::{Config, DirectClock, Timing, TimingError, WindowRegs};

/// The chip this driver drives.
const CHIP: Chip = Chip::IS66WVS16M8FBLL;
/// Read ID in QPI mode: wait cycles before the ID, as bytes at quad width.
const QPI_ID_WAIT_BYTES: usize = CHIP.qpi_read_id_wait_cycles as usize / 2;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The back-end failed.
    Bus(BusError),
    /// No valid timing for this clk_sys and configuration.
    Timing(TimingError),
    /// The SPI-mode Read ID does not identify the chip.
    Id(IdError),
    /// The QPI-mode Read ID differs from the SPI-mode one: a data line other than SD0/SD1 fails.
    QpiMismatch {
        /// The three ID bytes read in QPI mode.
        found: [u8; 3],
    },
    /// `remap` before a successful `init`.
    NotInitialized,
}

impl From<BusError> for Error {
    fn from(e: BusError) -> Self {
        Error::Bus(e)
    }
}

impl From<TimingError> for Error {
    fn from(e: TimingError) -> Self {
        Error::Timing(e)
    }
}

impl From<IdError> for Error {
    fn from(e: IdError) -> Self {
        Error::Id(e)
    }
}

/// What `init` found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Info {
    /// The Read ID response.
    pub id: Id,
    /// Size of the memory window in bytes.
    pub size: usize,
}

#[derive(Clone, Copy)]
struct State {
    info: Info,
    timing: Timing,
}

/// The PSRAM on QMI chip select 1.
pub struct Psram<Q> {
    qmi: Q,
    config: Config,
    state: Option<State>,
}

impl<Q: QmiBus> Psram<Q> {
    /// Wraps the back-end. Nothing happens on the bus until `init`.
    pub const fn new(qmi: Q) -> Self {
        Self {
            qmi,
            config: Config::DEFAULT,
            state: None,
        }
    }

    /// Sets an SCK cap or an RXDELAY override. Takes effect at the next `init` or `remap`.
    pub fn set_config(&mut self, config: Config) {
        self.config = config;
    }

    /// Resets and identifies the chip, puts it in QPI mode and programs the memory window.
    ///
    /// Works from SPI or QPI mode. Sends no write command, so the contents are left to the chip
    /// (the datasheet does not say whether its reset keeps them). The window must not be accessed
    /// while `init` runs. On error nothing is handed out until a later `init` succeeds.
    pub fn init(&mut self, delay: &mut impl DelayNs, clk_sys_hz: u32) -> Result<Info, Error> {
        self.state = None;
        let timing = Timing::compute(clk_sys_hz, &CHIP, &self.config)?;
        let clock = timing.direct;
        // A dirty cache line written back while the chip is reset or the window re-programmed
        // would go out in the wrong format.
        self.qmi.flush_cache();
        // QPI exit. In SPI mode the chip sees two bits of a command on SI and ignores it.
        self.qmi
            .transfer(clock, Width::Quad, &[opcode::EXIT_QPI], &mut [])?;
        // Reset enable and reset, consecutive, each in its own chip select (p19).
        self.qmi
            .transfer(clock, Width::Single, &[opcode::RESET_ENABLE], &mut [])?;
        self.qmi
            .transfer(clock, Width::Single, &[opcode::RESET], &mut [])?;
        delay.delay_us(CHIP.reset_us);
        // SPI Read ID: 24 don't-care address bits, then the ID (p18).
        let mut id = [0u8; 3];
        self.qmi
            .transfer(clock, Width::Single, &[opcode::READ_ID, 0, 0, 0], &mut id)?;
        let id = Id::from_bytes(id);
        id.check(&CHIP)?;
        self.qmi
            .transfer(clock, Width::Single, &[opcode::ENTER_QPI], &mut [])?;
        // QPI Read ID: no address, wait cycles, then the same ID on all four data lines.
        let mut qpi = [0u8; QPI_ID_WAIT_BYTES + 3];
        self.qmi
            .transfer(clock, Width::Quad, &[opcode::READ_ID], &mut qpi)?;
        let found = [
            qpi[QPI_ID_WAIT_BYTES],
            qpi[QPI_ID_WAIT_BYTES + 1],
            qpi[QPI_ID_WAIT_BYTES + 2],
        ];
        if found != id.to_bytes() {
            return Err(Error::QpiMismatch { found });
        }
        self.qmi.program_window(&timing.registers(&CHIP))?;
        let info = Info {
            id,
            size: CHIP.size,
        };
        self.state = Some(State { info, timing });
        Ok(info)
    }

    /// Re-programs the memory window after something reset the QMI M1 registers, for example
    /// programming the in-package flash through the boot ROM. Sends nothing to the chip and does
    /// not touch its contents.
    ///
    /// Uses the configuration from [`Self::set_config`], so it is also how a new SCK cap or
    /// RXDELAY takes effect. The window must not be accessed between the reset and this call.
    /// On error the window registers are left as they were.
    pub fn remap(&mut self) -> Result<(), Error> {
        let state = self.state.as_mut().ok_or(Error::NotInitialized)?;
        let timing = Timing::compute(state.timing.clk_sys_hz, &CHIP, &self.config)?;
        self.qmi.program_window(&timing.registers(&CHIP))?;
        state.timing = timing;
        Ok(())
    }

    /// What `init` found, once it succeeded.
    pub fn info(&self) -> Option<Info> {
        self.state.map(|s| s.info)
    }

    /// The timing in effect, once `init` succeeded.
    pub fn timing(&self) -> Option<Timing> {
        self.state.map(|s| s.timing)
    }

    /// Size of the memory window in bytes; 0 before `init` succeeded.
    pub fn len(&self) -> usize {
        self.state.map_or(0, |s| s.info.size)
    }

    /// True before `init` succeeded.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The memory window; empty before `init` succeeded.
    #[allow(unsafe_code)]
    pub fn as_slice(&self) -> &[u8] {
        let len = self.len();
        if len == 0 {
            return &[];
        }
        // SAFETY: `init` succeeded, so `program_window` returned Ok with registers computed for
        // this chip, and by the `QmiBus` contract the `len` bytes at `window_base` are ordinary
        // memory no other reference covers. The shared borrow of `self` keeps `init`, `remap` and
        // `release` from running while the slice lives.
        unsafe { core::slice::from_raw_parts(self.qmi.window_base() as *const u8, len) }
    }

    /// The memory window, writable; empty before `init` succeeded.
    #[allow(unsafe_code)]
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        let len = self.len();
        if len == 0 {
            return &mut [];
        }
        // SAFETY: as in `as_slice`; the exclusive borrow of `self` makes this the only reference.
        unsafe { core::slice::from_raw_parts_mut(self.qmi.window_base() as *mut u8, len) }
    }

    /// Start of the memory window, for an allocator; null before `init` succeeded. The memory is
    /// valid for [`Self::len`] bytes until the next `init`, `remap` or `release`, and must not be
    /// accessed while one of those runs.
    pub fn base(&self) -> *mut u8 {
        if self.is_empty() {
            core::ptr::null_mut()
        } else {
            self.qmi.window_base() as *mut u8
        }
    }

    /// Gives the back-end back. The chip stays in QPI mode and the window stays programmed.
    pub fn release(self) -> Q {
        self.qmi
    }
}

#[cfg(test)]
mod tests;
