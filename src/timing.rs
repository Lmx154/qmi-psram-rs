//! QMI timing for chip select 1, computed from clk_sys and the chip limits, and the register
//! values that program the memory window.
//!
//! RP2350 page numbers refer to the RP2350 datasheet, section 12.14 (QMI). The chip limits come
//! from [`Chip`].
//!
//! What holds by construction for every [`Timing`] that [`Timing::compute`] returns:
//!
//! - SCK is at most the chip's limit for the commands the window uses (EBh read, 38h write) and
//!   at most the optional cap.
//! - The longest CE# low time of the window, [`Timing::select_low_max_cycles`], is at most tCEM.
//!   It counts MAX_SELECT, one unit of MAX_SELECT granularity and a whole access in flight when
//!   MAX_SELECT expires, the worst case the RP2350 datasheet names (p1241).
//! - The longest direct-mode transaction `init` issues fits in tCEM with an allowance for the
//!   back-end's own instructions.
//! - CE# high between accesses lasts at least tCPH plus the CS1 pad skew.
//! - CE# setup to the first SCK edge meets tCSP, and CE# hold after the last edge meets tHD,
//!   with the worst-case output delays of the CS1 GPIO pad and the QSPI pads.

use crate::chip::Chip;

/// Picoseconds per second.
const PS: u64 = 1_000_000_000_000;
/// Nanoseconds per second.
const NS: u64 = 1_000_000_000;

/// System clock to QSPI output (SCK, SD0-3), worst case at VDDIO = 3.3 V (Table 1292, p1232).
const QSPI_OUT_MAX_PS: u64 = 2_500;
/// QSPI input to system clock, worst case at VDDIO = 3.3 V (Table 1292, p1232).
const QSPI_IN_MAX_PS: u64 = 1_500;
/// System clock to bank 0 GPIO output on the QFN-80, where CS1n is, worst case (Table 1292, p1233).
const GPIO_OUT_MAX_PS: u64 = 4_100;
/// Bank 0 GPIO (QFN-80) output skew, maximum (Table 1291, p1232). Margin on the CE# high time.
const GPIO_SKEW_MAX_PS: u64 = 2_100;
/// MAX_SELECT and COOLDOWN count in units of 64 clk_sys cycles (p1239, p1241).
const SELECT_UNIT: u64 = 64;
/// Longest direct-mode transaction that `init` issues, in SCK cycles: the SPI Read ID, four
/// bytes out and three in at one bit per cycle.
const DIRECT_LONGEST_SCK: u64 = 56;
/// Allowance, in clk_sys cycles, for the back-end's own instructions between asserting and
/// releasing CS1 in direct mode. On the board (psram_smoke P10) the RP2350 back-end's Read ID
/// keeps CE# low 1347 ns at 150 MHz: 747 ns of SCK plus 90 cycles.
const DIRECT_SOFTWARE_CYCLES: u64 = 128;

/// Quad width code of the RFMT/WFMT width fields (p1242-1246).
const WIDTH_QUAD: u32 = 2;

/// Optional limits for the timing calculation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Lower cap on SCK in hertz, for example when the board cannot carry the chip's fastest
    /// clock. `None`: the chip's limit.
    pub max_sck_hz: Option<u32>,
    /// RXDELAY of the window in half clk_sys cycles, 0..=7. `None`: [`default_rx_delay`].
    pub rx_delay: Option<u8>,
}

impl Config {
    /// No cap, computed RXDELAY.
    pub const DEFAULT: Config = Config {
        max_sck_hz: None,
        rx_delay: None,
    };
}

/// Clock of a direct-mode transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DirectClock {
    /// SCK divisor, 1..=256.
    pub clkdiv: u16,
    /// Read sample delay in half clk_sys cycles, 0..=3.
    pub rx_delay: u8,
}

impl DirectClock {
    /// The RXDELAY (bits 31:30) and CLKDIV (bits 29:22) fields of DIRECT_CSR (p1237). A divisor
    /// of 256 is encoded as 0.
    pub const fn csr_bits(self) -> u32 {
        ((self.rx_delay as u32 & 0x3) << 30) | ((self.clkdiv as u32 & 0xFF) << 22)
    }
}

/// Why no timing could be computed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimingError {
    /// clk_sys or the SCK cap is zero.
    NoClock,
    /// clk_sys is more than 256 times the allowed SCK.
    ClockDivider,
    /// The RXDELAY override is above 7.
    RxDelay,
    /// No SELECT_SETUP or SELECT_HOLD meets tCSP or tHD.
    ChipSelectEdge,
    /// The CE# high minimum needs more than 31 extra cycles.
    Deselect,
    /// Not even a single access, or the longest direct-mode transaction, fits in tCEM.
    ChipSelectLow,
    /// The chip's wrap page or wait cycles have no QMI encoding.
    Format,
}

/// Register values for QMI window 1: M1_TIMING, M1_RFMT, M1_RCMD, M1_WFMT, M1_WCMD, ATRANS4..7
/// and XIP_CTRL.WRITABLE_M1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WindowRegs {
    /// M1_TIMING (p1239).
    pub timing: u32,
    /// M1_RFMT (p1242).
    pub rfmt: u32,
    /// M1_RCMD (p1243).
    pub rcmd: u32,
    /// M1_WFMT (p1244).
    pub wfmt: u32,
    /// M1_WCMD (p1246).
    pub wcmd: u32,
    /// ATRANS4..ATRANS7: address translation of window 1 (p1246).
    pub atrans: [u32; 4],
    /// XIP_CTRL.WRITABLE_M1: writes to window 1 allowed (p346).
    pub writable: bool,
}

/// Identity translation of window 1: pane k maps 4 MiB at physical offset k x 4 MiB. SIZE (bits
/// 26:16) and BASE (bits 11:0) count 4 KiB units (p1246).
const ATRANS_IDENTITY: [u32; 4] = [
    0x400 << 16,
    (0x400 << 16) | 0x400,
    (0x400 << 16) | 0x800,
    (0x400 << 16) | 0xC00,
];

impl WindowRegs {
    /// The values after a QMI reset (p1239-1247): 03h serial read, 02h serial write, CLKDIV 4,
    /// COOLDOWN 1, identity translation, window read-only.
    pub const RESET: WindowRegs = WindowRegs {
        timing: 0x4000_0004,
        rfmt: 0x0000_1000,
        rcmd: 0x0000_A003,
        wfmt: 0x0000_1000,
        wcmd: 0x0000_A002,
        atrans: ATRANS_IDENTITY,
        writable: false,
    };
}

/// QMI timing of window 1 and of direct mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timing {
    /// The clk_sys frequency it was computed for.
    pub clk_sys_hz: u32,
    /// SCK divisor, 1..=256.
    pub clkdiv: u16,
    /// Read sample delay in half clk_sys cycles, 0..=7.
    pub rx_delay: u8,
    /// Chip select stays low this many 64-cycle units after an access, waiting for a sequential
    /// one (0 or 1).
    pub cooldown: u8,
    /// Bursts break at this power-of-two boundary in bytes (256, 1024 or 4096).
    pub page_break: u16,
    /// One extra clk_sys cycle between CE# low and the first SCK edge (0 or 1).
    pub select_setup: u8,
    /// Extra clk_sys cycles of CE# hold after the last SCK edge, 0..=3.
    pub select_hold: u8,
    /// Longest CE# low in units of 64 clk_sys cycles, 1..=63; 0 when `cooldown` is 0.
    pub max_select: u8,
    /// Extra clk_sys cycles of CE# high on top of half an SCK period, 0..=31.
    pub min_deselect: u8,
    /// Upper bound of any CE# low time of the window, in clk_sys cycles.
    pub select_low_max_cycles: u32,
    /// Clock of the direct-mode transactions of `init`.
    pub direct: DirectClock,
}

/// RXDELAY that samples in the middle of the data-valid window, in half clk_sys cycles.
///
/// The chip launches each bit tACLK after the SCK falling edge and holds the previous one for
/// tOH (p29). Seen from the QMI's sampling register, data becomes valid at most
/// `QSPI out + tACLK + QSPI in` after the internal falling edge and stays valid until at least tOH
/// after the next one. The QMI samples at the internal rising edge plus RXDELAY (p1241), half an
/// SCK period after the falling edge, so the centre of the window is RXDELAY =
/// `(QSPI out + tACLK + QSPI in + tOH) / clk_sys period`, independent of the divisor. Rounded to
/// the nearest step and clamped to the 3-bit field.
pub fn default_rx_delay(clk_sys_hz: u32, chip: &Chip) -> u8 {
    let window_ps =
        QSPI_OUT_MAX_PS + u64::from(chip.taclk_ps) + QSPI_IN_MAX_PS + u64::from(chip.toh_ps);
    let half_cycles = (window_ps * u64::from(clk_sys_hz) + PS / 2) / PS;
    half_cycles.min(7) as u8
}

/// `cycles` clk_sys cycles in picoseconds, rounded down.
fn cycles_ps(cycles: u64, clk: u64) -> u64 {
    cycles * PS / clk
}

/// `ps` picoseconds in clk_sys cycles, rounded up.
fn ps_cycles(ps: u64, clk: u64) -> u64 {
    (ps * clk).div_ceil(PS)
}

impl Timing {
    /// Computes the timing for `clk_sys_hz`, the fastest SCK `chip` and `config` allow.
    pub fn compute(clk_sys_hz: u32, chip: &Chip, config: &Config) -> Result<Self, TimingError> {
        let clk = u64::from(clk_sys_hz);
        let mut limit = u64::from(chip.max_hz_quad_read.min(chip.max_hz_quad_write));
        if let Some(cap) = config.max_sck_hz {
            limit = limit.min(u64::from(cap));
        }
        if clk == 0 || limit == 0 {
            return Err(TimingError::NoClock);
        }
        let clkdiv = clk.div_ceil(limit);
        if clkdiv > 256 {
            return Err(TimingError::ClockDivider);
        }
        let rx_delay = match config.rx_delay {
            Some(r) if r <= 7 => r,
            Some(_) => return Err(TimingError::RxDelay),
            None => default_rx_delay(clk_sys_hz, chip),
        };
        let page_break = match chip.wrap_page {
            256 => 256,
            1024 => 1024,
            4096 => 4096,
            _ => return Err(TimingError::Format),
        };
        if chip.quad_read_wait_cycles > 7 {
            return Err(TimingError::Format);
        }

        // CE# goes low half an SCK period before the first rising edge (p1231), plus SELECT_SETUP.
        // CS1n is a GPIO pad and may lag SCK by its whole output delay.
        let half_sck_ps = clkdiv * PS / (2 * clk);
        let select_setup = (0..=1u64)
            .find(|&s| {
                (half_sck_ps + cycles_ps(s, clk)).saturating_sub(GPIO_OUT_MAX_PS)
                    >= u64::from(chip.tcsp_ps)
            })
            .ok_or(TimingError::ChipSelectEdge)?;
        // CE# stays low one clk_sys cycle plus SELECT_HOLD after the last falling edge (p1240),
        // which is half an SCK period after the last rising edge. SCK may lag CE# by the QSPI
        // pad's output delay.
        let select_hold = (0..=3u64)
            .find(|&h| {
                (half_sck_ps + cycles_ps(1 + h, clk)).saturating_sub(QSPI_OUT_MAX_PS)
                    >= u64::from(chip.thd_ps)
            })
            .ok_or(TimingError::ChipSelectEdge)?;

        // CE# high lasts half an SCK period rounded up plus MIN_DESELECT cycles (p1241). It must
        // cover tCPH clock periods (p29) plus the skew of the CS1 pad.
        let half_sck_cycles = clkdiv.div_ceil(2);
        let tcph = u64::from(chip.tcph_clocks);
        let high_cycles = (tcph * clkdiv)
            .max(ps_cycles(tcph * u64::from(chip.min_clock_period_ps), clk))
            + ps_cycles(GPIO_SKEW_MAX_PS, clk);
        let min_deselect = high_cycles.saturating_sub(half_sck_cycles);
        if min_deselect > 31 {
            return Err(TimingError::Deselect);
        }

        // The longest access: 8-bit prefix, 24-bit address, wait cycles and one 64-bit cache line,
        // all at quad width, plus chip-select setup and hold. Hold may stretch to two cycles after
        // the last sample (p1240).
        let wait = u64::from(chip.quad_read_wait_cycles);
        let read_sck = 2 + 6 + wait + 16;
        let write_sck = 2 + 6 + 16;
        let in_flight = read_sck.max(write_sck) * clkdiv
            + half_sck_cycles
            + select_setup
            + 1
            + select_hold
            + 2
            + u64::from(rx_delay).div_ceil(2);
        let tcem_cycles = u64::from(chip.tcem_ns) * clk / NS;
        let (cooldown, max_select, select_low_max) = if tcem_cycles >= in_flight + 2 * SELECT_UNIT {
            let units = ((tcem_cycles - in_flight - SELECT_UNIT) / SELECT_UNIT).min(63);
            (1, units, units * SELECT_UNIT + SELECT_UNIT + in_flight)
        } else if in_flight <= tcem_cycles {
            // Too slow to chain: every access gets its own chip select.
            (0, 0, in_flight)
        } else {
            return Err(TimingError::ChipSelectLow);
        };

        let direct = DirectClock {
            clkdiv: clkdiv as u16,
            rx_delay: rx_delay.min(3),
        };
        if DIRECT_LONGEST_SCK * clkdiv + DIRECT_SOFTWARE_CYCLES > tcem_cycles {
            return Err(TimingError::ChipSelectLow);
        }

        Ok(Self {
            clk_sys_hz,
            clkdiv: clkdiv as u16,
            rx_delay,
            cooldown,
            page_break,
            select_setup: select_setup as u8,
            select_hold: select_hold as u8,
            max_select: max_select as u8,
            min_deselect: min_deselect as u8,
            select_low_max_cycles: select_low_max as u32,
            direct,
        })
    }

    /// SCK frequency in hertz, rounded down.
    pub const fn sck_hz(&self) -> u32 {
        self.clk_sys_hz / self.clkdiv as u32
    }

    /// MAX_SELECT x 64 / clk_sys in nanoseconds, rounded up.
    pub fn max_select_ns(&self) -> u32 {
        self.cycles_ns(u64::from(self.max_select) * SELECT_UNIT)
    }

    /// [`Self::select_low_max_cycles`] in nanoseconds, rounded up.
    pub fn select_low_max_ns(&self) -> u32 {
        self.cycles_ns(u64::from(self.select_low_max_cycles))
    }

    /// Shortest CE# high time between accesses in nanoseconds, rounded down.
    pub fn deselect_ns(&self) -> u32 {
        let cycles = u64::from(self.clkdiv).div_ceil(2) + u64::from(self.min_deselect);
        (cycles * NS / u64::from(self.clk_sys_hz)) as u32
    }

    fn cycles_ns(&self, cycles: u64) -> u32 {
        (cycles * NS).div_ceil(u64::from(self.clk_sys_hz)) as u32
    }

    /// The register values for this timing: EBh quad read with 8-bit prefix and the chip's wait
    /// cycles as dummy bits, 38h quad write, identity translation, writes enabled.
    pub fn registers(&self, chip: &Chip) -> WindowRegs {
        let page_break = match self.page_break {
            256 => 1,
            1024 => 2,
            4096 => 3,
            _ => 0,
        };
        // M1_TIMING (p1239-1241).
        let timing = (u32::from(self.cooldown & 0x3) << 30)
            | (page_break << 28)
            | (u32::from(self.select_setup & 0x1) << 25)
            | (u32::from(self.select_hold & 0x3) << 23)
            | (u32::from(self.max_select & 0x3F) << 17)
            | (u32::from(self.min_deselect & 0x1F) << 12)
            | (u32::from(self.rx_delay & 0x7) << 8)
            | (u32::from(self.clkdiv) & 0xFF);
        // QPI mode: every phase at quad width, 8-bit prefix, no suffix (p1242-1246). DUMMY_LEN
        // counts 4-bit units, one quad-width cycle each.
        let quad = WIDTH_QUAD
            | (WIDTH_QUAD << 2)
            | (WIDTH_QUAD << 4)
            | (WIDTH_QUAD << 6)
            | (WIDTH_QUAD << 8)
            | (1 << 12);
        let dummy_len = u32::from(chip.quad_read_wait_cycles & 0x7);
        WindowRegs {
            timing,
            rfmt: quad | (dummy_len << 16),
            rcmd: u32::from(crate::opcode::QUAD_READ),
            wfmt: quad,
            wcmd: u32::from(crate::opcode::QUAD_WRITE),
            atrans: ATRANS_IDENTITY,
            writable: true,
        }
    }
}
