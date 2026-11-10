//! What the driver knows about the chip: opcodes, the Read ID layout and the datasheet limits.
//!
//! Page numbers refer to the ISSI IS66/67WVS16M8FALL/BLL datasheet, Rev. A, 01/16/2025.

/// Command opcodes (Table 4.1, p10).
pub mod opcode {
    /// Read. SPI 1-1-1, no wait cycles, at most 33 MHz. In QPI mode the same as fast read.
    pub const READ: u8 = 0x03;
    /// Fast read. SPI: 8 wait cycles, 104 MHz. QPI: 4 wait cycles, 84 MHz.
    pub const FAST_READ: u8 = 0x0B;
    /// Quad IO read. 6 wait cycles, 104 MHz, in SPI (1-4-4) and QPI (4-4-4) mode.
    pub const QUAD_READ: u8 = 0xEB;
    /// Write. No wait cycles, 104 MHz.
    pub const WRITE: u8 = 0x02;
    /// Quad IO write. No wait cycles, 104 MHz.
    pub const QUAD_WRITE: u8 = 0x38;
    /// Enter QPI mode (SPI mode only).
    pub const ENTER_QPI: u8 = 0x35;
    /// Exit QPI mode (QPI mode only).
    pub const EXIT_QPI: u8 = 0xF5;
    /// Reset enable. Must be followed directly by [`RESET`] (p19).
    pub const RESET_ENABLE: u8 = 0x66;
    /// Reset to SPI standby mode, the power-up state (p19).
    pub const RESET: u8 = 0x99;
    /// Toggle the wrapped burst length between 1024 and 32 bytes (p20).
    pub const SET_BURST_LENGTH: u8 = 0xC0;
    /// Read ID. SPI: 24 don't-care address bits, no wait cycles. QPI: no address, 6 wait cycles (p18).
    pub const READ_ID: u8 = 0x9F;
    /// Deep power-down entry (p23).
    pub const DEEP_POWER_DOWN: u8 = 0xB9;
}

/// The start of the Read ID response: manufacturer, known-good-die and the first EID byte
/// (Figure 5.12 p18, Table 6.2 p25).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Id {
    /// Manufacturer ID, bits 63:56.
    pub manufacturer: u8,
    /// Known-good-die byte, bits 55:48.
    pub kgd: u8,
    /// EID bits 47:40. The top three bits are the density.
    pub eid_high: u8,
}

/// Why a Read ID response was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IdError {
    /// Every byte read 0x00, or every byte 0xFF: nothing drove the data line.
    NoResponse,
    /// The manufacturer byte is not the chip's.
    WrongManufacturer(u8),
    /// The KGD byte reads "not passed yet" (0x55): the die failed the manufacturer's test.
    FailedDie,
    /// The KGD byte is neither pass nor fail.
    UnknownKgd(u8),
    /// The density field (EID bits 47:45) is not the chip's.
    WrongDensity(u8),
}

impl Id {
    /// Builds the ID from the first three response bytes, in the order they arrive.
    pub const fn from_bytes(bytes: [u8; 3]) -> Self {
        Self {
            manufacturer: bytes[0],
            kgd: bytes[1],
            eid_high: bytes[2],
        }
    }

    /// The three bytes in the order they arrive.
    pub const fn to_bytes(self) -> [u8; 3] {
        [self.manufacturer, self.kgd, self.eid_high]
    }

    /// Device density field, EID bits 47:45.
    pub const fn density_code(self) -> u8 {
        self.eid_high >> 5
    }

    /// Density in Mbit: code 0b011 is 64 Mbit, 0b100 is 128 Mbit, the others are reserved (p25).
    pub const fn density_mbit(self) -> Option<u32> {
        match self.density_code() {
            0b011 => Some(64),
            0b100 => Some(128),
            _ => None,
        }
    }

    /// Checks the ID against what `chip` reports.
    pub fn check(self, chip: &Chip) -> Result<(), IdError> {
        let bytes = self.to_bytes();
        if bytes == [0x00; 3] || bytes == [0xFF; 3] {
            return Err(IdError::NoResponse);
        }
        if self.manufacturer != chip.manufacturer {
            return Err(IdError::WrongManufacturer(self.manufacturer));
        }
        if self.kgd == chip.kgd_fail {
            return Err(IdError::FailedDie);
        }
        if self.kgd != chip.kgd_pass {
            return Err(IdError::UnknownKgd(self.kgd));
        }
        if self.density_code() != chip.density_code {
            return Err(IdError::WrongDensity(self.density_code()));
        }
        Ok(())
    }
}

/// Datasheet parameters the driver uses. Times are maxima or minima as the datasheet states them,
/// for the temperature range of the part (up to 85 degC for the -I grade).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Chip {
    /// Part number.
    pub name: &'static str,
    /// Manufacturer ID.
    pub manufacturer: u8,
    /// KGD byte of a die that passed.
    pub kgd_pass: u8,
    /// KGD byte of a die that has not passed.
    pub kgd_fail: u8,
    /// Density field, EID bits 47:45.
    pub density_code: u8,
    /// Size in bytes.
    pub size: usize,
    /// Size of one die in bytes. No burst may cross a die boundary.
    pub die_size: usize,
    /// Every read and write wraps within a page of this many bytes.
    pub wrap_page: usize,
    /// Highest clock for 03h read (SPI).
    pub max_hz_read: u32,
    /// Highest clock for 0Bh fast read in QPI mode.
    pub max_hz_fast_read_qpi: u32,
    /// Highest clock for EBh quad IO read.
    pub max_hz_quad_read: u32,
    /// Highest clock for 38h quad IO write.
    pub max_hz_quad_write: u32,
    /// Wait cycles of EBh quad IO read.
    pub quad_read_wait_cycles: u8,
    /// Wait cycles of 9Fh Read ID in QPI mode.
    pub qpi_read_id_wait_cycles: u8,
    /// Shortest clock period, tCLK at the highest clock, in picoseconds.
    pub min_clock_period_ps: u32,
    /// CE# setup time to the first active clock edge, tCSP, in picoseconds.
    pub tcsp_ps: u32,
    /// Hold time from the last clock rising edge to CE# high, tHD, in picoseconds.
    pub thd_ps: u32,
    /// Clock falling edge to output valid, tACLK, in picoseconds.
    pub taclk_ps: u32,
    /// Output hold time from clock falling edge, tOH, in picoseconds.
    pub toh_ps: u32,
    /// CE# high between operations, tCPH, in clock periods.
    pub tcph_clocks: u8,
    /// Longest CE# low time, tCEM, in nanoseconds.
    pub tcem_ns: u32,
    /// Self-initialisation time after power-up, tPU, in microseconds.
    pub power_up_us: u32,
    /// Wait after the reset command before the next command, in microseconds.
    pub reset_us: u32,
}

impl Chip {
    /// ISSI IS66WVS16M8FBLL: 128 Mbit (16 MiB) SPI/QPI PSRAM, 2.7-3.6 V, -40 to 85 degC.
    pub const IS66WVS16M8FBLL: Chip = Chip {
        name: "IS66WVS16M8FBLL",
        // Table 6.2, p25.
        manufacturer: 0x9D,
        kgd_pass: 0x5D,
        kgd_fail: 0x55,
        density_code: 0b100,
        // p10, 4.1: addressed with A23..A0.
        size: 16 * 1024 * 1024,
        // p2 and p8: two stacked 64 Mbit dies; die boundary crossing is not allowed in a burst.
        die_size: 8 * 1024 * 1024,
        // p10, 4.2: reads and writes always wrap within 1024 bytes.
        wrap_page: 1024,
        // Table 4.1, p10.
        max_hz_read: 33_000_000,
        max_hz_fast_read_qpi: 84_000_000,
        max_hz_quad_read: 104_000_000,
        max_hz_quad_write: 104_000_000,
        // Table 4.1 p10, Figure 5.8 p16.
        quad_read_wait_cycles: 6,
        // p18, 5.7 and Figure 5.12.
        qpi_read_id_wait_cycles: 6,
        // 7.6 AC characteristics, p29.
        min_clock_period_ps: 10_000,
        tcsp_ps: 3_000,
        thd_ps: 2_000,
        taclk_ps: 7_000,
        toh_ps: 1_500,
        tcph_clocks: 1,
        tcem_ns: 4_000,
        // p9, Figure 3.1.
        power_up_us: 150,
        // The datasheet gives no recovery time for the software reset (5.8, p19); this uses the
        // power-up self-initialisation time tPU (p9), the only device-ready time it states for
        // the SPI standby state.
        reset_us: 150,
    };
}
