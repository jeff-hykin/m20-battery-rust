//! The Deep Robotics M20's two battery packs, as the ECAN board reports them once a second (frame type 0x87 on the
//! motor bus, decoded by [`m20_ecan`]).
//!
//! Each pack is one 258-byte `ecan::BatterySensorData` (vendor `data_structures.h`, `#pragma pack(2)`, little-endian).
//! Its fields are the JBD-style BMS "basic info" (command 0x03) and cell voltages (0x04) the board relays:
//!
//! ```text
//! 0   u16 voltage (10 mV)            2   i16 current (10 mA, + = charging)  4   u16 remaining capacity (10 mAh)
//! 6   u16 nominal capacity (10 mAh)  8   u16 cycles                         10  u16 production date (JBD packed)
//! 12  u16 balance bits, cells 1-16   14  u16 balance bits, cells 17-32      16  u16 protection state bits
//! 18  u8 software version   19 u8 level (%)   20 u8 MOS state   21 u8 cells in series   22 u8 NTC count   (23 pad)
//! 24  f32 x 4 temperatures (deg C)   40 [u8; 32] serial number               72  u16 x 20 cell voltages (mV)
//! 112 u16 charger input "ch+" (10 mV)  114 u8 charge-detect input  115 reserved to 258
//! ```
//!
//! ```
//! # let frame = include_bytes!("../tests/fixtures/sitting_docked_20261120_125557.pcapng");
//! # let frames = m20_ecan::tap::read_capture(frame).unwrap();
//! # let battery_frame = frames.iter().find(|f| f.2[19] == 0x87).unwrap().2;
//! let report = m20_battery::BatteryReport::from_ethernet(battery_frame).unwrap();
//! assert_eq!(report.left.level_percent, 90);
//! assert!(report.left.voltage_v > 80.0);
//! ```

pub mod charge;

use m20_ecan::message::{Direction, Message, BATTERY_PACK_LEN};
use m20_ecan::FrameError;

/// Cell voltage slots in a record.
pub const CELL_SLOTS: usize = 20;
/// Temperature slots in a record.
pub const TEMPERATURE_SLOTS: usize = 4;

/// The JBD protection-state bits.
pub const PROTECTION_NAMES: [&str; 13] = [
    "cell overvoltage",
    "cell undervoltage",
    "pack overvoltage",
    "pack undervoltage",
    "charge overtemperature",
    "charge undertemperature",
    "discharge overtemperature",
    "discharge undertemperature",
    "charge overcurrent",
    "discharge overcurrent",
    "short circuit",
    "front-end IC error",
    "MOS software lock",
];

/// A pack's production date as the BMS stores it: `(year - 2000) << 9 | month << 5 | day`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProductionDate {
    pub year: u16,
    pub month: u8,
    pub day: u8,
}

impl ProductionDate {
    pub fn from_packed(packed: u16) -> Self {
        ProductionDate { year: 2000 + (packed >> 9), month: ((packed >> 5) & 0x0f) as u8, day: (packed & 0x1f) as u8 }
    }
}

/// One pack.
#[derive(Clone, Debug, PartialEq)]
pub struct BatteryPack {
    pub voltage_v: f32,
    /// Positive while charging.
    pub current_a: f32,
    /// As the vendor header defines it (10 mAh units); it does not track `level_percent` closely on this robot.
    pub remaining_capacity_ah: f32,
    pub nominal_capacity_ah: f32,
    pub cycles: u16,
    pub production_date: ProductionDate,
    /// Bit n set = cell n+1 balancing (cells 1-32).
    pub balancing: u32,
    /// [`PROTECTION_NAMES`] bits.
    pub protection_state: u16,
    pub software_version: u8,
    /// The BMS's state of charge.
    pub level_percent: u8,
    /// Bit 0 charge MOS on, bit 1 discharge MOS on.
    pub mos_state: u8,
    /// Cells in series (20 on the M20).
    pub cell_count: u8,
    /// Temperature sensors the BMS has (the record holds the first four).
    pub ntc_count: u8,
    pub temperatures_c: [f32; TEMPERATURE_SLOTS],
    /// Printable ASCII up to the first NUL (empty on this robot).
    pub serial_number: String,
    /// All 20 slots; the first `cell_count` are cells.
    pub cell_voltages_mv: [u16; CELL_SLOTS],
    /// Voltage on the charge contacts: the charger's ~84-85 V on the dock, ~10.5 V or 0 off it.
    pub charger_input_v: f32,
    /// The BMS charge-detect input: 0 while a charger is on the contacts, 1 off the dock (active low).
    pub charge_detect: u8,
}

/// Why bytes are not a battery report.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BatteryError {
    Frame(FrameError),
    /// The frame is ECAN but not a battery report.
    NotBattery,
    /// A pack record shorter than 258 bytes.
    ShortRecord(usize),
    /// A battery frame without two packs.
    PackCount(usize),
}

impl std::fmt::Display for BatteryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Frame(error) => write!(f, "{error}"),
            Self::NotBattery => write!(f, "not a battery frame"),
            Self::ShortRecord(len) => write!(f, "battery record of {len} bytes, expected {BATTERY_PACK_LEN}"),
            Self::PackCount(count) => write!(f, "battery frame with {count} packs, expected 2"),
        }
    }
}

impl std::error::Error for BatteryError {}

impl From<FrameError> for BatteryError {
    fn from(error: FrameError) -> Self {
        Self::Frame(error)
    }
}

impl BatteryPack {
    /// Decode one 258-byte record.
    pub fn parse(record: &[u8]) -> Result<Self, BatteryError> {
        if record.len() < BATTERY_PACK_LEN {
            return Err(BatteryError::ShortRecord(record.len()));
        }
        let u16_at = |at: usize| u16::from_le_bytes([record[at], record[at + 1]]);
        let f32_at = |at: usize| f32::from_le_bytes(record[at..at + 4].try_into().unwrap());
        let serial = &record[40..72];
        let serial = &serial[..serial.iter().position(|&b| b == 0).unwrap_or(serial.len())];
        Ok(BatteryPack {
            voltage_v: u16_at(0) as f32 / 100.0,
            current_a: i16::from_le_bytes([record[2], record[3]]) as f32 / 100.0,
            remaining_capacity_ah: u16_at(4) as f32 / 100.0,
            nominal_capacity_ah: u16_at(6) as f32 / 100.0,
            cycles: u16_at(8),
            production_date: ProductionDate::from_packed(u16_at(10)),
            balancing: u16_at(12) as u32 | (u16_at(14) as u32) << 16,
            protection_state: u16_at(16),
            software_version: record[18],
            level_percent: record[19],
            mos_state: record[20],
            cell_count: record[21],
            ntc_count: record[22],
            temperatures_c: [f32_at(24), f32_at(28), f32_at(32), f32_at(36)],
            serial_number: serial.iter().filter(|b| b.is_ascii_graphic() || **b == b' ').map(|&b| b as char).collect(),
            cell_voltages_mv: std::array::from_fn(|i| u16_at(72 + 2 * i)),
            charger_input_v: u16_at(112) as f32 / 100.0,
            charge_detect: record[114],
        })
    }

    /// The cells actually present.
    pub fn cells_mv(&self) -> &[u16] {
        &self.cell_voltages_mv[..(self.cell_count as usize).min(CELL_SLOTS)]
    }

    /// The temperature sensors actually present.
    pub fn temperatures(&self) -> &[f32] {
        &self.temperatures_c[..(self.ntc_count as usize).min(TEMPERATURE_SLOTS)]
    }

    pub fn charge_mos_on(&self) -> bool {
        self.mos_state & 1 != 0
    }

    pub fn discharge_mos_on(&self) -> bool {
        self.mos_state & 2 != 0
    }

    /// Names of the protection bits that are set (unknown bits as `bit N`).
    pub fn protections(&self) -> Vec<String> {
        (0..16)
            .filter(|bit| self.protection_state & (1 << bit) != 0)
            .map(|bit| PROTECTION_NAMES.get(bit).map_or_else(|| format!("bit {bit}"), |name| name.to_string()))
            .collect()
    }

    pub fn is_charging(&self) -> bool {
        self.current_a > 0.0
    }
}

/// Both packs. "Left" is record 0 (rl_deploy's "Battery L"), the side basic_server also calls left.
#[derive(Clone, Debug, PartialEq)]
pub struct BatteryReport {
    pub left: BatteryPack,
    pub right: BatteryPack,
}

impl BatteryReport {
    /// From a decoded ECAN message.
    pub fn from_message(message: &Message) -> Result<Self, BatteryError> {
        let Message::Battery(packs) = message else { return Err(BatteryError::NotBattery) };
        match packs.as_slice() {
            [left, right] => Ok(BatteryReport { left: BatteryPack::parse(left)?, right: BatteryPack::parse(right)? }),
            other => Err(BatteryError::PackCount(other.len())),
        }
    }

    /// From a whole Ethernet frame as the shim captures it.
    pub fn from_ethernet(ethernet: &[u8]) -> Result<Self, BatteryError> {
        let (_, message) = m20_ecan::decode_ethernet(ethernet, Direction::FromBoard)?;
        Self::from_message(&message)
    }

    pub fn packs(&self) -> [&BatteryPack; 2] {
        [&self.left, &self.right]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn date() {
        assert_eq!(ProductionDate::from_packed(12670), ProductionDate { year: 2024, month: 11, day: 30 });
    }

    #[test]
    fn short_record() {
        assert_eq!(BatteryPack::parse(&[0; 100]), Err(BatteryError::ShortRecord(100)));
    }

    #[test]
    fn protections_by_name() {
        let mut record = [0u8; BATTERY_PACK_LEN];
        record[16] = 0b0000_0101;
        record[17] = 0x80;
        assert_eq!(
            BatteryPack::parse(&record).unwrap().protections(),
            ["cell overvoltage", "pack overvoltage", "bit 15"]
        );
    }

    #[test]
    fn not_battery() {
        assert_eq!(BatteryReport::from_message(&Message::ChargeEnable), Err(BatteryError::NotBattery));
    }
}
