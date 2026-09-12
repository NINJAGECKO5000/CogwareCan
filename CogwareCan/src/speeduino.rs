// SPDX-License-Identifier: GPL-3.0-only
//! Speeduino serial converter: turns a realtime-data packet from the ECU's
//! TunerStudio serial protocol into canonical gauge values.
//!
//! Field offsets and scalings follow the `[OutputChannels]` section of
//! `reference/speeduino.ini`. The realtime layout has shifted several times
//! between releases (notably 201905 and 202108), so the table is selected by
//! `Layout`. Read the firmware signature with the `Q` command and pick the
//! matching layout; `Layout::LATEST` is master (202504-dev, ochBlockSize 139).
//!
//! Two packet shapes are understood:
//!
//! * `n` command reply: `'n', 0x32, len, data[len]`  (see `parse_n`)
//! * `A` command reply: `'A', data[..]`               (see `parse_a`)
//!
//! Both carry the same `data` layout, indexed from zero in the field tables.

use crate::decode::{apply_fields, conv::*, Field};
use crate::field;

/// Header bytes preceding the payload in an `n` reply.
pub const N_HEADER_LEN: usize = 3;
/// Header bytes preceding the payload in an `A` reply.
pub const A_HEADER_LEN: usize = 1;
/// Largest `n` reply the firmware can send (length byte is a u8).
pub const MAX_N_PACKET_LEN: usize = N_HEADER_LEN + u8::MAX as usize;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeeduinoError {
    /// Packet is shorter than its header.
    Truncated,
    /// First byte is not the expected command echo.
    BadCommand(u8),
    /// `n` reply format byte was not 0x32.
    BadFormat(u8),
    /// The header promised more payload bytes than the packet holds.
    ShortPayload { declared: usize, got: usize },
}

/// ini scale 2.0 kPa into tenths of a kPa.
fn kpa_x2(v: i32) -> i32 {
    v * 20
}
/// Realtime layout of `speeduino.ini` on master (202504-dev), ochBlockSize 139.
/// Offsets 42..=73 are the 16 CAN-in words; 3 is the sync-loss counter;
/// 121 onward (EMAP, fan duty, knock, pulse widths 5-8) have no gauges yet.
static FIELDS_202504: &[Field] = &[
    field!(0, U8, STA_TIME, raw),
    field!(1, U8, STA_STATUS1, raw),
    field!(2, U8, STA_ENG, raw),
    field!(4, U16, MAP, x10),
    field!(6, U8, IAT, celsius_plus40),
    field!(7, U8, CLNT, celsius_plus40),
    field!(8, U8, BAT_CORRECT, x10),
    field!(9, U8, BAT_VOL, tenths_to_thousandths),
    field!(10, U8, AFR_PRI, tenths_to_hundredths),
    field!(11, U8, EGO_CORRECT, centred100_to_trim_tenths),
    field!(12, U8, IAT_CORRECT, x10),
    field!(13, U8, WUE_CORRECT, x10),
    field!(14, U16, RPM, raw),
    field!(16, U8, ACCEL_ENRICH, x20),
    field!(17, U16, GAMME_E, x10),
    field!(19, U8, VE1, x10),
    field!(20, U8, VE2, x10),
    field!(21, U8, AFR_TARGET, tenths_to_hundredths),
    field!(22, S16, TPS_DOT, x10),
    field!(24, S8, CUR_SPARK_ADVANCE, x10),
    field!(25, U8, TPS, half),
    field!(26, U16, LOOP_PS, raw),
    field!(28, U16, FREE_MEM, raw),
    field!(30, U8, BOOST_TARGET, kpa_x2),
    field!(31, U8, BOOST_PWM, x10),
    field!(32, U8, STA_SPARK, raw),
    field!(33, S16, RPM_DOT, raw),
    field!(35, U8, ETHANOL_PERCENT, x10),
    field!(36, U8, FLEX_CORRECT, x10),
    field!(37, S8, FLEX_IGN_CORRECT, x10),
    field!(38, U8, IDLE_LOAD, raw),
    field!(39, U8, TEST_OUTPUTS, raw),
    field!(40, U8, AFR_SEC, tenths_to_hundredths),
    field!(41, U8, BARO, x10),
    field!(74, U8, TPS_ADC, raw),
    field!(75, U8, NEXT_ERROR, raw),
    field!(76, U16, PULSE_WIDTH1, raw),
    field!(78, U16, PULSE_WIDTH2, raw),
    field!(80, U16, PULSE_WIDTH3, raw),
    field!(82, U16, PULSE_WIDTH4, raw),
    field!(84, U8, STA_STATUS2, raw),
    field!(85, U8, ENG_PROTECT_STA, raw),
    field!(86, S16, FUEL_LOAD, raw),
    field!(88, S16, IGN_LOAD, raw),
    field!(90, U16, DWELL, raw),
    field!(92, U8, CL_IDLE_TARGET, x10),
    field!(93, S16, MAP_DOT, x10),
    field!(95, S16, VVT_ANGLE, half),
    field!(97, U8, VVT_TARGET_ANGLE, half),
    field!(98, U8, VVT_DUTY, half),
    field!(99, S16, FLEX_BOOST_CORRECT, x10),
    field!(101, U8, BARO_CORRECTION, x10),
    field!(102, U8, VE, x10),
    field!(103, U8, ASE, x10),
    field!(104, U16, VSS, x10),
    field!(106, U8, GEAR, raw),
    field!(107, U8, FUEL_PRES, psi),
    field!(108, U8, OIL_PRES, psi),
    field!(109, U8, WMI_PW, x10),
    field!(110, U8, STA_STATUS4, raw),
    field!(111, S16, VVT_ANGLE2, half),
    field!(113, U8, VVT_TARGET_ANGLE2, half),
    field!(114, U8, VVT_DUTY2, half),
    field!(115, U8, STATUS_OUT_STA, raw),
    field!(116, U8, FLEX_FUEL_TEMP, celsius_plus40),
    field!(117, U8, FUEL_TEMP_CORRECT, x10),
    field!(118, S8, ADVANCE1, x10),
    field!(119, S8, ADVANCE2, x10),
    field!(120, U8, SD_STA, raw),
];

/// Which firmware's realtime layout to decode with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// `speeduino.ini` on master, signature "speeduino 202504-dev".
    /// Also matches 202402 for every field mapped here.
    V202504,
}

impl Layout {
    pub const LATEST: Layout = Layout::V202504;

    pub fn fields(self) -> &'static [Field] {
        match self {
            Layout::V202504 => FIELDS_202504,
        }
    }

    /// Payload bytes the firmware sends (`ochBlockSize`).
    pub const fn payload_len(self) -> usize {
        match self {
            Layout::V202504 => 139,
        }
    }
}

/// Payload length of `Layout::LATEST`.
pub const FULL_PAYLOAD_LEN: usize = Layout::LATEST.payload_len();

/// Write every field present in `data` (a header-less realtime payload) into
/// its gauge. Fields beyond the end of `data` are left untouched, so a short
/// payload from older firmware still fills what it carries.
/// Returns how many gauges were written.
pub fn apply_realtime(data: &[u8]) -> usize {
    apply_realtime_with(Layout::LATEST, data)
}

/// `apply_realtime` for a specific firmware layout.
pub fn apply_realtime_with(layout: Layout, data: &[u8]) -> usize {
    apply_fields(layout.fields(), data)
}

/// Validate an `n` reply header and return the payload length it declares.
pub fn n_payload_len(header: &[u8]) -> Result<usize, SpeeduinoError> {
    if header.len() < N_HEADER_LEN {
        return Err(SpeeduinoError::Truncated);
    }
    if header[0] != b'n' {
        return Err(SpeeduinoError::BadCommand(header[0]));
    }
    if header[1] != 0x32 {
        return Err(SpeeduinoError::BadFormat(header[1]));
    }
    Ok(header[2] as usize)
}

/// Parse a complete `n` reply (header included) into the gauges.
/// Returns how many gauges were written.
pub fn parse_n(packet: &[u8]) -> Result<usize, SpeeduinoError> {
    parse_n_with(Layout::LATEST, packet)
}

/// `parse_n` for a specific firmware layout.
pub fn parse_n_with(layout: Layout, packet: &[u8]) -> Result<usize, SpeeduinoError> {
    let declared = n_payload_len(packet)?;
    let body = &packet[N_HEADER_LEN..];
    if body.len() < declared {
        return Err(SpeeduinoError::ShortPayload { declared, got: body.len() });
    }
    Ok(apply_realtime_with(layout, &body[..declared]))
}

/// Parse a complete `A` reply (header included) into the gauges.
/// The `A` reply has no length byte, so everything after the echo is payload.
pub fn parse_a(packet: &[u8]) -> Result<usize, SpeeduinoError> {
    parse_a_with(Layout::LATEST, packet)
}

/// `parse_a` for a specific firmware layout.
pub fn parse_a_with(layout: Layout, packet: &[u8]) -> Result<usize, SpeeduinoError> {
    match packet.first() {
        None => Err(SpeeduinoError::Truncated),
        Some(b'A') => Ok(apply_realtime_with(layout, &packet[A_HEADER_LEN..])),
        Some(&other) => Err(SpeeduinoError::BadCommand(other)),
    }
}
