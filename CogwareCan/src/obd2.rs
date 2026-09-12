// SPDX-License-Identifier: GPL-3.0-only
//! OBD2 (ISO 15765-4 / SAE J1979) mode 01 converter.
//!
//! OBD2 is request/response, not broadcast: the master sends `request(pid)`
//! on 0x7DF, the car's ECU answers on 0x7E8..=0x7EF, and `feed_response`
//! decodes the answer. Poll `PIDS` in turn; a car may not support every one
//! (PID 0x00 and friends report support bitmaps, not decoded here).
//!
//! Response layout: `[len, 0x41, pid, A, B, C, D]`, big-endian, and the
//! formulas below are the J1979 ones with `A` at field offset 0.

use crate::decode::{apply_fields, conv::*, Field};
use crate::field;
use crate::protocol::{frame, id_of};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

/// Functional request ID every ECU listens on.
pub const REQUEST_ID: u16 = 0x7DF;
pub const RESPONSE_ID_MIN: u16 = 0x7E8;
pub const RESPONSE_ID_MAX: u16 = 0x7EF;
const MODE_CURRENT_DATA: u8 = 0x01;
const MODE_RESPONSE: u8 = 0x41;

pub struct Pid {
    pub pid: u8,
    pub fields: &'static [Field],
}

/// A*100/255 into tenths of a percent.
fn pct255(v: i32) -> i32 {
    div_round(v * 1000, 255)
}
/// (256A+B)/4 rpm.
fn quarter(v: i32) -> i32 {
    div_round(v, 4)
}
/// A/2 - 64 degrees into tenths.
fn half_minus64(v: i32) -> i32 {
    v * 5 - 640
}
/// A/1.28 - 100 percent trim into tenths.
fn trim128(v: i32) -> i32 {
    div_round(v * 1000, 128) - 1000
}
/// (256A+B)/32768 lambda into hundredths of AFR.
fn lambda32768(v: i32) -> i32 {
    div_round(v * 1470, 32768)
}
/// A bit 7 is the MIL lamp; the low bits count stored DTCs.
fn dtc_count(v: i32) -> i32 {
    v & 0x7F
}
/// A*3 kPa gauge into tenths.
fn kpa_x3(v: i32) -> i32 {
    v * 30
}
/// (256B+C)/10 - 40 °C into tenths.
fn egt(v: i32) -> i32 {
    v - 400
}

static P01: &[Field] = &[field!(0, U8, ERROR_COUNT, dtc_count)];
static P04: &[Field] = &[field!(0, U8, FUEL_LOAD, pct255)];
static P05: &[Field] = &[field!(0, U8, CLNT, celsius_plus40)];
static P06: &[Field] = &[field!(0, U8, EGO_CORRECT, trim128)];
static P0A: &[Field] = &[field!(0, U8, FUEL_PRES, kpa_x3)];
static P0B: &[Field] = &[field!(0, U8, MAP, x10)];
static P0C: &[Field] = &[field!(0, U16BE, RPM, quarter)];
static P0D: &[Field] = &[field!(0, U8, VSS, x10)];
static P0E: &[Field] = &[field!(0, U8, CUR_SPARK_ADVANCE, half_minus64)];
static P0F: &[Field] = &[field!(0, U8, IAT, celsius_plus40)];
static P11: &[Field] = &[field!(0, U8, TPS, pct255)];
static P1F: &[Field] = &[field!(0, U16BE, STA_TIME, raw)];
static P23: &[Field] = &[field!(0, U16BE, FUEL_PRES, x100)];
static P2F: &[Field] = &[field!(0, U8, FUEL_LEVEL, pct255)];
static P33: &[Field] = &[field!(0, U8, BARO, x10)];
static P42: &[Field] = &[field!(0, U16BE, BAT_VOL, raw)];
static P44: &[Field] = &[field!(0, U16BE, AFR_TARGET, lambda32768)];
static P52: &[Field] = &[field!(0, U8, ETHANOL_PERCENT, pct255)];
static P5A: &[Field] = &[field!(0, U8, PEDAL, pct255)];
static P5C: &[Field] = &[field!(0, U8, OIL_TEMP, celsius_plus40)];
static P78: &[Field] = &[field!(1, U16BE, EGT1, egt)];
// 0.1 km/bit, which is what an ODOMETER count already is.
static PA6: &[Field] = &[field!(0, U32BE, ODOMETER, raw)];

/// Every PID this converter decodes, in a sensible polling order.
pub static PIDS: &[Pid] = &[
    Pid { pid: 0x0C, fields: P0C },
    Pid { pid: 0x0B, fields: P0B },
    Pid { pid: 0x11, fields: P11 },
    Pid { pid: 0x0E, fields: P0E },
    Pid { pid: 0x05, fields: P05 },
    Pid { pid: 0x0F, fields: P0F },
    Pid { pid: 0x0D, fields: P0D },
    Pid { pid: 0x42, fields: P42 },
    Pid { pid: 0x44, fields: P44 },
    Pid { pid: 0x06, fields: P06 },
    Pid { pid: 0x04, fields: P04 },
    Pid { pid: 0x33, fields: P33 },
    Pid { pid: 0x0A, fields: P0A },
    Pid { pid: 0x23, fields: P23 },
    Pid { pid: 0x2F, fields: P2F },
    Pid { pid: 0x52, fields: P52 },
    Pid { pid: 0x5A, fields: P5A },
    Pid { pid: 0x5C, fields: P5C },
    Pid { pid: 0x78, fields: P78 },
    Pid { pid: 0x1F, fields: P1F },
    Pid { pid: 0x01, fields: P01 },
    Pid { pid: 0xA6, fields: PA6 },
];

/// Mode 01 request for `pid`.
pub fn request(pid: u8) -> Option<CanFrame> {
    frame(REQUEST_ID, &[0x02, MODE_CURRENT_DATA, pid, 0, 0, 0, 0, 0])
}

pub fn is_response(f: &CanFrame) -> bool {
    matches!(id_of(f), Some(id) if (RESPONSE_ID_MIN..=RESPONSE_ID_MAX).contains(&id))
}

/// Decode a mode 01 single-frame response. Returns the number of gauges
/// written, or `None` if the frame is not a response to a PID known here.
pub fn feed_response(f: &CanFrame) -> Option<usize> {
    if !is_response(f) {
        return None;
    }
    let d = &f.data()[..f.dlc()];
    if d.len() < 3 || d[1] != MODE_RESPONSE {
        return None;
    }
    let len = d[0] as usize;
    if !(2..=7).contains(&len) {
        return None;
    }
    let end = (1 + len).min(d.len());
    let payload = &d[3..end];
    let pid = PIDS.iter().find(|p| p.pid == d[2])?;
    Some(apply_fields(pid.fields, payload))
}
