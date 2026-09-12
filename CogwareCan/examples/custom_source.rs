// SPDX-License-Identifier: GPL-3.0-only
//! Adding a new ECU: write tables, not code.
//!
//! This is the whole job for any CAN-broadcast ECU. Copy this file, put the
//! real offsets and scalings in, and add the source to `CAN_SOURCES`.
//!
//!     cargo run --example custom_source

use cogware_can::decode::{conv::*, CanFrameMap, CanSource, Field};
use cogware_can::{field, *};

// Suppose "Acme EFI" broadcasts two frames from a base ID of 0x640:
//
//   0x640: rpm u16 LE | MAP in kPa*4 u16 LE | coolant °C+50 u8 | TPS %*2 u8
//   0x641: battery mV u16 LE | lambda*1000 u16 LE | oil pressure psi u8

/// kPa*4 into the tenths-of-a-kPa the MAP gauge stores.
fn kpa_x4(v: i32) -> i32 {
    div_round(v * 10, 4)
}
/// °C sent with a +50 offset, into tenths of a degree.
fn celsius_plus50(v: i32) -> i32 {
    (v - 50) * 10
}
/// %*2 into tenths of a percent.
fn pct_x2(v: i32) -> i32 {
    v * 5
}

static ENGINE: &[Field] = &[
    field!(0, U16, RPM, raw),
    field!(2, U16, MAP, kpa_x4),
    field!(4, U8, CLNT, celsius_plus50),
    field!(5, U8, TPS, pct_x2),
];

static SENSORS: &[Field] = &[
    field!(0, U16, BAT_VOL, raw),          // already millivolts
    field!(2, U16, AFR_PRI, lambda_thousandths),
    field!(4, U8, OIL_PRES, psi),
];

static ACME: CanSource = CanSource {
    name: "Acme EFI",
    default_base: 0x640,
    frames: &[
        CanFrameMap { id_offset: 0, fields: ENGINE },
        CanFrameMap { id_offset: 1, fields: SENSORS },
    ],
};

fn main() {
    clear_all();

    let mut engine = 3500u16.to_le_bytes().to_vec();
    engine.extend(380u16.to_le_bytes()); // 95.0 kPa as kPa*4
    engine.extend([87 + 50, 100]); // 87 °C, 50.0 %
    let mut sensors = 13_800u16.to_le_bytes().to_vec();
    sensors.extend(898u16.to_le_bytes()); // lambda 0.898
    sensors.extend([43]); // 43 psi

    println!("wrote {:?} gauges", ACME.feed_default(&protocol::frame(0x640, &engine).unwrap()));
    println!("wrote {:?} gauges", ACME.feed_default(&protocol::frame(0x641, &sensors).unwrap()));

    println!("\n  rpm       {:>7.0}", RPM.as_f32().unwrap());
    println!("  MAP       {:>7.1} kPa", MAP.as_f32().unwrap());
    println!("  coolant   {:>7.1} °C", CLNT.as_f32().unwrap());
    println!("  throttle  {:>7.1} %", TPS.as_f32().unwrap());
    println!("  battery   {:>7.2} V", BAT_VOL.as_f32().unwrap());
    println!("  AFR       {:>7.2}", AFR_PRI.as_f32().unwrap());
    println!("  oil press {:>7.1} kPa ({:.0} psi back)", OIL_PRES.as_f32().unwrap(), OIL_PRES.to(Unit::PSI).unwrap());

    // The base ID is configurable on most ECUs; `feed` takes the one in use.
    clear_all();
    ACME.feed(0x500, &protocol::frame(0x500, &engine).unwrap());
    println!("\nrelocated to base 0x500: rpm {:?}", RPM.get());

    // Frames belonging to another protocol are left alone.
    println!("a frame at 0x360:        {:?}", ACME.feed_default(&protocol::frame(0x360, &[0; 8]).unwrap()));
}
