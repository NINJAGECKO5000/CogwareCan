// SPDX-License-Identifier: GPL-3.0-only
//! Unit conversion on readings.

use cogware_can::*;

mod common;
use common::bus;

fn close(a: Option<f32>, b: f32) -> bool {
    a.is_some_and(|a| (a - b).abs() < 0.01)
}

#[test]
fn pressure_conversions() {
    let _bus = bus();
    MAP.set(1000); // 100.0 kPa
    let r = MAP.reading().unwrap();
    assert!(close(r.kpa(), 100.0));
    assert!(close(r.psi(), 14.5038));
    assert!(close(r.bar(), 1.0));
    assert!(close(r.inhg(), 29.53));
    assert!(close(MAP.to(Unit::PSI), 14.5038));
    assert_eq!(r.celsius(), None, "pressure is not a temperature");
    assert_eq!(r.rpm(), None);
}

#[test]
fn temperature_conversions() {
    let _bus = bus();
    CLNT.set(875); // 87.5 °C
    let r = CLNT.reading().unwrap();
    assert!(close(r.celsius(), 87.5));
    assert!(close(r.fahrenheit(), 189.5));
    assert!(close(r.kelvin(), 360.65));
    CLNT.set(-400);
    assert!(close(CLNT.reading().unwrap().fahrenheit(), -40.0));
    assert_eq!(CLNT.to(Unit::PSI), None);
}

#[test]
fn speed_voltage_mixture_duration() {
    let _bus = bus();
    VSS.set(1000); // 100 km/h
    assert!(close(VSS.to(Unit::MPH), 62.137));
    BAT_VOL.set(13_800);
    assert!(close(BAT_VOL.reading().unwrap().volts(), 13.8));
    assert!(close(BAT_VOL.to(Unit::MILLIVOLT), 13_800.0));
    AFR_PRI.set(1470);
    let r = AFR_PRI.reading().unwrap();
    assert!(close(r.afr(), 14.7));
    assert!(close(r.lambda(), 1.0));
    assert!(close(r.lambda_for(9.8), 1.5));
    PULSE_WIDTH1.set(2750);
    assert!(close(PULSE_WIDTH1.reading().unwrap().ms(), 2.75));
    assert!(close(PULSE_WIDTH1.to(Unit::MICROSECONDS), 2750.0));
    STA_TIME.set(120);
    assert!(close(STA_TIME.to(Unit::MINUTES), 2.0));
}

#[test]
fn volume_conversions() {
    let _bus = bus();
    FUEL_VOLUME.set(378); // 37.8 L
    let r = FUEL_VOLUME.reading().unwrap();
    assert!(close(r.litres(), 37.8));
    assert!(close(r.gallons(), 9.986));
    assert!(close(r.gallons_uk(), 8.315));
    assert_eq!(r.psi(), None);
}

#[test]
fn raw_and_unset_do_not_convert() {
    let _bus = bus();
    STA_STATUS1.set(0xFF);
    assert_eq!(STA_STATUS1.to(Unit::RAW), None);
    assert_eq!(STA_STATUS1.reading().unwrap().value(), 255.0);
    OIL_TEMP.clear();
    assert_eq!(OIL_TEMP.reading(), None);
    assert_eq!(OIL_TEMP.to(Unit::FAHRENHEIT), None);
}

#[test]
fn unit_convert_is_symmetric() {
    for (a, b, v) in [
        (Unit::CELSIUS, Unit::FAHRENHEIT, 37.0),
        (Unit::KPA, Unit::PSI, 250.0),
        (Unit::KMH, Unit::MPH, 88.0),
        (Unit::AFR, Unit::LAMBDA, 12.5),
    ] {
        let there = a.convert(v, b).unwrap();
        let back = b.convert(there, a).unwrap();
        assert!((back - v).abs() < 0.001, "{} -> {} -> back", a.symbol, b.symbol);
    }
}

#[test]
fn distance_conversions() {
    let _bus = bus();
    ODOMETER.set(1_234_567); // 123456.7 km
    let r = ODOMETER.reading().unwrap();
    assert!(close(r.kilometres(), 123_456.7));
    assert!(close(r.miles(), 76_712.44));
    // A distance is not a speed, however alike the units read.
    assert_eq!(r.kmh(), None);
    assert_eq!(r.mph(), None);
    assert_eq!(r.litres(), None);
}

#[test]
fn gps_speed_converts_like_any_other_speed() {
    let _bus = bus();
    GPS_SPEED.set(880); // 88.0 km/h
    assert!(close(GPS_SPEED.to(Unit::MPH), 54.68));
    assert_eq!(GPS_SPEED.unit, VSS.unit, "so a scene can swap one for the other");
    // Distance is a different dimension, so it does not convert.
    assert_eq!(GPS_SPEED.to(Unit::KILOMETRES), None);
}
