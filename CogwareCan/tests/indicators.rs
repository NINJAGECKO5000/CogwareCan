// SPDX-License-Identifier: GPL-3.0-only
//! Dashboard tell-tales: bit packing, and the trip across the bus.

use cogware_can::indicators::{self, Indicator, IndicatorSet};
use cogware_can::*;

mod common;
use common::bus;

#[test]
fn every_lamp_has_a_unique_bank_and_bit() {
    let mut seen = std::collections::HashSet::new();
    for &i in indicators::ALL_INDICATORS {
        assert!((1..=2).contains(&i.bank()), "{} is in bank {}", i.name(), i.bank());
        assert!(i.bit() < 32, "{} is bit {}", i.name(), i.bit());
        assert!(seen.insert((i.bank(), i.bit())), "{} reuses a bit", i.name());
    }
    assert_eq!(seen.len(), indicators::ALL_INDICATORS.len());
}

#[test]
fn a_set_holds_lamps_from_both_banks() {
    let mut s = IndicatorSet::new();
    assert!(s.is_empty());
    assert!(s.set(Indicator::TurnLeft, true));
    assert!(!s.set(Indicator::TurnLeft, true), "already lit");
    assert!(s.set(Indicator::Antilag, true));
    assert_eq!((s.len(), s.bank1, s.bank2), (2, 1, 1 << 2));
    assert!(s.contains(Indicator::TurnLeft) && !s.contains(Indicator::TurnRight));

    // Iteration is in table order, bank 1 before bank 2.
    assert_eq!(s.iter().collect::<Vec<_>>(), vec![Indicator::TurnLeft, Indicator::Antilag]);

    assert!(s.set(Indicator::TurnLeft, false));
    assert_eq!(s.len(), 1);
    s.clear();
    assert!(s.is_empty());
}

#[test]
fn the_top_bit_of_each_bank_survives_the_wire() {
    let _b = bus();
    // Bit 31 is the sign bit of the i32 a gauge stores, so it is the one that
    // would be lost to a widening conversion somewhere in the path.
    indicators::set(Indicator::RevLimit, true);
    indicators::set(Indicator::User11, true);
    assert_eq!(INDICATORS1.get(), Some(-1i32 << 31));

    let frames: Vec<_> = [Gauge::Indicators1, Gauge::Indicators2]
        .iter()
        .map(|g| frame_for(g.id()).expect("both banks are set"))
        .collect();

    clear_all();
    assert!(!indicators::get(Indicator::RevLimit), "cleared");
    for f in &frames {
        feed_frame(f).expect("an indicator bank is an ordinary gauge frame");
    }
    assert!(indicators::get(Indicator::RevLimit));
    assert!(indicators::get(Indicator::User11));
    assert_eq!(indicators::all().len(), 2);
}

#[test]
fn setting_one_lamp_leaves_the_rest_of_its_bank_alone() {
    let _b = bus();
    indicators::set(Indicator::CheckEngine, true);
    indicators::set(Indicator::OilPressure, true);
    indicators::set(Indicator::HighBeam, true);
    assert!(indicators::set(Indicator::OilPressure, false));
    assert!(!indicators::set(Indicator::OilPressure, false), "already out");

    assert!(indicators::get(Indicator::CheckEngine) && indicators::get(Indicator::HighBeam));
    assert!(!indicators::get(Indicator::OilPressure));
    assert_eq!(indicators::all().len(), 2);
}

#[test]
fn an_unset_bank_reads_as_all_lamps_out() {
    let _b = bus();
    // A display must not light every warning on a bus that has said nothing.
    assert!(!INDICATORS1.is_set());
    assert!(indicators::all().is_empty());
    assert!(!indicators::get(Indicator::Airbag));
}

#[test]
fn a_whole_dash_is_published_at_once() {
    let _b = bus();
    let mut wanted = IndicatorSet::new();
    wanted.set(Indicator::TurnRight, true);
    wanted.set(Indicator::LowFuel, true);
    wanted.set(Indicator::LaunchArmed, true);
    indicators::apply(wanted);

    assert_eq!(indicators::all(), wanted);
    assert!(INDICATORS1.is_set() && INDICATORS2.is_set());

    indicators::clear_all();
    assert!(indicators::all().is_empty());
    assert!(INDICATORS1.is_set(), "cleared means zero, not unset");
}

#[test]
fn the_banks_sit_in_the_gauge_range_and_answer_to_lookup() {
    for g in [&INDICATORS1, &INDICATORS2] {
        assert!(protocol::is_gauge_range(g.id));
        assert_eq!(g.wire.len(), 4, "a bank is 32 bits wide");
        assert_eq!(gauge_by_id(g.id).map(|d| d.name), Some(g.name));
        // A display subscribes to a bank exactly as it does to any gauge.
        assert!(g.id <= 0xFF);
    }
}
