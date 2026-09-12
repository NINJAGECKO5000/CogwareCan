// SPDX-License-Identifier: GPL-3.0-only
//! The gauge spec: reading values, units, and what "unset" means.
//!
//!     cargo run --example gauges

use cogware_can::*;

fn main() {
    // Nothing has written anything yet, so every gauge is unset. Unset is
    // distinct from zero: 0 rpm is a real reading, "no reading" is None.
    clear_all();
    println!("before any source: RPM = {:?}", RPM.get());
    println!("with a fallback:   RPM = {}", RPM.get_or(0));

    // A converter normally does this. Values are fixed point: each gauge
    // stores `physical value * unit.scale`.
    RPM.set(3500);
    MAP.set(1450); // 145.0 kPa, scale 10
    CLNT.set(875); // 87.5 °C
    AFR_PRI.set(1265); // 12.65 AFR, scale 100
    VSS.set(880); // 88.0 km/h
    BAT_VOL.set(13_800); // 13.800 V, scale 1000

    println!("\nreadings in canonical units");
    for g in [&RPM, &MAP, &CLNT, &AFR_PRI, &VSS, &BAT_VOL] {
        // as_f32 divides out the scale; the unit symbol comes from the spec.
        println!("  {:<8} {:>8.2} {}", g.name, g.as_f32().unwrap(), g.unit.symbol);
    }

    // Conversions are checked against the gauge's physical dimension.
    println!("\nthe same values in other units");
    println!("  MAP      {:>8.2} psi", MAP.to(Unit::PSI).unwrap());
    println!("  MAP      {:>8.3} bar", MAP.to(Unit::BAR).unwrap());
    println!("  CLNT     {:>8.1} °F", CLNT.to(Unit::FAHRENHEIT).unwrap());
    println!("  VSS      {:>8.1} mph", VSS.to(Unit::MPH).unwrap());
    println!("  AFR_PRI  {:>8.3} lambda", AFR_PRI.reading().unwrap().lambda().unwrap());
    // E85 has a different stoichiometric ratio, so lambda differs.
    println!("  AFR_PRI  {:>8.3} lambda on E85", AFR_PRI.reading().unwrap().lambda_for(9.8).unwrap());

    // Asking for the wrong dimension gives None rather than a wrong number.
    println!("\nMAP in °C:      {:?}", MAP.to(Unit::CELSIUS));
    println!("unset gauge:    {:?}", OIL_TEMP.to(Unit::FAHRENHEIT));

    // Source says which kind of ECU can supply a gauge, so a display knows
    // whether a missing value is a fault or simply unsupported.
    println!("\ngauges by source tier");
    for tier in [Source::Common, Source::Standalone, Source::Speeduino, Source::Protocol] {
        let names: Vec<&str> = ALL_GAUGES.iter().filter(|g| g.source == tier).map(|g| g.name).collect();
        println!("  {:?} ({}): {}", tier, names.len(), names[..5.min(names.len())].join(", "));
    }
    println!("\ntotal gauges in the spec: {}", ALL_GAUGES.len());
}
