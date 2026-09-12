// SPDX-License-Identifier: GPL-3.0-only
//! Dashboard tell-tales: one bit each, packed into two 32-bit gauges.
//!
//! `INDICATORS1` carries the lamps a road car is expected to show and
//! `INDICATORS2` the motorsport and standalone-ECU ones. They are ordinary
//! gauge rows, so a display subscribes to them exactly as it does to `RPM`
//! and the server broadcasts them on the same path.
//!
//! ```ignore
//! set(Indicator::TurnLeft, blink);     // server
//! if get(Indicator::CheckEngine) { .. } // display, after feed_frame
//! for lit in all().iter() { draw(lit.name()); }
//! ```

use crate::{INDICATORS1, INDICATORS2};
use paste::paste;

/// Generates the `Indicator` enum, its bank and bit accessors, and the name
/// lookup, from one row per lamp.
macro_rules! indicators {
    ($($bank:literal : $name:ident = $bit:literal;)+) => {
        paste! {
            /// One dashboard tell-tale. The discriminant is `bank << 8 | bit`,
            /// so a value stays stable as lamps are added around it.
            #[repr(u16)]
            #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
            pub enum Indicator {
                $(
                    #[doc = concat!("`", stringify!($name), "`, bank ", stringify!($bank), " bit ", stringify!($bit), ".")]
                    [<$name:camel>] = ($bank << 8) | $bit,
                )+
            }

            /// Every indicator in table order.
            pub static ALL_INDICATORS: &[Indicator] = &[$(Indicator::[<$name:camel>]),+];

            impl Indicator {
                /// Its name as written in the table, for a display listing lamps.
                pub const fn name(self) -> &'static str {
                    match self {
                        $(Indicator::[<$name:camel>] => stringify!($name),)+
                    }
                }
            }
        }
    };
}

impl Indicator {
    /// Which bank holds it: 1 for `INDICATORS1`, 2 for `INDICATORS2`.
    pub const fn bank(self) -> u8 {
        (self as u16 >> 8) as u8
    }

    /// Its bit position within its bank, 0..=31.
    pub const fn bit(self) -> u8 {
        (self as u16 & 0xFF) as u8
    }

    /// Its bit as a mask over its bank's word.
    pub const fn mask(self) -> u32 {
        1u32 << self.bit()
    }
}

// Bank 1: the lamps a road car is expected to show. Bank 2: motorsport,
// standalone-ECU state, and room to grow. Bits are never renumbered once
// shipped, because a display built against an older table would light the
// wrong lamp rather than fail.
indicators! {
    1: TURN_LEFT           = 0;
    1: TURN_RIGHT          = 1;
    1: HAZARD              = 2;
    1: HIGH_BEAM           = 3;
    1: LOW_BEAM            = 4;
    1: SIDE_LIGHTS         = 5;
    1: FOG_FRONT           = 6;
    1: FOG_REAR            = 7;
    1: CHECK_ENGINE        = 8;
    1: OIL_PRESSURE        = 9;
    1: BATTERY             = 10;
    1: COOLANT_TEMP        = 11;
    1: BRAKE               = 12;
    1: PARK_BRAKE          = 13;
    1: ABS                 = 14;
    1: TRACTION_CONTROL    = 15;
    1: STABILITY_CONTROL   = 16;
    1: STABILITY_OFF       = 17;
    1: AIRBAG              = 18;
    1: SEATBELT            = 19;
    1: DOOR_AJAR           = 20;
    1: BONNET              = 21;
    1: BOOT                = 22;
    1: LOW_FUEL            = 23;
    1: CRUISE_ON           = 24;
    1: CRUISE_ENGAGED      = 25;
    1: GLOW_PLUG           = 26;
    1: DPF                 = 27;
    1: WASHER_FLUID        = 28;
    1: TYRE_PRESSURE       = 29;
    1: SHIFT_LIGHT         = 30;
    1: REV_LIMIT           = 31;

    2: LAUNCH_ARMED        = 0;
    2: LAUNCH_ACTIVE       = 1;
    2: ANTILAG             = 2;
    2: NITROUS_ARMED       = 3;
    2: NITROUS_ACTIVE      = 4;
    2: BOOST_LIMIT         = 5;
    2: ENGINE_PROTECT      = 6;
    2: OVERHEAT            = 7;
    2: KNOCK               = 8;
    2: LEAN_WARN           = 9;
    2: FAN                 = 10;
    2: FUEL_PUMP           = 11;
    2: WATER_INJECTION     = 12;
    2: FLEX_ACTIVE         = 13;
    2: CLOSED_LOOP         = 14;
    2: IDLE_CONTROL        = 15;
    2: GEAR_SHIFT_CUT      = 16;
    2: PIT_LIMITER         = 17;
    2: SD_LOGGING          = 18;
    2: ECU_LINK            = 19;
    2: BUS_ERROR           = 20;
    2: USER1               = 21;
    2: USER2               = 22;
    2: USER3               = 23;
    2: USER4               = 24;
    2: USER5               = 25;
    2: USER6               = 26;
    2: USER7               = 27;
    2: USER8               = 28;
    2: USER9               = 29;
    2: USER10              = 30;
    2: USER11              = 31;
}

/// Both banks as one value, for building a whole dash state before publishing
/// it or for reading one consistent snapshot back.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct IndicatorSet {
    /// Bits of `INDICATORS1`.
    pub bank1: u32,
    /// Bits of `INDICATORS2`.
    pub bank2: u32,
}

impl IndicatorSet {
    /// A set with every lamp off.
    pub const fn new() -> Self {
        IndicatorSet { bank1: 0, bank2: 0 }
    }

    /// True if `ind` is lit.
    pub const fn contains(&self, ind: Indicator) -> bool {
        let word = if ind.bank() == 1 { self.bank1 } else { self.bank2 };
        word & ind.mask() != 0
    }

    /// Light or extinguish `ind`. Returns true if it changed.
    pub fn set(&mut self, ind: Indicator, on: bool) -> bool {
        let word = if ind.bank() == 1 { &mut self.bank1 } else { &mut self.bank2 };
        let next = if on { *word | ind.mask() } else { *word & !ind.mask() };
        let changed = next != *word;
        *word = next;
        changed
    }

    /// Extinguish every lamp.
    pub fn clear(&mut self) {
        self.bank1 = 0;
        self.bank2 = 0;
    }

    /// True if no lamp is lit.
    pub const fn is_empty(&self) -> bool {
        self.bank1 == 0 && self.bank2 == 0
    }

    /// How many lamps are lit.
    pub const fn len(&self) -> usize {
        (self.bank1.count_ones() + self.bank2.count_ones()) as usize
    }

    /// The lit lamps, in table order.
    pub fn iter(&self) -> impl Iterator<Item = Indicator> + '_ {
        ALL_INDICATORS.iter().copied().filter(|i| self.contains(*i))
    }
}

/// Light or extinguish one lamp in the gauge table. Returns true if it changed.
pub fn set(ind: Indicator, on: bool) -> bool {
    let gauge = if ind.bank() == 1 { &INDICATORS1 } else { &INDICATORS2 };
    let word = gauge.get_or(0) as u32;
    let next = if on { word | ind.mask() } else { word & !ind.mask() };
    gauge.set(next as i32);
    next != word
}

/// True if `ind` is lit. An unset bank reads as all lamps off.
pub fn get(ind: Indicator) -> bool {
    all().contains(ind)
}

/// Both banks as they stand.
pub fn all() -> IndicatorSet {
    IndicatorSet {
        bank1: INDICATORS1.get_or(0) as u32,
        bank2: INDICATORS2.get_or(0) as u32,
    }
}

/// Publish a whole dash state at once, so a display never sees a half-updated
/// bank ordering.
pub fn apply(set: IndicatorSet) {
    INDICATORS1.set(set.bank1 as i32);
    INDICATORS2.set(set.bank2 as i32);
}

/// Extinguish every lamp, leaving both banks set to zero.
pub fn clear_all() {
    apply(IndicatorSet::new());
}
