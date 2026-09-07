//! Units and conversions.
//!
//! Every gauge stores fixed-point counts in one canonical `Unit`. A `Reading`
//! pairs those counts with their unit and converts on request:
//!
//! ```ignore
//! MAP.reading()?.psi()          // Some(2.9) for 20 kPa gauge... None if unset
//! CLNT.reading()?.fahrenheit()
//! MAP.reading()?.celsius()      // None: wrong dimension
//! MAP.to(Unit::BAR)             // generic form, straight from the gauge
//! ```
//!
//! Internally each dimension has a base (kPa, °C, km/h, V, AFR, ms, ...) and
//! each unit is `base = value * factor + offset`.

/// Physical dimension of a unit. Conversions only happen within a dimension.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quantity {
    /// Bitfields, counters, enumerations: never converted.
    Raw,
    Time,
    Frequency,
    AngularSpeed,
    AngularSpeedRate,
    Temperature,
    Pressure,
    PressureRate,
    Percent,
    PercentRate,
    Voltage,
    /// Air-fuel ratio. Base is gasoline AFR; lambda assumes stoich 14.7.
    Mixture,
    Angle,
    Duration,
    Speed,
    Data,
    Volume,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unit {
    pub symbol: &'static str,
    /// Stored counts per one unit when this is a gauge's canonical unit.
    pub scale: i32,
    pub quantity: Quantity,
    /// `base = value * factor + offset`
    factor: f32,
    offset: f32,
}

impl Unit {
    const fn base(symbol: &'static str, scale: i32, quantity: Quantity) -> Self {
        Unit { symbol, scale, quantity, factor: 1.0, offset: 0.0 }
    }
    const fn derived(symbol: &'static str, quantity: Quantity, factor: f32, offset: f32) -> Self {
        Unit { symbol, scale: 1, quantity, factor, offset }
    }

    // Canonical (base) units. The `scale` here is what gauges store in.
    pub const RAW: Unit = Unit::base("", 1, Quantity::Raw);
    pub const SECONDS: Unit = Unit::base("s", 1, Quantity::Time);
    pub const HZ: Unit = Unit::base("Hz", 1, Quantity::Frequency);
    pub const RPM: Unit = Unit::base("rpm", 1, Quantity::AngularSpeed);
    pub const RPM_PER_S: Unit = Unit::base("rpm/s", 1, Quantity::AngularSpeedRate);
    pub const CELSIUS: Unit = Unit::base("°C", 10, Quantity::Temperature);
    pub const KPA: Unit = Unit::base("kPa", 10, Quantity::Pressure);
    pub const KPA_PER_S: Unit = Unit::base("kPa/s", 10, Quantity::PressureRate);
    pub const PERCENT: Unit = Unit::base("%", 10, Quantity::Percent);
    pub const PERCENT_PER_S: Unit = Unit::base("%/s", 10, Quantity::PercentRate);
    pub const VOLT: Unit = Unit::base("V", 1000, Quantity::Voltage);
    pub const AFR: Unit = Unit::base("AFR", 100, Quantity::Mixture);
    /// Crank degrees (advance, cam angle, injection angle).
    pub const DEGREES: Unit = Unit::base("°", 10, Quantity::Angle);
    /// Milliseconds stored as microseconds (pulse width, dwell).
    pub const MS: Unit = Unit::base("ms", 1000, Quantity::Duration);
    pub const KMH: Unit = Unit::base("km/h", 10, Quantity::Speed);
    pub const BYTES: Unit = Unit::base("B", 1, Quantity::Data);
    pub const LITRES: Unit = Unit::base("L", 10, Quantity::Volume);

    // Display-only units, reachable through conversion.
    pub const FAHRENHEIT: Unit = Unit::derived("°F", Quantity::Temperature, 5.0 / 9.0, -160.0 / 9.0);
    pub const KELVIN: Unit = Unit::derived("K", Quantity::Temperature, 1.0, -273.15);
    pub const PSI: Unit = Unit::derived("psi", Quantity::Pressure, 6.894_757, 0.0);
    pub const BAR: Unit = Unit::derived("bar", Quantity::Pressure, 100.0, 0.0);
    pub const INHG: Unit = Unit::derived("inHg", Quantity::Pressure, 3.386_39, 0.0);
    pub const MPH: Unit = Unit::derived("mph", Quantity::Speed, 1.609_344, 0.0);
    pub const LAMBDA: Unit = Unit::derived("λ", Quantity::Mixture, 14.7, 0.0);
    pub const MILLIVOLT: Unit = Unit::derived("mV", Quantity::Voltage, 0.001, 0.0);
    pub const MICROSECONDS: Unit = Unit::derived("µs", Quantity::Duration, 0.001, 0.0);
    pub const MINUTES: Unit = Unit::derived("min", Quantity::Time, 60.0, 0.0);
    pub const GALLONS_US: Unit = Unit::derived("gal", Quantity::Volume, 3.785_411_8, 0.0);
    pub const GALLONS_UK: Unit = Unit::derived("gal (UK)", Quantity::Volume, 4.546_09, 0.0);

    /// Convert a value expressed in `self` to `target`, or `None` across dimensions.
    pub fn convert(&self, value: f32, target: Unit) -> Option<f32> {
        if self.quantity != target.quantity || self.quantity == Quantity::Raw {
            return None;
        }
        let base = value * self.factor + self.offset;
        Some((base - target.offset) / target.factor)
    }

    /// Fixed-point counts in `self` to a value in `target`.
    pub fn convert_counts(&self, counts: i32, target: Unit) -> Option<f32> {
        self.convert(counts as f32 / self.scale as f32, target)
    }
}

/// A gauge value with its unit attached.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    pub counts: i32,
    pub unit: Unit,
}

macro_rules! conversions {
    ($($fn:ident => $unit:ident),+ $(,)?) => {
        $(
            #[doc = concat!("Value in ", stringify!($unit), ", or `None` if the dimension differs.")]
            pub fn $fn(&self) -> Option<f32> {
                self.to(Unit::$unit)
            }
        )+
    };
}

impl Reading {
    /// Value in the gauge's own unit.
    pub fn value(&self) -> f32 {
        self.counts as f32 / self.unit.scale as f32
    }

    pub fn to(&self, target: Unit) -> Option<f32> {
        self.unit.convert_counts(self.counts, target)
    }

    conversions! {
        celsius => CELSIUS, fahrenheit => FAHRENHEIT, kelvin => KELVIN,
        kpa => KPA, psi => PSI, bar => BAR, inhg => INHG,
        kmh => KMH, mph => MPH,
        volts => VOLT, millivolts => MILLIVOLT,
        afr => AFR, lambda => LAMBDA,
        ms => MS, microseconds => MICROSECONDS,
        seconds => SECONDS, minutes => MINUTES,
        rpm => RPM, percent => PERCENT, degrees => DEGREES,
        litres => LITRES, gallons => GALLONS_US, gallons_uk => GALLONS_UK,
    }

    /// Lambda for a fuel with a different stoichiometric ratio (E85 is about 9.8).
    pub fn lambda_for(&self, stoich: f32) -> Option<f32> {
        self.afr().map(|afr| afr / stoich)
    }
}
