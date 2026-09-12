// SPDX-License-Identifier: GPL-3.0-only
//! Settings: the descriptor table, the blob, and what an editor sees.

use cogware_can::config::{self, ConfigError, Kind};
use cogware_can::units::Unit;
use std::collections::HashSet;

mod common;
use common::settings as table;

#[test]
fn every_key_is_unique_and_fits_a_record() {
    let mut seen = HashSet::new();
    for s in config::ALL_SETTINGS {
        assert!(seen.insert(s.key), "{} is used twice", s.key);
        assert!(!s.key.is_empty() && s.key.len() <= config::KEY_MAX);
        assert!(
            s.key.bytes().all(|b| b.is_ascii_lowercase() || b == b'_' || b.is_ascii_digit()),
            "{} is not a plain lowercase key",
            s.key
        );
    }
}

#[test]
fn the_table_describes_itself_consistently() {
    for s in config::ALL_SETTINGS {
        assert!(s.min <= s.default && s.default <= s.max, "{}", s.key);
        assert!(!s.doc.is_empty(), "{} has no documentation", s.key);
        match s.kind {
            // A choice's range must index its labels exactly, or an editor
            // offers an option that stores a value the node will clamp away.
            Kind::Choice => {
                assert_eq!((s.min, s.max as usize), (0, s.choices.len() - 1), "{}", s.key);
            }
            Kind::Toggle => {
                assert_eq!((s.min, s.max), (0, 1), "{}", s.key);
                assert!(s.choices.is_empty(), "{}", s.key);
            }
            Kind::Number => assert!(s.choices.is_empty(), "{}", s.key),
        }
    }
}

#[test]
fn a_value_is_clamped_rather_than_refused() {
    let _t = table();
    assert!(config::BRIGHTNESS.is_default());
    assert!(!config::BRIGHTNESS.set(500), "in range, nothing clamped");
    assert_eq!(config::BRIGHTNESS.get(), 500);
    assert!(!config::BRIGHTNESS.is_default());

    assert!(config::BRIGHTNESS.set(5000), "over max is clamped");
    assert_eq!(config::BRIGHTNESS.get(), config::BRIGHTNESS.max);
    assert!(config::BRIGHTNESS.set(-1));
    assert_eq!(config::BRIGHTNESS.get(), config::BRIGHTNESS.min);

    config::BRIGHTNESS.reset();
    assert!(config::BRIGHTNESS.is_default());
}

#[test]
fn values_read_back_in_physical_units() {
    let _t = table();
    // Fixed point in the row's unit, exactly as a gauge stores it.
    assert_eq!(config::WARN_COOLANT.as_f32(), 105.0);
    assert_eq!(config::WARN_COOLANT.unit, Unit::CELSIUS);
    assert_eq!(config::BRIGHTNESS.as_f32(), 80.0);

    config::WARN_COOLANT.set(1150);
    assert_eq!(config::WARN_COOLANT.as_f32(), 115.0);

    assert_eq!(config::UNITS_TEMP.choice(), Some("Celsius"));
    config::UNITS_TEMP.set(1);
    assert_eq!(config::UNITS_TEMP.choice(), Some("Fahrenheit"));

    assert!(!config::DEMO_MODE.is_on());
    config::DEMO_MODE.set(1);
    assert!(config::DEMO_MODE.is_on());
}

#[test]
fn a_blob_round_trips_every_setting() {
    let _t = table();
    config::BRIGHTNESS.set(350);
    config::SHIFT_LIGHT_RPM.set(5800);
    config::UNITS_SPEED.set(1);
    config::DEMO_MODE.set(1);

    let mut buf = vec![0u8; config::blob_len()];
    let n = config::encode(&mut buf).unwrap();
    assert_eq!(n, config::blob_len());
    assert_eq!(&buf[..4], &config::MAGIC);

    config::reset_all();
    assert_eq!(config::changed_count(), 0);

    let applied = config::decode(&buf[..n]).unwrap();
    assert_eq!(applied.applied, config::ALL_SETTINGS.len());
    assert_eq!((applied.skipped, applied.clamped), (0, 0));
    assert_eq!(config::BRIGHTNESS.get(), 350);
    assert_eq!(config::SHIFT_LIGHT_RPM.get(), 5800);
    assert_eq!(config::UNITS_SPEED.choice(), Some("mph"));
    assert!(config::DEMO_MODE.is_on());
    assert_eq!(config::changed_count(), 4);
}

#[test]
fn a_key_this_build_lacks_is_skipped_not_fatal() {
    let _t = table();
    // What a newer editor sends: one known key, one this build never had.
    let mut blob = Vec::new();
    blob.extend_from_slice(&config::MAGIC);
    blob.push(config::VERSION);
    blob.push(0);
    blob.extend_from_slice(&2u16.to_le_bytes());
    for (key, value) in [("brightness", 425i32), ("holographic_tint", 7)] {
        blob.push(key.len() as u8);
        blob.extend_from_slice(key.as_bytes());
        blob.extend_from_slice(&value.to_le_bytes());
    }

    let applied = config::decode(&blob).unwrap();
    assert_eq!((applied.applied, applied.skipped), (1, 1));
    assert_eq!(config::BRIGHTNESS.get(), 425);
}

#[test]
fn an_out_of_range_value_in_a_blob_is_reported() {
    let _t = table();
    let mut blob = Vec::new();
    blob.extend_from_slice(&config::MAGIC);
    blob.push(config::VERSION);
    blob.push(0);
    blob.extend_from_slice(&1u16.to_le_bytes());
    blob.push(b"brightness".len() as u8);
    blob.extend_from_slice(b"brightness");
    blob.extend_from_slice(&99_999i32.to_le_bytes());

    let applied = config::decode(&blob).unwrap();
    assert_eq!((applied.applied, applied.clamped), (1, 1));
    assert_eq!(config::BRIGHTNESS.get(), config::BRIGHTNESS.max);
}

#[test]
fn a_malformed_blob_is_refused_rather_than_half_applied() {
    let _t = table();
    let mut buf = vec![0u8; config::blob_len()];
    let n = config::encode(&mut buf).unwrap();

    assert_eq!(config::decode(&[]), Err(ConfigError::BadMagic));
    assert_eq!(config::decode(b"NOPE1234"), Err(ConfigError::BadMagic));

    let mut wrong = buf.clone();
    wrong[4] = config::VERSION + 1;
    assert_eq!(config::decode(&wrong), Err(ConfigError::BadVersion(config::VERSION + 1)));

    // A record that runs off the end is a truncation, not a silent stop.
    assert_eq!(config::decode(&buf[..n - 2]), Err(ConfigError::Truncated));

    // encode refuses a buffer it cannot fill rather than writing a short blob.
    let mut small = [0u8; 8];
    assert_eq!(
        config::encode(&mut small),
        Err(ConfigError::TooSmall { needed: config::blob_len(), got: 8 })
    );
}

#[test]
fn a_blob_rides_the_transfer_path_as_a_config_image() {
    use cogware_can::xfer::{kind, BufferSink, Image, Receiver, Sender, TxState};

    let _t = table();
    config::WARN_OIL_PRESSURE.set(1500);
    let mut blob = vec![0u8; config::blob_len()];
    let n = config::encode(&mut blob).unwrap();

    let mut store = vec![0u8; n];
    let mut tx = Sender::new(4, &blob[..n], kind::CONFIG, 16, 1);
    let mut rx = Receiver::new(4, BufferSink::new(&mut store));
    for _ in 0..4096 {
        if let Some(f) = tx.next_frame() {
            if let Some(reply) = rx.feed(&f) {
                tx.feed_reply(&reply);
            }
        }
        if tx.state() == TxState::Done {
            break;
        }
    }
    assert_eq!(tx.state(), TxState::Done);
    assert_eq!(rx.image(), Image { size: n as u32, kind: kind::CONFIG, slot: 0 });

    config::reset_all();
    let received = rx.sink().buf.to_vec();
    assert_eq!(config::decode(&received).unwrap().applied, config::ALL_SETTINGS.len());
    assert_eq!(config::WARN_OIL_PRESSURE.get(), 1500);
}

#[test]
fn the_description_is_json_an_editor_can_parse() {
    let _t = table();
    config::BRIGHTNESS.set(640);

    let mut json = String::new();
    config::describe(&mut json).unwrap();

    assert!(json.starts_with('[') && json.ends_with(']'));
    // Every setting appears once, with the live value alongside the default.
    for s in config::ALL_SETTINGS {
        let key = format!(r#""key":"{}""#, s.key);
        assert_eq!(json.matches(&key).count(), 1, "{} is missing", s.key);
    }
    assert!(json.contains(r#""key":"brightness","kind":"number""#));
    assert!(json.contains(r#""default":800,"value":640"#));
    assert!(json.contains(r#""choices":["km/h","mph"]"#));
    assert!(json.contains(r#""scale":10"#), "an editor needs the fixed-point scale");

    // Quotes are balanced, which is what an unescaped symbol or doc would break.
    assert_eq!(json.matches('"').count() % 2, 0);
    assert!(!json.contains(r#"""""#));
}

#[test]
fn a_unit_symbol_survives_the_description_intact() {
    let _t = table();
    let mut json = String::new();
    config::describe(&mut json).unwrap();

    // The degree sign is the only non-ASCII character the table reaches, so
    // it is what a byte-wise writer would mangle.
    assert!(json.contains("\"unit\":\"\u{b0}C\""), "the degree sign came through");
    assert!(json.contains("\"unit\":\"rpm\""));
    assert!(json.contains("\"unit\":\"\""), "a RAW row has an empty symbol");

    // Nothing in the table needs escaping today, which is why the JSON holds
    // no backslash at all; the escaper is there for a doc line that later does.
    assert!(!json.contains('\\'));
}
