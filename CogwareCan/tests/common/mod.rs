// SPDX-License-Identifier: GPL-3.0-only
//! Shared setup for the integration tests.
//!
//! The gauge table is a set of process-wide statics, so every test in a
//! binary shares one copy of it. Any test that reads or writes gauges takes
//! the `bus()` guard, which serialises those tests against each other and
//! empties the table on entry *and* on exit.
//!
//! Clearing on entry means a test can never inherit another's values.
//! Clearing on exit means it can never leave any, even when it panics part
//! way through. Without both, a suite passes or fails on test *ordering*: a
//! test can be cleaned up by whichever test happens to sort after it, which
//! then breaks when a test is renamed, added, or filtered out.

#![allow(dead_code)]

use cogware_can::{clear_all, config};
use std::sync::{Mutex, MutexGuard};

static LOCK: Mutex<()> = Mutex::new(());
static SETTINGS_LOCK: Mutex<()> = Mutex::new(());

/// Exclusive use of the gauge table for as long as this value is alive.
pub struct Bus(MutexGuard<'static, ()>);

/// Take the gauge table, empty.
pub fn bus() -> Bus {
    // A test that panicked while holding the lock poisons it. The state it
    // left behind is cleared here regardless, so the poison tells us nothing
    // and the next test is entitled to a clean table.
    let guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clear_all();
    Bus(guard)
}

impl Drop for Bus {
    fn drop(&mut self) {
        clear_all();
    }
}

/// Exclusive use of the settings table for as long as this value is alive.
/// Settings are process-wide statics too, and take the same treatment.
pub struct Settings(MutexGuard<'static, ()>);

/// Take the settings table, at its defaults.
pub fn settings() -> Settings {
    let guard = SETTINGS_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    config::reset_all();
    Settings(guard)
}

impl Drop for Settings {
    fn drop(&mut self) {
        config::reset_all();
    }
}
