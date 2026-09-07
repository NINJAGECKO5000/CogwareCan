//! CogwareCan: one crate holding the gauge spec, the ECU converters, and the
//! CAN framing, so a server or display only ever talks to this crate.
//!
//! Server side (ECU in, CAN out):
//! ```ignore
//! speeduino::parse_n(&uart_packet)?;          // serial ECU bytes -> gauges
//! haltech::SOURCE.feed_default(&ecu_frame);   // CAN-broadcast ECU -> gauges
//! if let Some(f) = frame_for(Gauge::Rpm.id()) // gauge -> CAN frame
//!     { can.send_message(f)?; }
//! ```
//!
//! Display side (CAN in, gauges out):
//! ```ignore
//! feed_frame(&frame)?;                       // CAN frame -> gauge
//! let rpm = RPM.get_or(0);                   // canonical units, see `Unit`
//! let clt = CLNT.as_f32();                   // 87.5 for 87.5 °C, None if unset
//! let psi = MAP.to(Unit::PSI);               // unit conversion, see `units`
//! ```
//!
//! Adding an ECU: one new module holding a `CanSource` table (CAN broadcast)
//! or a `Field` table plus a packet parser (serial). See `decode`. OBD2 is
//! request/response and lives in `obd2`.
//!
//! Bus housekeeping: `protocol` holds the ID map and node addressing,
//! `subscribe` the display-to-master gauge subscriptions, and `xfer` the
//! master-to-node file transfer used for over-the-air updates. All three are
//! frame in, frame out, with no driver or timer of their own.

#![cfg_attr(not(test), no_std)]

pub mod crc;
pub mod decode;
pub mod ecumaster;
mod gauge;
pub mod haltech;
pub mod maxxecu;
pub mod megasquirt;
pub mod obd2;
pub mod rusefi;
pub mod speeduino;
pub mod subscribe;
pub mod units;
pub mod xfer;

pub use decode::CanSource;
pub use gauge::*;
pub use units::{Quantity, Reading, Unit};

/// Every CAN-broadcast ECU this crate understands. A server that does not
/// know which ECU is attached can try each in turn:
/// `CAN_SOURCES.iter().find_map(|s| s.feed_default(&frame))`.
pub static CAN_SOURCES: &[&CanSource] = &[
    &rusefi::SOURCE,
    &haltech::SOURCE,
    &megasquirt::SOURCE,
    &maxxecu::SOURCE,
    &ecumaster::SOURCE,
];

use embedded_hal_0_2::can::{Frame, Id, StandardId};
use mcp2515::frame::CanFrame;

/// The CAN ID map of the display bus.
///
/// | Range           | Use                                                    |
/// |-----------------|--------------------------------------------------------|
/// | 0x000..=0x01F   | Reserved: bus control and node addressing (this module) |
/// | 0x020..=0x6FF   | Gauges, one ID each (see `gauge`)                       |
/// | 0x700..=0x7FF   | Left clear for OBD2 (0x7DF request, 0x7E8.. replies)    |
///
/// Inside the reserved range: 0x000 master ack, 0x010..=0x012 file transfer
/// (see `xfer`), 0x015 client subscribe request. Every node on the bus has a
/// one-byte node address chosen by the firmware that embeds this crate;
/// `NODE_MASTER` is the server and `NODE_BROADCAST` addresses all nodes.
pub mod protocol {
    use super::*;

    /// Lowest and highest IDs reserved for bus control.
    pub const RESERVED_ID_MIN: u16 = 0x000;
    pub const RESERVED_ID_MAX: u16 = 0x01F;
    /// Server acknowledges a client's subscription; payload is the gauge ID echoed back.
    pub const MASTER_ACK_ID: u16 = 0x000;
    /// Client asks the server to start broadcasting gauges; payload is up to 8 gauge IDs.
    pub const CLIENT_REQUEST_ID: u16 = 0x015;
    /// First ID available to gauges.
    pub const GAUGE_ID_MIN: u16 = 0x020;
    /// Top of the gauge range.
    pub const GAUGE_ID_MAX: u16 = 0x6FF;

    /// Node address of the master (the ECU-facing server).
    pub const NODE_MASTER: u8 = 0x00;
    /// Node address that every node answers to.
    pub const NODE_BROADCAST: u8 = 0xFF;

    pub fn is_reserved(id: u16) -> bool {
        (RESERVED_ID_MIN..=RESERVED_ID_MAX).contains(&id)
    }

    pub fn is_gauge_range(id: u16) -> bool {
        (GAUGE_ID_MIN..=GAUGE_ID_MAX).contains(&id)
    }

    /// Build a standard-ID frame; `None` if the ID is over 11 bits or the payload over 8 bytes.
    pub fn frame(id: u16, data: &[u8]) -> Option<CanFrame> {
        CanFrame::new(Id::Standard(StandardId::new(id)?), data)
    }

    /// Standard ID of a frame, or `None` for extended IDs.
    pub fn id_of(frame: &CanFrame) -> Option<u16> {
        match frame.id() {
            Id::Standard(s) => Some(s.as_raw()),
            Id::Extended(_) => None,
        }
    }

    /// Frame a client sends to subscribe to `ids` (at most 8 per frame).
    pub fn request_frame(ids: &[u8]) -> Option<CanFrame> {
        frame(CLIENT_REQUEST_ID, ids)
    }

    /// Frame the server sends to acknowledge subscription to `id`.
    pub fn ack_frame(id: u8) -> Option<CanFrame> {
        frame(MASTER_ACK_ID, &[id])
    }

    pub fn is_request(frame: &CanFrame) -> bool {
        id_of(frame) == Some(CLIENT_REQUEST_ID)
    }

    pub fn is_ack(frame: &CanFrame) -> bool {
        id_of(frame) == Some(MASTER_ACK_ID)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedError {
    /// Extended (29-bit) IDs are not part of the spec.
    ExtendedId,
    /// No gauge is defined at this ID (protocol frames land here too).
    UnknownId(u16),
    /// Payload length does not match the gauge's wire width.
    BadLength { id: u16, expected: usize, got: usize },
}

/// Store an incoming gauge frame. Frames that are not gauges are rejected,
/// never panicked on, so this is safe to call on every frame off the bus.
pub fn feed_frame(frame: &CanFrame) -> Result<&'static GaugeData, FeedError> {
    let id = match frame.id() {
        Id::Standard(s) => s.as_raw(),
        Id::Extended(_) => return Err(FeedError::ExtendedId),
    };
    let gauge = gauge_by_id(id).ok_or(FeedError::UnknownId(id))?;
    gauge
        .set_from_frame(frame)
        .map_err(|FrameError::BadLength { expected, got }| FeedError::BadLength {
            id,
            expected,
            got,
        })?;
    Ok(gauge)
}

/// Frame for the gauge at `id`, or `None` if the ID is unknown or the gauge
/// has not been set yet.
pub fn frame_for(id: u16) -> Option<CanFrame> {
    gauge_by_id(id)?.to_frame()
}

/// Reset every gauge to unset. Useful when the ECU link drops.
pub fn clear_all() {
    for g in ALL_GAUGES {
        g.clear();
    }
}
