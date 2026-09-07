//! Which gauges each display wants, and how it asks for them.
//!
//! Both halves are pure state machines over CAN frames. They touch no driver,
//! no timer and no pin, so they are the same on every board and every CAN
//! controller. The firmware owns the loop: it hands frames in and transmits
//! the frames that come out, and it alone decides retry pacing.
//!
//! Display side:
//!
//! ```ignore
//! let mut sub = Subscription::new(&WANTED);
//! while !sub.is_complete() {
//!     for f in sub.request() { can.send_message(f).ok(); }   // only what is still missing
//!     while waiting {
//!         if let Ok(f) = can.read_message() { sub.feed(&f); }
//!     }
//! }
//! ```
//!
//! Master side:
//!
//! ```ignore
//! for ack in publisher.feed(&frame) { can.send_message(ack).ok(); }
//! for f in publisher.broadcast() { can.send_message(f).ok(); }
//! ```

use crate::protocol::{ack_frame, is_ack, is_request, request_frame};
use crate::{frame_for, gauge_by_id};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

/// Gauge IDs that fit in one request frame.
pub const IDS_PER_FRAME: usize = 8;

/// A set of gauge IDs. IDs are one byte, so the whole set is 32 bytes and
/// there is no capacity to run out of. Iteration is in ID order, which is
/// also CAN arbitration order.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct IdSet {
    bits: [u8; 32],
}

impl IdSet {
    pub const fn new() -> Self {
        IdSet { bits: [0; 32] }
    }

    /// Add an ID. Returns true if it was not already present.
    pub fn insert(&mut self, id: u8) -> bool {
        let (i, m) = (id as usize / 8, 1 << (id % 8));
        let new = self.bits[i] & m == 0;
        self.bits[i] |= m;
        new
    }

    pub fn remove(&mut self, id: u8) {
        self.bits[id as usize / 8] &= !(1 << (id % 8));
    }

    pub fn contains(&self, id: u8) -> bool {
        self.bits[id as usize / 8] & (1 << (id % 8)) != 0
    }

    pub fn len(&self) -> usize {
        self.bits.iter().map(|b| b.count_ones() as usize).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.bits.iter().all(|b| *b == 0)
    }

    pub fn clear(&mut self) {
        self.bits = [0; 32];
    }

    pub fn iter(&self) -> IdIter {
        IdIter { set: *self, next: 0 }
    }
}

impl IntoIterator for IdSet {
    type Item = u8;
    type IntoIter = IdIter;

    fn into_iter(self) -> IdIter {
        IdIter { set: self, next: 0 }
    }
}

impl Default for IdSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Yields the IDs in a set, lowest first. Holds its own copy of the set, so
/// the set it came from stays free to change.
pub struct IdIter {
    set: IdSet,
    next: u16,
}

impl Iterator for IdIter {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        while self.next < 256 {
            let id = self.next as u8;
            self.next += 1;
            if self.set.contains(id) {
                return Some(id);
            }
        }
        None
    }
}

// ===================== master side =====================

/// Tracks what the displays on the bus have asked for.
#[derive(Clone, Copy, Debug, Default)]
pub struct Publisher {
    wanted: IdSet,
}

impl Publisher {
    pub const fn new() -> Self {
        Publisher { wanted: IdSet::new() }
    }

    /// Feed every frame off the bus. Requests are recorded and the returned
    /// iterator yields one acknowledgement per ID asked for; everything else
    /// yields nothing.
    ///
    /// Every requested ID is acknowledged, including one this build has no
    /// gauge for, so a display running an older spec finishes subscribing
    /// instead of retrying forever. Only IDs that name a real gauge are
    /// recorded, so `broadcast` never wastes bus time on them.
    pub fn feed(&mut self, frame: &CanFrame) -> Acks {
        if !is_request(frame) {
            return Acks::empty();
        }
        let ids = &frame.data()[..frame.dlc().min(IDS_PER_FRAME)];
        for &id in ids {
            if gauge_by_id(id as u16).is_some() {
                self.wanted.insert(id);
            }
        }
        Acks::new(ids)
    }

    /// Record a subscription directly, for a master with a fixed set of
    /// displays that does not wait to be asked.
    pub fn subscribe(&mut self, id: u8) -> bool {
        gauge_by_id(id as u16).is_some() && self.wanted.insert(id)
    }

    pub fn wanted(&self) -> &IdSet {
        &self.wanted
    }

    pub fn is_empty(&self) -> bool {
        self.wanted.is_empty()
    }

    pub fn len(&self) -> usize {
        self.wanted.len()
    }

    /// Forget every subscription, so displays must ask again.
    pub fn clear(&mut self) {
        self.wanted.clear();
    }

    /// One frame per subscribed gauge that has a value, lowest ID first.
    /// A gauge that has never been written yields no frame, so a display can
    /// tell "this ECU does not supply it" from "the value is zero".
    pub fn broadcast(&self) -> Broadcast {
        Broadcast { ids: self.wanted.into_iter() }
    }
}

/// Acknowledgements to transmit, at most one frame's worth.
pub struct Acks {
    ids: [u8; IDS_PER_FRAME],
    len: u8,
    at: u8,
}

impl Acks {
    fn empty() -> Self {
        Acks { ids: [0; IDS_PER_FRAME], len: 0, at: 0 }
    }

    fn new(ids: &[u8]) -> Self {
        let mut a = Acks::empty();
        a.len = ids.len().min(IDS_PER_FRAME) as u8;
        a.ids[..a.len as usize].copy_from_slice(&ids[..a.len as usize]);
        a
    }
}

impl Iterator for Acks {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        while self.at < self.len {
            let id = self.ids[self.at as usize];
            self.at += 1;
            if let Some(f) = ack_frame(id) {
                return Some(f);
            }
        }
        None
    }
}

pub struct Broadcast {
    ids: IdIter,
}

impl Iterator for Broadcast {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        loop {
            let id = self.ids.next()?;
            if let Some(f) = frame_for(id as u16) {
                return Some(f);
            }
        }
    }
}

// ===================== display side =====================

/// Asks the master for a fixed list of gauges and tracks the replies.
#[derive(Clone, Copy, Debug)]
pub struct Subscription<'a> {
    wanted: &'a [u8],
    acked: IdSet,
}

impl<'a> Subscription<'a> {
    /// `wanted` is the display's gauge list, fixed at build time.
    pub const fn new(wanted: &'a [u8]) -> Self {
        Subscription { wanted, acked: IdSet::new() }
    }

    /// Frames asking for everything not yet acknowledged, eight IDs each.
    /// Call again after a timeout to retry; the list shrinks as acks arrive.
    ///
    /// The iterator holds its own snapshot, so acks may be fed while it is
    /// still being drained.
    pub fn request(&self) -> Requests<'a> {
        Requests { wanted: self.wanted, acked: self.acked, at: 0 }
    }

    /// Feed every frame off the bus. Returns true if this frame acknowledged
    /// a gauge that was still outstanding.
    pub fn feed(&mut self, frame: &CanFrame) -> bool {
        if !is_ack(frame) || frame.dlc() < 1 {
            return false;
        }
        let id = frame.data()[0];
        self.wanted.contains(&id) && self.acked.insert(id)
    }

    pub fn is_complete(&self) -> bool {
        self.wanted.iter().all(|id| self.acked.contains(*id))
    }

    /// How many gauges are still waiting to be acknowledged.
    pub fn pending(&self) -> usize {
        self.wanted.iter().filter(|id| !self.acked.contains(**id)).count()
    }

    pub fn wanted(&self) -> &'a [u8] {
        self.wanted
    }

    /// Start over, after the master reset or the bus dropped.
    pub fn reset(&mut self) {
        self.acked.clear();
    }
}

pub struct Requests<'a> {
    wanted: &'a [u8],
    acked: IdSet,
    at: usize,
}

impl Iterator for Requests<'_> {
    type Item = CanFrame;

    fn next(&mut self) -> Option<CanFrame> {
        let mut buf = [0u8; IDS_PER_FRAME];
        let mut n = 0;
        while self.at < self.wanted.len() && n < IDS_PER_FRAME {
            let id = self.wanted[self.at];
            self.at += 1;
            if !self.acked.contains(id) {
                buf[n] = id;
                n += 1;
            }
        }
        if n == 0 {
            return None;
        }
        request_frame(&buf[..n])
    }
}
