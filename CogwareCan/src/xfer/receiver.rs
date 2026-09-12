// SPDX-License-Identifier: GPL-3.0-only
//! Node side of a transfer: consume the master's frames into a `Sink`.

use super::{Image, Status, Sink, CMD_ID, DATA_ID, REPLY_ID};
use crate::crc::Crc32;
use crate::protocol::{frame, id_of, NODE_BROADCAST};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxState {
    Idle,
    Receiving,
    Verified,
    Failed,
}

/// Node side of a transfer.
pub struct Receiver<S: Sink> {
    node: u8,
    sink: S,
    state: RxState,
    image: Image,
    expected_seq: u16,
    received: u32,
    missing: Option<u16>,
    crc: Crc32,
}

impl<S: Sink> Receiver<S> {
    /// `node` is this node's bus address, fixed by the firmware.
    pub fn new(node: u8, sink: S) -> Self {
        Receiver {
            node,
            sink,
            state: RxState::Idle,
            image: Image { size: 0, kind: 0, slot: 0 },
            expected_seq: 0,
            received: 0,
            missing: None,
            crc: Crc32::new(),
        }
    }

    pub fn state(&self) -> RxState {
        self.state
    }

    pub fn sink(&mut self) -> &mut S {
        &mut self.sink
    }

    /// Bytes stored so far and the declared image size.
    pub fn progress(&self) -> (u32, u32) {
        (self.received, self.image.size)
    }

    /// What the current session is carrying, meaningless while `Idle`.
    pub fn image(&self) -> Image {
        self.image
    }

    fn reply(&self, status: Status, seq: u16) -> Option<CanFrame> {
        let [lo, hi] = seq.to_le_bytes();
        frame(REPLY_ID, &[self.node, status as u8, lo, hi])
    }

    /// Feed every frame off the bus. Returns a reply frame to transmit, if any.
    pub fn feed(&mut self, f: &CanFrame) -> Option<CanFrame> {
        let data = &f.data()[..f.dlc()];
        match id_of(f)? {
            DATA_ID => self.data(data),
            CMD_ID => self.command(data),
            _ => None,
        }
    }

    fn data(&mut self, data: &[u8]) -> Option<CanFrame> {
        if self.state != RxState::Receiving || data.len() < 2 {
            return None;
        }
        let seq = u16::from_le_bytes([data[0], data[1]]);
        if seq != self.expected_seq {
            if self.missing.is_none() && self.received < self.image.size {
                self.missing = Some(self.expected_seq);
            }
            return None;
        }
        self.missing = None;
        let remaining = (self.image.size - self.received) as usize;
        let payload = &data[2..2 + remaining.min(data.len() - 2)];
        if payload.is_empty() {
            return None;
        }
        if !self.sink.write(self.received, payload) {
            return self.fail();
        }
        self.crc.update(payload);
        self.received += payload.len() as u32;
        self.expected_seq = self.expected_seq.wrapping_add(1);
        None
    }

    fn fail(&mut self) -> Option<CanFrame> {
        self.state = RxState::Failed;
        self.sink.abort();
        self.reply(Status::Aborted, self.expected_seq)
    }

    fn command(&mut self, data: &[u8]) -> Option<CanFrame> {
        if data.len() < 2 || (data[0] != self.node && data[0] != NODE_BROADCAST) {
            return None;
        }
        match data[1] {
            1 if data.len() >= 7 => {
                let image = Image {
                    size: u32::from_le_bytes([data[2], data[3], data[4], data[5]]),
                    kind: data[6],
                    // Pre-slot senders send seven bytes; slot 0 is the only
                    // slot a node with one of each kind ever has.
                    slot: data.get(7).copied().unwrap_or(0),
                };
                if !self.sink.begin(image) {
                    return self.reply(Status::Refused, 0);
                }
                self.image = image;
                self.expected_seq = 0;
                self.received = 0;
                self.missing = None;
                self.crc = Crc32::new();
                self.state = RxState::Receiving;
                self.reply(Status::Ready, 0)
            }
            2 if self.state == RxState::Receiving && data.len() >= 4 => {
                let last_sent = u16::from_le_bytes([data[2], data[3]]);
                match self.missing {
                    Some(seq) => self.reply(Status::Missing, seq),
                    None if self.expected_seq != last_sent.wrapping_add(1) => {
                        self.reply(Status::Missing, self.expected_seq)
                    }
                    None => self.reply(Status::Ok, last_sent),
                }
            }
            3 if self.state == RxState::Receiving && data.len() >= 6 => {
                let crc = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
                if self.received == self.image.size && self.crc.finish() == crc {
                    self.state = RxState::Verified;
                    self.reply(Status::Done, self.expected_seq)
                } else {
                    self.state = RxState::Failed;
                    self.sink.abort();
                    self.reply(Status::CrcFail, self.expected_seq)
                }
            }
            4 if self.state == RxState::Verified => {
                self.sink.commit(self.image);
                self.state = RxState::Idle;
                None
            }
            5 if self.state != RxState::Idle => {
                self.state = RxState::Idle;
                self.sink.abort();
                self.reply(Status::Aborted, self.expected_seq)
            }
            _ => None,
        }
    }
}
