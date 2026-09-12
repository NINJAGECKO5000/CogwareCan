// SPDX-License-Identifier: GPL-3.0-only
//! Master side of a transfer: turn an image into frames and track replies.

use super::{Cmd, Image, Status, BLOCK_LEN, CMD_ID, DATA_ID, REPLY_ID};
use crate::protocol::{frame, id_of, NODE_BROADCAST};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxState {
    /// `next_frame` yields `Begin`.
    Begin,
    /// Waiting for `Ready` from every target.
    WaitReady,
    /// `next_frame` yields data blocks, then `Check`.
    Data,
    /// Waiting for `Ok`/`Missing` from every target.
    WaitCheck,
    /// `next_frame` yields `End`.
    End,
    /// Waiting for `Done` from every target.
    WaitDone,
    /// `next_frame` yields `Commit`.
    Commit,
    Done,
    Failed(Status),
}

/// Master side of a transfer.
pub struct Sender<'a> {
    node: u8,
    image: &'a [u8],
    kind: u8,
    slot: u8,
    window: u16,
    replies_needed: u8,
    state: TxState,
    seq: u16,
    offset: usize,
    in_window: u16,
    replies: u8,
    rewind_to: Option<u16>,
    blocks: usize,
    crc: u32,
}

impl<'a> Sender<'a> {
    /// Push `image` (tagged `kind`, see the `kind` module) to `node`, or to
    /// all nodes with `NODE_BROADCAST`. `window` is blocks between
    /// checkpoints; `replies_needed` is how many nodes must answer each
    /// checkpoint (1 for a single node, the node count for broadcast).
    pub fn new(node: u8, image: &'a [u8], kind: u8, window: u16, replies_needed: u8) -> Self {
        Sender {
            node,
            image,
            kind,
            slot: 0,
            window: window.max(1),
            replies_needed: replies_needed.max(1),
            state: TxState::Begin,
            seq: 0,
            offset: 0,
            in_window: 0,
            replies: 0,
            rewind_to: None,
            blocks: 0,
            crc: crate::crc::crc32(image),
        }
    }

    /// Land the image in `slot` rather than slot 0, for a node that keeps
    /// more than one image of this kind.
    pub fn with_slot(mut self, slot: u8) -> Self {
        self.slot = slot;
        self
    }

    pub fn state(&self) -> TxState {
        self.state
    }

    /// What this sender is pushing.
    pub fn image(&self) -> Image {
        Image { size: self.image.len() as u32, kind: self.kind, slot: self.slot }
    }

    /// Bytes acknowledged so far and the image size.
    pub fn progress(&self) -> (usize, usize) {
        (self.offset, self.image.len())
    }

    /// Bytes acknowledged as a percentage, for `bus::Bus::update_progress`.
    pub fn percent(&self) -> u8 {
        if self.image.is_empty() {
            return 100;
        }
        (self.offset.min(self.image.len()) * 100 / self.image.len()) as u8
    }

    /// Call after a timeout while in a `Wait*` state to resend the last control frame.
    pub fn resend(&mut self) -> Option<CanFrame> {
        self.replies = 0;
        match self.state {
            TxState::WaitReady => self.ctrl(Cmd::Begin),
            TxState::WaitCheck => self.ctrl(Cmd::Check),
            TxState::WaitDone => self.ctrl(Cmd::End),
            _ => None,
        }
    }

    pub fn abort(&mut self) -> Option<CanFrame> {
        self.state = TxState::Failed(Status::Aborted);
        self.ctrl(Cmd::Abort)
    }

    fn ctrl(&self, cmd: Cmd) -> Option<CanFrame> {
        let mut buf = [0u8; 8];
        buf[0] = self.node;
        buf[1] = cmd as u8;
        let len = match cmd {
            Cmd::Begin => {
                buf[2..6].copy_from_slice(&(self.image.len() as u32).to_le_bytes());
                buf[6] = self.kind;
                buf[7] = self.slot;
                8
            }
            Cmd::Check => {
                buf[2..4].copy_from_slice(&self.seq.wrapping_sub(1).to_le_bytes());
                4
            }
            Cmd::End => {
                buf[2..6].copy_from_slice(&self.crc.to_le_bytes());
                6
            }
            Cmd::Commit | Cmd::Abort => 2,
        };
        frame(CMD_ID, &buf[..len])
    }

    /// Next frame to transmit, or `None` while waiting for replies or when finished.
    pub fn next_frame(&mut self) -> Option<CanFrame> {
        match self.state {
            TxState::Begin => {
                self.state = TxState::WaitReady;
                self.replies = 0;
                self.ctrl(Cmd::Begin)
            }
            TxState::Data => {
                if self.offset >= self.image.len() || self.in_window >= self.window {
                    self.in_window = 0;
                    self.replies = 0;
                    self.rewind_to = None;
                    self.state = TxState::WaitCheck;
                    return self.ctrl(Cmd::Check);
                }
                let end = (self.offset + BLOCK_LEN).min(self.image.len());
                let mut buf = [0u8; 8];
                buf[..2].copy_from_slice(&self.seq.to_le_bytes());
                buf[2..2 + end - self.offset].copy_from_slice(&self.image[self.offset..end]);
                let f = frame(DATA_ID, &buf[..2 + end - self.offset]);
                self.offset = end;
                self.seq = self.seq.wrapping_add(1);
                self.blocks += 1;
                self.in_window += 1;
                f
            }
            TxState::End => {
                self.state = TxState::WaitDone;
                self.replies = 0;
                self.ctrl(Cmd::End)
            }
            TxState::Commit => {
                self.state = TxState::Done;
                self.ctrl(Cmd::Commit)
            }
            _ => None,
        }
    }

    /// Feed node replies (frames with `REPLY_ID`); other frames are ignored.
    pub fn feed_reply(&mut self, f: &CanFrame) {
        if id_of(f) != Some(REPLY_ID) || f.dlc() < 4 {
            return;
        }
        let d = f.data();
        if self.node != NODE_BROADCAST && d[0] != self.node {
            return;
        }
        let Some(status) = Status::from_u8(d[1]) else { return };
        let seq = u16::from_le_bytes([d[2], d[3]]);
        match (self.state, status) {
            (TxState::WaitReady, Status::Ready) => self.count(TxState::Data),
            (TxState::WaitCheck, Status::Ok) => self.count(TxState::Data),
            (TxState::WaitCheck, Status::Missing) => {
                self.rewind_to = Some(match self.rewind_to {
                    Some(r) if self.seq.wrapping_sub(r) > self.seq.wrapping_sub(seq) => r,
                    _ => seq,
                });
                self.count(TxState::Data);
            }
            (TxState::WaitDone, Status::Done) => self.count(TxState::Commit),
            (_, Status::Refused | Status::CrcFail | Status::Aborted) => {
                self.state = TxState::Failed(status)
            }
            _ => {}
        }
    }

    fn count(&mut self, next: TxState) {
        self.replies += 1;
        if self.replies < self.replies_needed {
            return;
        }
        if let Some(seq) = self.rewind_to.take() {
            let back = self.seq.wrapping_sub(seq) as usize;
            self.blocks = self.blocks.saturating_sub(back);
            self.offset = (self.blocks * BLOCK_LEN).min(self.image.len());
            self.seq = seq;
        } else if next == TxState::Data && self.offset >= self.image.len() {
            self.state = TxState::End;
            return;
        }
        self.state = next;
    }
}
