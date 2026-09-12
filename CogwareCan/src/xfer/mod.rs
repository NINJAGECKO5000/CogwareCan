// SPDX-License-Identifier: GPL-3.0-only
//! Bus file transfer: the master pushes an image to one node or to every
//! node, for over-the-air updates. Pure state machines with no I/O or timers;
//! the firmware moves frames between them and its CAN driver.
//!
//! Frames (IDs in the reserved control range, see `protocol`):
//!
//! | ID    | Direction     | Payload                                        |
//! |-------|---------------|------------------------------------------------|
//! | 0x010 | master → node | `[node, cmd, args..]` control (see `Cmd`)       |
//! | 0x011 | master → node | `[seq lo, seq hi, up to 6 image bytes]`         |
//! | 0x012 | node → master | `[node, status, seq lo, seq hi]` (see `Status`) |
//!
//! Session: `Begin(image)` → node replies `Ready`; master streams data blocks
//! and every `window` blocks sends `Check(seq)`; each node replies `Ok` or
//! `Missing(first lost seq)`, and the master rewinds to the lowest missing
//! block. `End(crc32)` → `Done` or `CrcFail`; `Commit` tells the node to apply
//! the image. Data frames are not node-addressed; a node only consumes them
//! while inside a session that `Begin` addressed to it.
//!
//! The master should raise `bus::FLAG_UPDATE` for the length of a session and
//! feed `Sender::progress` into `Bus::update_progress`, because that flag is
//! how the nodes *not* being written learn why the bus is busy. Nothing here
//! does that itself; the firmware owns both state machines.
//!
//! Node side: implement `Sink` (write to flash, a buffer, an SD card) and run
//! a `Receiver`. Master side: run a `Sender` over the image bytes.

mod receiver;
mod sender;

pub use receiver::{Receiver, RxState};
pub use sender::{Sender, TxState};

pub const CMD_ID: u16 = 0x010;
pub const DATA_ID: u16 = 0x011;
pub const REPLY_ID: u16 = 0x012;
/// Image bytes per data frame.
pub const BLOCK_LEN: usize = 6;

/// What an image is, so a node can refuse one it has nowhere to put.
/// The values are part of the wire format and never change meaning.
pub mod kind {
    /// A node firmware image, applied by the bootloader.
    pub const FIRMWARE: u8 = 1;
    /// A `.scene` layout, one per slot.
    pub const SCENE: u8 = 2;
    /// A settings blob.
    pub const CONFIG: u8 = 3;
    /// A font, bitmap or other asset a scene refers to.
    pub const ASSET: u8 = 4;
}

/// What is being transferred: how big, what it is, and which of the
/// receiver's slots it lands in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Image {
    /// Length in bytes.
    pub size: u32,
    /// One of the constants in `kind`.
    pub kind: u8,
    /// Which slot of that kind to overwrite; 0 when the node keeps only one.
    pub slot: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Cmd {
    /// `[node, 1, size u32 LE, kind, slot]`
    Begin = 1,
    /// `[node, 2, seq u16 LE]` — reply with `Ok` or `Missing`
    Check = 2,
    /// `[node, 3, crc32 u32 LE]`
    End = 3,
    /// `[node, 4]` — apply the verified image
    Commit = 4,
    /// `[node, 5]`
    Abort = 5,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    Ready = 1,
    /// All blocks up to and including `seq` received.
    Ok = 2,
    /// `seq` is the first block not received.
    Missing = 3,
    /// Image complete and CRC verified.
    Done = 4,
    CrcFail = 5,
    /// Node declined `Begin` (wrong kind, too large, busy).
    Refused = 6,
    Aborted = 7,
}

impl Status {
    fn from_u8(v: u8) -> Option<Status> {
        Some(match v {
            1 => Status::Ready,
            2 => Status::Ok,
            3 => Status::Missing,
            4 => Status::Done,
            5 => Status::CrcFail,
            6 => Status::Refused,
            7 => Status::Aborted,
            _ => return None,
        })
    }
}

/// Where a node puts the incoming image.
pub trait Sink {
    /// Accept or refuse `image`. Refusing is how a node declines a kind or a
    /// slot it has no room for.
    fn begin(&mut self, image: Image) -> bool;
    /// Store `data` at `offset`. Return false to abort the transfer.
    fn write(&mut self, offset: u32, data: &[u8]) -> bool;
    /// The image is complete and CRC-verified; apply it.
    fn commit(&mut self, image: Image);
    /// The transfer failed or was cancelled; discard anything partial.
    fn abort(&mut self) {}
}

/// A `Sink` over a RAM buffer, for tests and small images.
pub struct BufferSink<'a> {
    pub buf: &'a mut [u8],
    pub len: usize,
    pub committed: bool,
}

impl<'a> BufferSink<'a> {
    pub fn new(buf: &'a mut [u8]) -> Self {
        BufferSink { buf, len: 0, committed: false }
    }
}

impl Sink for BufferSink<'_> {
    fn begin(&mut self, image: Image) -> bool {
        self.len = 0;
        self.committed = false;
        image.size as usize <= self.buf.len()
    }
    fn write(&mut self, offset: u32, data: &[u8]) -> bool {
        let off = offset as usize;
        if off + data.len() > self.buf.len() {
            return false;
        }
        self.buf[off..off + data.len()].copy_from_slice(data);
        self.len = self.len.max(off + data.len());
        true
    }
    fn commit(&mut self, _image: Image) {
        self.committed = true;
    }
}
