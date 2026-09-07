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
//! Session: `Begin(size, kind)` → node replies `Ready`; master streams data
//! blocks and every `window` blocks sends `Check(seq)`; each node replies `Ok`
//! or `Missing(first lost seq)`, and the master rewinds to the lowest missing
//! block. `End(crc32)` → `Done` or `CrcFail`; `Commit` tells the node to apply
//! the image. Data frames are not node-addressed; a node only consumes them
//! while inside a session that `Begin` addressed to it.
//!
//! Node side: implement `Sink` (write to flash, a buffer, an SD card) and run
//! a `Receiver`. Master side: run a `Sender` over the image bytes.

use crate::crc::Crc32;
use crate::protocol::{frame, id_of, NODE_BROADCAST};
use embedded_hal_0_2::can::Frame;
use mcp2515::frame::CanFrame;

pub const CMD_ID: u16 = 0x010;
pub const DATA_ID: u16 = 0x011;
pub const REPLY_ID: u16 = 0x012;
/// Image bytes per data frame.
pub const BLOCK_LEN: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Cmd {
    /// `[node, 1, size u32 LE, kind]`
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
    /// Accept or refuse a transfer of `size` bytes of `kind`.
    fn begin(&mut self, size: u32, kind: u8) -> bool;
    /// Store `data` at `offset`. Return false to abort the transfer.
    fn write(&mut self, offset: u32, data: &[u8]) -> bool;
    /// The image is complete and CRC-verified; apply it.
    fn commit(&mut self, size: u32, kind: u8);
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
    fn begin(&mut self, size: u32, _kind: u8) -> bool {
        self.len = 0;
        self.committed = false;
        size as usize <= self.buf.len()
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
    fn commit(&mut self, _size: u32, _kind: u8) {
        self.committed = true;
    }
}

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
    size: u32,
    kind: u8,
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
            size: 0,
            kind: 0,
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
        (self.received, self.size)
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
            if self.missing.is_none() && self.received < self.size {
                self.missing = Some(self.expected_seq);
            }
            return None;
        }
        self.missing = None;
        let remaining = (self.size - self.received) as usize;
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
                let size = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
                let kind = data[6];
                if !self.sink.begin(size, kind) {
                    return self.reply(Status::Refused, 0);
                }
                self.size = size;
                self.kind = kind;
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
                if self.received == self.size && self.crc.finish() == crc {
                    self.state = RxState::Verified;
                    self.reply(Status::Done, self.expected_seq)
                } else {
                    self.state = RxState::Failed;
                    self.sink.abort();
                    self.reply(Status::CrcFail, self.expected_seq)
                }
            }
            4 if self.state == RxState::Verified => {
                self.sink.commit(self.size, self.kind);
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
    /// Push `image` (tagged `kind`) to `node`, or to all nodes with
    /// `NODE_BROADCAST`. `window` is blocks between checkpoints;
    /// `replies_needed` is how many nodes must answer each checkpoint (1 for a
    /// single node, the node count for broadcast).
    pub fn new(node: u8, image: &'a [u8], kind: u8, window: u16, replies_needed: u8) -> Self {
        Sender {
            node,
            image,
            kind,
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

    pub fn state(&self) -> TxState {
        self.state
    }

    /// Bytes acknowledged so far and the image size.
    pub fn progress(&self) -> (usize, usize) {
        (self.offset, self.image.len())
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
                7
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
