//! The four ISO-TP protocol control information bytes, and the frames they
//! open.
//!
//! ISO 15765-2 lays a small state machine over CAN's eight-byte frame: a
//! single frame carries a whole short payload, a first frame opens a longer
//! one and consecutive frames continue it, and a flow-control frame is how the
//! receiver paces the sender. The first PCI nibble says which of the four a
//! frame is; the rest of the byte, and sometimes the next, says how long or
//! how far along.

use transport::error::{Result, protocol_error};

/// The largest payload the twelve-bit length of a first frame can name.
pub const CLASSIC_CEILING: usize = 4095;

/// A single frame carries at most this, the seven bytes left beside its one
/// PCI byte.
pub const SINGLE_MAX: usize = 7;

/// A first frame carries this many payload bytes beside its two PCI bytes.
pub const FIRST_DATA: usize = 6;

/// A first frame with the length escape carries this many, the four bytes
/// of length having taken the rest.
pub const ESCAPE_DATA: usize = 2;

/// A consecutive frame carries this many payload bytes beside its one PCI byte.
pub const CONSECUTIVE_DATA: usize = 7;

/// What the low nibble of a flow-control frame says about carrying on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlowStatus {
    /// Clear to send: the sender may send the next block.
    Continue,
    /// Wait: the sender holds until another flow control arrives.
    Wait,
    /// The receiver has no room and the transfer is abandoned.
    Overflow,
}

impl FlowStatus {
    /// The low nibble that names this status.
    #[must_use]
    pub const fn nibble(self) -> u8 {
        match self {
            Self::Continue => 0,
            Self::Wait => 1,
            Self::Overflow => 2,
        }
    }

    /// The status a low nibble names.
    ///
    /// # Errors
    /// A nibble outside the three ISO 15765-2 defines.
    pub fn from_nibble(nibble: u8) -> Result<Self> {
        match nibble {
            0 => Ok(Self::Continue),
            1 => Ok(Self::Wait),
            2 => Ok(Self::Overflow),
            other => Err(protocol_error(format!("a flow status of {other}"))),
        }
    }
}

/// One frame read off the bus, told apart by its PCI.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Pci {
    /// A whole payload, this long.
    Single { data: Vec<u8> },
    /// The opening of a payload of `length` bytes, carrying its first six.
    First { length: usize, data: Vec<u8> },
    /// The `index`th continuation, carrying up to seven bytes.
    Consecutive { index: u8, data: Vec<u8> },
    /// The receiver's pacing: how many frames a block is, and the gap between.
    Flow {
        status: FlowStatus,
        block_size: u8,
        separation: u8,
    },
}

/// The single frame carrying `payload`.
///
/// # Errors
/// A payload over [`SINGLE_MAX`], which does not fit one frame.
pub fn single(payload: &[u8]) -> Result<Vec<u8>> {
    if payload.len() > SINGLE_MAX {
        return Err(protocol_error("a single frame carries at most seven bytes"));
    }
    let mut frame = Vec::with_capacity(1 + payload.len());
    // High nibble 0, low nibble the length.
    let len = u8::try_from(payload.len()).unwrap_or(0);
    frame.push(len);
    frame.extend_from_slice(payload);
    Ok(frame)
}

/// The first frame of a payload of `total` bytes, carrying its opening six.
///
/// The twelve-bit length holds up to [`CLASSIC_CEILING`]; a longer payload
/// takes the 32-bit escape, where the length nibbles are zero and four bytes
/// follow. `escape` says the escape is permitted; without it a payload over
/// the ceiling is refused before a frame is built.
///
/// # Errors
/// A payload short enough for a single frame, or one over the ceiling with no
/// escape allowed.
pub fn first(total: usize, opening: &[u8], escape: bool) -> Result<Vec<u8>> {
    if total <= SINGLE_MAX {
        return Err(protocol_error("a payload that fits a single frame"));
    }
    let mut frame = Vec::with_capacity(2 + opening.len());
    if total <= CLASSIC_CEILING {
        let high = 0x10 | u8::try_from(total >> 8).unwrap_or(0);
        frame.push(high);
        frame.push(u8::try_from(total & 0xff).unwrap_or(0));
    } else if escape {
        frame.push(0x10);
        frame.push(0x00);
        frame.extend_from_slice(&u32::try_from(total).unwrap_or(u32::MAX).to_be_bytes());
    } else {
        return Err(protocol_error(
            "a payload over 4095 bytes without the length escape",
        ));
    }
    frame.extend_from_slice(opening);
    Ok(frame)
}

/// The `index`th consecutive frame carrying `chunk`.
///
/// # Errors
/// A chunk over [`CONSECUTIVE_DATA`], which does not fit one frame.
pub fn consecutive(index: u8, chunk: &[u8]) -> Result<Vec<u8>> {
    if chunk.len() > CONSECUTIVE_DATA {
        return Err(protocol_error(
            "a consecutive frame carries at most seven bytes",
        ));
    }
    let mut frame = Vec::with_capacity(1 + chunk.len());
    // High nibble 2, low nibble the sequence number, wrapping at sixteen.
    frame.push(0x20 | (index & 0x0f));
    frame.extend_from_slice(chunk);
    Ok(frame)
}

/// A flow-control frame.
#[must_use]
pub fn flow(status: FlowStatus, block_size: u8, separation: u8) -> Vec<u8> {
    vec![0x30 | status.nibble(), block_size, separation]
}

/// Read the PCI a frame's bytes open with.
///
/// # Errors
/// A frame too short for its own kind, a PCI type outside the four, or a
/// single or first length that overruns the bytes present.
pub fn parse(bytes: &[u8]) -> Result<Pci> {
    let first = *bytes
        .first()
        .ok_or_else(|| protocol_error("an empty CAN frame is no ISO-TP frame"))?;
    match first >> 4 {
        0 => parse_single(first, bytes),
        1 => parse_first(first, bytes),
        2 => Ok(Pci::Consecutive {
            index: first & 0x0f,
            data: bytes[1..].to_vec(),
        }),
        3 => parse_flow(first, bytes),
        other => Err(protocol_error(format!("a PCI type of {other}"))),
    }
}

fn parse_single(first: u8, bytes: &[u8]) -> Result<Pci> {
    let length = usize::from(first & 0x0f);
    if length + 1 > bytes.len() {
        return Err(protocol_error("a single frame shorter than its length"));
    }
    Ok(Pci::Single {
        data: bytes[1..=length].to_vec(),
    })
}

fn parse_first(first: u8, bytes: &[u8]) -> Result<Pci> {
    let short = (usize::from(first & 0x0f) << 8) | usize::from(*bytes.get(1).unwrap_or(&0));
    if short == 0 {
        // The 32-bit escape: four length bytes, then the opening data.
        let raw = bytes
            .get(2..6)
            .ok_or_else(|| protocol_error("a first frame that promises an escape but is short"))?;
        let length = u32::from_be_bytes([raw[0], raw[1], raw[2], raw[3]]) as usize;
        return Ok(Pci::First {
            length,
            data: bytes[6..].to_vec(),
        });
    }
    Ok(Pci::First {
        length: short,
        data: bytes.get(2..).unwrap_or(&[]).to_vec(),
    })
}

fn parse_flow(first: u8, bytes: &[u8]) -> Result<Pci> {
    let block_size = *bytes
        .get(1)
        .ok_or_else(|| protocol_error("a flow-control frame with no block size"))?;
    let separation = *bytes
        .get(2)
        .ok_or_else(|| protocol_error("a flow-control frame with no separation time"))?;
    Ok(Pci::Flow {
        status: FlowStatus::from_nibble(first & 0x0f)?,
        block_size,
        separation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_single_frame_carries_a_short_payload_and_parses_back() {
        let frame = single(&[1, 2, 3]).expect("single");
        assert_eq!(frame, [0x03, 1, 2, 3]);
        assert_eq!(
            parse(&frame).expect("pci"),
            Pci::Single {
                data: vec![1, 2, 3]
            }
        );
        assert!(single(&[0; 8]).is_err(), "eight bytes is too many");
    }

    #[test]
    fn a_first_frame_names_its_length_short_and_escaped() {
        let short = first(100, &[0xaa; FIRST_DATA], false).expect("first");
        assert_eq!(short[0], 0x10);
        assert_eq!(short[1], 100);
        assert_eq!(
            parse(&short).expect("pci"),
            Pci::First {
                length: 100,
                data: vec![0xaa; FIRST_DATA],
            }
        );
        let escaped = first(70_000, &[0xbb; FIRST_DATA], true).expect("escape");
        assert_eq!(&escaped[0..2], &[0x10, 0x00]);
        assert_eq!(
            parse(&escaped).expect("pci"),
            Pci::First {
                length: 70_000,
                data: vec![0xbb; FIRST_DATA],
            }
        );
        assert!(first(5, &[], false).is_err(), "fits a single frame");
        assert!(first(9000, &[], false).is_err(), "no escape allowed");
    }

    #[test]
    fn consecutive_and_flow_frames_round_trip() {
        let cf = consecutive(0x13, &[9, 9]).expect("cf");
        assert_eq!(cf[0], 0x23, "the sequence wraps into four bits");
        assert_eq!(
            parse(&cf).expect("pci"),
            Pci::Consecutive {
                index: 3,
                data: vec![9, 9],
            }
        );
        let fc = flow(FlowStatus::Continue, 8, 20);
        assert_eq!(fc, [0x30, 8, 20]);
        assert_eq!(
            parse(&fc).expect("pci"),
            Pci::Flow {
                status: FlowStatus::Continue,
                block_size: 8,
                separation: 20,
            }
        );
    }

    #[test]
    fn a_frame_of_an_unknown_type_is_refused() {
        assert!(parse(&[0x40]).is_err(), "type four does not exist");
        assert!(parse(&[]).is_err(), "an empty frame is nothing");
        assert!(parse(&[0x07, 1, 2]).is_err(), "a single frame that lies");
        assert!(FlowStatus::from_nibble(9).is_err(), "no such flow status");
    }
}
