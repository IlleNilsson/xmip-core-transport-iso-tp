#![forbid(unsafe_code)]

//! Streams too long for one CAN frame, segmented the way ISO 15765-2 says.
//!
//! A classical CAN frame carries eight bytes, and most of what a diagnostic
//! tester or an ECU says does not fit. ISO-TP is the layer that makes it fit:
//! a payload of up to 4095 bytes — or, with the length escape, far more — is
//! split into a first frame and a run of consecutive frames, and the receiver
//! paces the sender with flow control, a block size and a minimum separation
//! time. A payload of seven bytes or fewer skips all of that and rides in one
//! single frame.
//!
//! The carrier is [`can_bus`](can_bus): ISO-TP reuses its
//! [`Bus`](can_bus::Bus), [`Frame`](can_bus::Frame) and
//! [`Loopback`](can_bus::Loopback) rather than knowing a wire of its
//! own. Two directed buses stand for the two CAN identifiers a session uses —
//! one carries the segmented data one way, the other carries flow control back
//! — so a [`Sender`] and a [`Receiver`] round-trip in process with no hardware.
//! UDS and OBD-II ride on this crate; what the reassembled bytes mean is
//! theirs.
//!
//! The origin URI names the identifier the data arrived under:
//! `isotp://<bus>/0x<id>`.

pub mod frame;

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_bus::{Bus, Frame};
use transport::error::{Result, protocol_error};
use transport::{Arrived, Directions, Transport};

use crate::frame::{CLASSIC_CEILING, CONSECUTIVE_DATA, ESCAPE_DATA, FIRST_DATA, FlowStatus, Pci};

/// How a session is paced and how far it will stretch. The default is no
/// block size, no separation and no escape.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pacing {
    /// Frames the receiver takes between flow controls; zero means all of them.
    pub block_size: u8,
    /// The minimum gap the receiver asks the sender to leave, in milliseconds
    /// (the sub-millisecond encodings of ISO 15765-2 are not modelled).
    pub separation: u8,
    /// Whether the 32-bit length escape is permitted; without it the payload
    /// ceiling is [`CLASSIC_CEILING`].
    pub escape: bool,
}

/// One end of an ISO-TP session over CAN.
///
/// `outbound` carries what this end transmits — its segmented frames when it
/// sends, its flow control when it receives; `inbound` carries what the other
/// end transmits. The peer is the same with the two buses swapped.
pub struct IsoTpTransport {
    outbound: Arc<dyn Bus>,
    inbound: Arc<dyn Bus>,
    id: u32,
    pacing: Pacing,
    timeout: Duration,
}

impl IsoTpTransport {
    /// A session that transmits under `id` on `outbound` and reads the peer
    /// on `inbound`.
    #[must_use]
    pub fn new(outbound: Arc<dyn Bus>, inbound: Arc<dyn Bus>, id: u32) -> Self {
        Self {
            outbound,
            inbound,
            id,
            pacing: Pacing::default(),
            timeout: Duration::from_secs(1),
        }
    }

    /// The block size and separation this end asks for as a receiver, and
    /// whether the length escape is allowed.
    #[must_use]
    pub const fn paced(mut self, pacing: Pacing) -> Self {
        self.pacing = pacing;
        self
    }

    /// Give up on a peer that stops mid-transfer.
    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// The largest payload this session will carry whole: [`CLASSIC_CEILING`],
    /// or far more when the escape is allowed.
    #[must_use]
    pub const fn ceiling(&self) -> usize {
        if self.pacing.escape {
            u32::MAX as usize
        } else {
            CLASSIC_CEILING
        }
    }

    /// Segment `payload` onto the data bus, obeying the flow control the peer
    /// returns.
    ///
    /// # Errors
    /// A payload over the ceiling, a peer that overflows or never answers, or
    /// a bus that refused a frame.
    pub fn deliver(&self, payload: &[u8]) -> Result<()> {
        if payload.len() > self.ceiling() {
            return Err(protocol_error(format!(
                "{} bytes is over the ISO-TP ceiling of {}",
                payload.len(),
                self.ceiling()
            )));
        }
        if payload.len() <= frame::SINGLE_MAX {
            return self.transmit(&frame::single(payload)?);
        }
        let opening = if payload.len() > CLASSIC_CEILING {
            ESCAPE_DATA
        } else {
            FIRST_DATA
        };
        self.transmit(&frame::first(
            payload.len(),
            &payload[..opening],
            self.pacing.escape,
        )?)?;
        self.send_consecutive(&payload[opening..])
    }

    /// Read one segmented message off the data bus, pacing the sender with
    /// flow control.
    ///
    /// # Errors
    /// A malformed frame, a sequence out of order, or a peer that stops.
    pub fn collect(&self) -> Result<Arrived> {
        let origin = format!("isotp://{}/{:#x}", self.inbound.name(), self.id);
        match self.read_pci()? {
            Pci::Single { data } => Ok(Arrived::new(origin, data)),
            Pci::First { length, data } => Ok(Arrived::new(origin, self.reassemble(length, data)?)),
            _ => Err(protocol_error("a session that did not open with data")),
        }
    }

    fn send_consecutive(&self, rest: &[u8]) -> Result<()> {
        self.await_clearance()?;
        let chunks: Vec<&[u8]> = rest.chunks(CONSECUTIVE_DATA).collect();
        let total = chunks.len();
        let mut index: u8 = 1;
        let mut in_block: u8 = 0;
        for (n, chunk) in chunks.into_iter().enumerate() {
            self.transmit(&frame::consecutive(index, chunk)?)?;
            index = index.wrapping_add(1);
            in_block = in_block.wrapping_add(1);
            let more = n + 1 < total;
            if self.pacing.block_size != 0 && in_block == self.pacing.block_size && more {
                in_block = 0;
                self.await_clearance()?;
            }
            if self.pacing.separation != 0 {
                std::thread::sleep(Duration::from_millis(u64::from(self.pacing.separation)));
            }
        }
        Ok(())
    }

    fn await_clearance(&self) -> Result<()> {
        let deadline = Instant::now() + self.timeout;
        loop {
            match self.read_flow()? {
                FlowStatus::Continue => return Ok(()),
                FlowStatus::Wait => {}
                FlowStatus::Overflow => {
                    return Err(protocol_error("the receiver overflowed"));
                }
            }
            if Instant::now() >= deadline {
                return Err(protocol_error("no flow control before the deadline"));
            }
        }
    }

    fn reassemble(&self, length: usize, opening: Vec<u8>) -> Result<Vec<u8>> {
        let mut bytes = opening;
        bytes.truncate(length);
        self.transmit_flow(FlowStatus::Continue)?;
        let mut expected: u8 = 1;
        let mut in_block: u8 = 0;
        while bytes.len() < length {
            let Pci::Consecutive { index, data } = self.read_pci()? else {
                return Err(protocol_error("a consecutive frame was expected"));
            };
            if index != (expected & 0x0f) {
                return Err(protocol_error("a consecutive frame out of sequence"));
            }
            let want = (length - bytes.len()).min(data.len());
            bytes.extend_from_slice(&data[..want]);
            expected = expected.wrapping_add(1);
            in_block = in_block.wrapping_add(1);
            if self.pacing.block_size != 0
                && in_block == self.pacing.block_size
                && bytes.len() < length
            {
                in_block = 0;
                self.transmit_flow(FlowStatus::Continue)?;
            }
        }
        Ok(bytes)
    }

    fn transmit(&self, bytes: &[u8]) -> Result<()> {
        self.outbound
            .transmit(&Frame::new(self.id, self.id > 0x7ff, bytes)?)
    }

    fn transmit_flow(&self, status: FlowStatus) -> Result<()> {
        let bytes = frame::flow(status, self.pacing.block_size, self.pacing.separation);
        self.outbound
            .transmit(&Frame::new(self.id, self.id > 0x7ff, &bytes)?)
    }

    fn read_pci(&self) -> Result<Pci> {
        frame::parse(&self.read_frame()?)
    }

    fn read_flow(&self) -> Result<FlowStatus> {
        match frame::parse(&self.read_frame()?)? {
            Pci::Flow { status, .. } => Ok(status),
            _ => Err(protocol_error("a frame that was not flow control")),
        }
    }

    fn read_frame(&self) -> Result<Vec<u8>> {
        let deadline = Instant::now() + self.timeout;
        loop {
            if let Some(frame) = self.inbound.receive(self.timeout)? {
                return Ok(frame.data);
            }
            if Instant::now() >= deadline {
                return Err(protocol_error("no ISO-TP frame before the deadline"));
            }
        }
    }
}

impl Transport for IsoTpTransport {
    fn name(&self) -> &'static str {
        "iso-tp"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    fn receive(&self) -> Result<Vec<Arrived>> {
        Ok(vec![self.collect()?])
    }

    fn send(&self, _target: &str, bytes: &[u8]) -> Result<()> {
        self.deliver(bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use can_bus::Loopback;

    /// A tester and an ECU sharing two directed buses, the ECU on a thread so
    /// flow control flows while the tester sends.
    fn round_trip(payload: &[u8], pacing: Pacing) -> Vec<u8> {
        let to_ecu: Arc<dyn Bus> = Arc::new(Loopback::new());
        let to_tester: Arc<dyn Bus> = Arc::new(Loopback::new());
        let tester = IsoTpTransport::new(Arc::clone(&to_ecu), Arc::clone(&to_tester), 0x7e0)
            .paced(pacing)
            .timing_out_after(Duration::from_secs(2));
        let ecu = IsoTpTransport::new(Arc::clone(&to_tester), Arc::clone(&to_ecu), 0x7e8)
            .paced(pacing)
            .timing_out_after(Duration::from_secs(2));
        let sent = payload.to_owned();
        let collector = std::thread::spawn(move || ecu.collect());
        tester.deliver(&sent).expect("delivering");
        collector.join().expect("thread").expect("collecting").bytes
    }

    #[test]
    fn a_short_payload_rides_one_single_frame() {
        assert_eq!(round_trip(b"hello", Pacing::default()), b"hello");
        assert_eq!(round_trip(&[], Pacing::default()), b"");
    }

    #[test]
    fn a_long_payload_segments_and_reassembles_whole() {
        let payload: Vec<u8> = (0..3000)
            .map(|n| u8::try_from(n % 256).unwrap_or(0))
            .collect();
        assert_eq!(round_trip(&payload, Pacing::default()), payload);
    }

    #[test]
    fn block_size_and_separation_pace_the_transfer_the_same() {
        let payload: Vec<u8> = (0..1000)
            .map(|n| u8::try_from(n % 251).unwrap_or(0))
            .collect();
        let pacing = Pacing {
            block_size: 4,
            separation: 1,
            escape: false,
        };
        assert_eq!(round_trip(&payload, pacing), payload);
    }

    #[test]
    fn the_escape_carries_beyond_the_classic_ceiling() {
        let pacing = Pacing {
            block_size: 0,
            separation: 0,
            escape: true,
        };
        let payload: Vec<u8> = (0..5000)
            .map(|n| u8::try_from(n % 256).unwrap_or(0))
            .collect();
        assert_eq!(round_trip(&payload, pacing), payload);
    }

    #[test]
    fn a_payload_over_the_ceiling_is_refused_without_the_escape() {
        let bus: Arc<dyn Bus> = Arc::new(Loopback::new());
        let flow: Arc<dyn Bus> = Arc::new(Loopback::new());
        let tester = IsoTpTransport::new(Arc::clone(&bus), Arc::clone(&flow), 0x7e0);
        assert_eq!(tester.ceiling(), CLASSIC_CEILING);
        assert!(tester.deliver(&vec![0u8; 5000]).is_err());
        assert_eq!(tester.name(), "iso-tp");
        assert!(tester.claims().is_none());
    }
}
