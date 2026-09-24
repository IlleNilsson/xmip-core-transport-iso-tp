//! Both ends of one ISO-TP session on this machine (ADR-0051): a tester and
//! an ECU, two nodes on a fresh simulated bus per round, the ECU on the far
//! end collecting one message. Apart from the transport since 2026-09-24, when
//! lib.rs outgrew the file gate, as the loopbacks of UDS, OBD-II and J1939
//! already are.
//!
//! The session is public because UDS and OBD-II ride on ISO-TP and stand the
//! same tester and ECU up for their own rounds; the sessions a loopback has
//! stood up and not yet taken are the capability's
//! [`transport::standing::Standing`].

use std::sync::Arc;

use can_bus::Bus;
use sdk::broadcast::Medium;
use transport::error::Result;
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};

use crate::frame::FlowStatus;
use crate::{ECU_ID, IsoTpTransport, TESTER_ID};

/// One loopback session: the tester and the ECU, each a node on one simulated
/// bus, hearing what the other transmits. Until 2026-09-24 a session was two
/// directed queues, because the in-process bus returned a node's own frames.
#[derive(Clone)]
pub struct Session {
    /// The tester's node.
    pub tester: Arc<dyn Bus>,
    /// The ECU's node.
    pub ecu: Arc<dyn Bus>,
}

impl Session {
    /// A fresh bus with a tester and an ECU on it.
    #[must_use]
    pub fn fresh() -> Self {
        let medium = Medium::new("loopback");
        Self {
            tester: Arc::new(medium.node()),
            ecu: Arc::new(medium.node()),
        }
    }
}

impl IsoTpTransport {
    /// Both ends on this machine: a tester whose far end is an ECU, the two
    /// nodes on a fresh simulated bus per round, the loopback timeout on
    /// both. The bus this instance itself holds carries nothing;
    /// every round stands up its own.
    #[must_use]
    pub fn loopback() -> Self {
        let idle: Arc<dyn Bus> = Arc::new(Medium::new("loopback").node());
        Self::new(Arc::clone(&idle), idle, TESTER_ID).timing_out_after(LOOPBACK_TIMEOUT)
    }

    /// The tester's end of the session at `address`.
    fn tester(&self, address: &str) -> Result<Self> {
        let session = self.standing.session(address)?;
        Ok(
            Self::new(Arc::clone(&session.tester), session.tester, TESTER_ID)
                .paced(self.pacing)
                .timing_out_after(self.timeout),
        )
    }
}

impl Loopback for IsoTpTransport {
    /// [`CLASSIC_CEILING`] unless the escape is allowed: the fact ISO 15765-2
    /// states about a first frame's twelve-bit length.
    ///
    /// [`CLASSIC_CEILING`]: crate::frame::CLASSIC_CEILING
    fn ceiling(&self) -> Option<usize> {
        Some(self.ceiling())
    }

    /// An ECU waiting to collect its one message. It owns the session: the
    /// address is forgotten once the message is taken.
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        let session = Session::fresh();
        let ecu = Self::new(Arc::clone(&session.ecu), Arc::clone(&session.ecu), ECU_ID)
            .paced(self.pacing)
            .timing_out_after(self.timeout);
        let address = self.standing.stand("isotp", session);
        Ok(self.standing.far_end(address, move || ecu.collect()))
    }

    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        self.tester(address)?.deliver(payload)
    }

    /// No socket to poke. A flow-control frame is what no session opens
    /// with, so an ECU whose tester was refused reads it and is judged now
    /// rather than at its deadline.
    fn unblock(&self, address: &str) {
        if let Ok(tester) = self.tester(address) {
            drop(tester.transmit_flow(FlowStatus::Continue));
        }
    }
}
