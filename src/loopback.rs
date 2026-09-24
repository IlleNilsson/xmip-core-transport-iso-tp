//! Both ends of one ISO-TP session on this machine (ADR-0051): a tester and
//! an ECU, two nodes on a fresh simulated bus per round, the ECU on the far
//! end collecting one message. Apart from the transport since 2026-09-24, when
//! lib.rs outgrew the file gate, as the loopbacks of UDS, OBD-II and J1939
//! already are.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use can_bus::Bus;
use sdk::broadcast::Medium;
use transport::Arrived;
use transport::error::{Result, protocol_error};
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};

use crate::frame::FlowStatus;
use crate::{ECU_ID, IsoTpTransport, TESTER_ID};

/// One loopback session: the tester and the ECU, each a node on one simulated
/// bus, hearing what the other transmits. Until 2026-09-24 a session was two
/// directed queues, because the in-process bus returned a node's own frames.
#[derive(Clone)]
pub(crate) struct Session {
    tester: Arc<dyn Bus>,
    ecu: Arc<dyn Bus>,
}

impl Session {
    /// A fresh bus with a tester and an ECU on it.
    fn fresh() -> Self {
        let medium = Medium::new("loopback");
        Self {
            tester: Arc::new(medium.node()),
            ecu: Arc::new(medium.node()),
        }
    }
}

/// The sessions a loopback has stood up and not yet taken, by address. A
/// fresh bus per round, so rounds driven at once from several
/// threads never read each other's frames — the file transport learned the
/// same the hard way on 2026-09-10.
pub(crate) type Standing = Arc<Mutex<HashMap<String, Session>>>;

/// Numbers the sessions, so each address names one.
static NEXT_SESSION: AtomicU64 = AtomicU64::new(1);

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
        let session = self
            .standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(address)
            .cloned()
            .ok_or_else(|| protocol_error(format!("{address} is not a session stood up here")))?;
        Ok(
            Self::new(Arc::clone(&session.tester), session.tester, TESTER_ID)
                .paced(self.pacing)
                .timing_out_after(self.timeout),
        )
    }
}

/// An ECU waiting to collect its one message. It owns the session: the
/// address is forgotten once the message is taken.
struct Ecu {
    end: IsoTpTransport,
    standing: Standing,
    address: String,
}

impl FarEnd for Ecu {
    fn address(&self) -> &str {
        &self.address
    }

    fn take_one(self: Box<Self>) -> Result<Arrived> {
        let taken = self.end.collect();
        self.standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .remove(&self.address);
        taken
    }
}

impl Loopback for IsoTpTransport {
    /// [`CLASSIC_CEILING`] unless the escape is allowed: the fact ISO 15765-2
    /// states about a first frame's twelve-bit length.
    fn ceiling(&self) -> Option<usize> {
        Some(self.ceiling())
    }

    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        let session = Session::fresh();
        let ecu = Self::new(Arc::clone(&session.ecu), Arc::clone(&session.ecu), ECU_ID)
            .paced(self.pacing)
            .timing_out_after(self.timeout);
        let address = format!(
            "isotp://loopback/{}",
            NEXT_SESSION.fetch_add(1, Ordering::Relaxed)
        );
        self.standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(address.clone(), session);
        Ok(Box::new(Ecu {
            end: ecu,
            standing: Arc::clone(&self.standing),
            address,
        }))
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
