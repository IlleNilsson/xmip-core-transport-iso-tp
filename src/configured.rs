//! How an ISO-TP end is configured: the CAN interface as the address, the
//! identifier and the pacing as its settings.

use std::sync::Arc;

use can_bus::Bus;
use transport::Configured;
use transport::error::Result;
use xcore::settings::{Applies, Kind, Presence, Setting, Settings};

use crate::{IsoTpTransport, Pacing};

impl Configured for IsoTpTransport {
    /// The address is the CAN interface, `can0`, both directions ride; the
    /// settings are the identifier and the pacing.
    const SETTINGS: &'static Settings = &Settings {
        technology: env!("CARGO_PKG_NAME"),
        settings: &[
            Setting {
                name: "id",
                kind: Kind::Integer {
                    minimum: 0,
                    maximum: 0x1fff_ffff,
                },
                presence: Presence::Required,
                meaning: "The CAN identifier this end transmits under, extended above 0x7ff.",
                applies: Applies::Both,
            },
            Setting {
                name: "block_size",
                kind: Kind::Integer {
                    minimum: 0,
                    maximum: 255,
                },
                presence: Presence::Optional,
                meaning: "Frames taken between flow controls; zero or left out means all of them.",
                applies: Applies::Both,
            },
            Setting {
                name: "separation",
                kind: Kind::Integer {
                    minimum: 0,
                    maximum: 127,
                },
                presence: Presence::Optional,
                meaning: "The minimum gap between consecutive frames, in milliseconds.",
                applies: Applies::Both,
            },
            Setting {
                name: "escape",
                kind: Kind::Boolean,
                presence: Presence::Optional,
                meaning: "Whether the 32-bit length escape carries a payload past 4095 bytes.",
                applies: Applies::Both,
            },
            Setting {
                name: "timeout",
                kind: Kind::Duration,
                presence: Presence::Optional,
                meaning: "How long a peer that stops mid-transfer is waited on.",
                applies: Applies::Both,
            },
        ],
    };

    /// The schemes a Location's address is entered with to choose this technology.
    const SCHEMES: &'static [&'static str] = &[crate::SCHEME];

    fn configured(address: &str, settings: &xcore::settings::Read) -> Result<Self> {
        let bus = can_bus::open_bus(address)?;
        Ok(Self::on(bus, settings))
    }
}

impl IsoTpTransport {
    /// This end on `bus`, both directions on the one node, as the settings
    /// read say.
    pub(crate) fn on(bus: Arc<dyn Bus>, settings: &xcore::settings::Read) -> Self {
        // The declaration holds every integer within its range.
        let byte = |name| {
            settings
                .optional_integer(name)
                .map(|n| u8::try_from(n).unwrap_or(0))
        };
        let id = u32::try_from(settings.integer("id")).unwrap_or(0);
        let unpaced = Pacing::default();
        let pacing = Pacing {
            block_size: byte("block_size").unwrap_or(unpaced.block_size),
            separation: byte("separation").unwrap_or(unpaced.separation),
            escape: settings
                .optional_boolean("escape")
                .unwrap_or(unpaced.escape),
        };
        let transport = Self::new(Arc::clone(&bus), bus, id).paced(pacing);
        match settings.optional_duration("timeout") {
            Some(timeout) => transport.timing_out_after(timeout),
            None => transport,
        }
    }
}
