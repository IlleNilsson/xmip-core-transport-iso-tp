//! This technology's declaration: the transport guide's "The declaration",
//! `module/core/capability/transport/doc/adding-a-transport.md`. The wait a
//! Location takes unless told otherwise is declared here, and the code takes
//! it from here; so are the identifiers of ISO 15765-4's physical pair, which
//! UDS's declaration defaults to, and UDS and OBD-II take the wait as theirs,
//! each by this crate's `declaration` feature.

use std::time::Duration;

use xcore::settings::{Applies, Kind, Presence, Setting, Settings};

/// The identifier a tester transmits under, and the one the first ECU answers
/// from: the physical request and response pair of ISO 15765-4.
pub const TESTER_ID: u32 = 0x7e0;
/// See [`TESTER_ID`].
pub const ECU_ID: u32 = 0x7e8;

/// How long a peer that stops mid-transfer is waited on unless a Location
/// says.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

/// The scheme an origin opens with, `isotp://<bus>/0x<id>`, and the one a
/// Location's address is entered with.
pub(crate) const SCHEME: &str = "isotp";

/// Every setting a Location takes beyond its address, the CAN interface,
/// `can0`, both directions ride: the identifier and the pacing.
pub const SETTINGS: &Settings = &Settings {
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
        transport::configured::timeout(
            Applies::Both,
            Some(DEFAULT_TIMEOUT),
            "How long a peer that stops mid-transfer is waited on.",
        ),
    ],
};

/// This technology's schemes, as [`transport::Configured::SCHEMES`] defines
/// them.
pub const SCHEMES: &[&str] = &[SCHEME];
