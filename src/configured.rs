//! How an ISO-TP end is configured: the CAN interface as the address, the
//! identifier and the pacing as its settings, which the declaration holds
//! (`declaration.rs`).

use std::sync::Arc;

use can_bus::Bus;
use transport::Configured;
use transport::error::Result;
use xcore::settings::Settings;

use crate::{IsoTpTransport, Pacing, declaration};

impl Configured for IsoTpTransport {
    const SETTINGS: &'static Settings = declaration::SETTINGS;
    const SCHEMES: &'static [&'static str] = declaration::SCHEMES;

    fn timeout_kept(&mut self) -> Option<&mut dyn transport::configured::TimeoutKept> {
        Some(&mut self.timeout)
    }

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
        Self::new(Arc::clone(&bus), bus, id).paced(pacing)
    }
}
