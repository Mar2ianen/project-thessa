//! Bevy resource adapter for the owned native server link. Transport,
//! receive timing, bounded mailboxes and child lifecycle live in flight-net.
use std::time::Duration;

#[derive(bevy::prelude::Resource, bevy::prelude::Deref)]
pub struct EmbeddedLink(thessa_flight_net::embedded::EmbeddedLink);

impl EmbeddedLink {
    pub fn spawn(handshake_timeout: Duration) -> Result<Self, String> {
        thessa_flight_net::embedded::EmbeddedLink::spawn(handshake_timeout).map(Self)
    }
}
