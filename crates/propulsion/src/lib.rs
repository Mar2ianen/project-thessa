//! Backend-neutral propulsion models and vehicle-mounted feed components.

#![forbid(unsafe_code)]

#[cfg(test)]
use thessa_aero_core::*;

mod feed;
mod propulsion;

pub(crate) mod atmosphere {
    pub use thessa_aero_core::{
        AtmosphereComposition, AtmosphereConfig, AtmosphereSample, GasKind,
    };
}

pub use feed::{CompiledTank, FEED_MAX_VELOCITY_MPS, FeedLine, TankMount, TankShape, TankSpec};
pub use propulsion::*;
