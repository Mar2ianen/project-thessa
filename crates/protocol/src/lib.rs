//! Reusable wire/domain protocol primitives.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProtocolVersion(pub u16);

impl ProtocolVersion {
    pub const CURRENT: Self = Self(1);
}
