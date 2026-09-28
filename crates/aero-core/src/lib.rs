//! Backend-neutral atmospheric and aerodynamic models shared by simulation
//! and authoring tools.

#![forbid(unsafe_code)]

mod aero;
mod aero_residual;
mod atmosphere;
mod high_speed;

pub use aero::{
    AeroBluntDisc, AeroCase, AeroCoefficientTable, AeroCoefficients, AeroConfig, AeroDiscLoad,
    AeroEnvironment, AeroError, AeroForceBreakdown, AeroGeometry, AeroModel, AeroPanel,
    AeroPanelLoad, AeroResult, AeroSimdScratch, AeroState, PanelAeroModel, PanelSoA,
    diederich_lift_slope, evaluate_batch,
};
pub use aero_residual::{
    AERO_RESIDUAL_TILE_EDGE, AeroCoefficientError, AeroPhysicalBudget, AeroPhysicalErrorBound,
    AeroResidualBudget, AeroResidualCodec, AeroResidualStats, AeroResidualTable, AeroResidualTile,
};
pub use atmosphere::{
    AtmosphereComposition, AtmosphereConfig, AtmosphereError, AtmosphereSample, BakedAtmosphere,
    GasKind,
};
pub use high_speed::{
    BOOM_ANCHOR_PSF, BOOM_OVERPRESSURE_GAIN, BoomCarpet, HighSpeedError, boom_carpet,
    buffet_fluctuation, buffet_gain, vapor_cone_active,
};
