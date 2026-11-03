//! ABB OmniCore bridge over Externally Guided Motion (EGM, UDP + protobuf).
//!
//! Default builds only listen to EGM feedback. The `control` feature replies
//! with joint corrections inside an explicitly enabled session.
mod bridge;
#[cfg(feature = "control")]
mod control;
mod mapping;
mod params;
mod protocol;
mod transport;
pub use bridge::{AbbRxChannels, AbbRxId, AbbTxChannels, AbbTxId, CuAbbBridge};
pub use params::CuAbbParams;
pub use protocol::{CuAbbMciState, CuAbbMotorState, CuAbbRapidState, CuAbbStatus};
