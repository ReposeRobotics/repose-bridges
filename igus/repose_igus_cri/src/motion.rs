//! Point-to-point `Move Joint` tracking: local gates, one move in flight, and
//! completion judged from measured joints. No I/O.

use crate::protocol::CuCriMotionRejection;
use crate::protocol::CuCriMotionState;
use crate::protocol::CuCriMotionStatus;
use crate::protocol::CuCriStatus;
use crate::protocol::CuCriText;
use cu29::units::si::angle::degree;
use cu29::units::si::f64::Angle;
#[cfg(feature = "control")]
use repose_kinematics::CuJointPosition;
#[cfg(feature = "control")]
use repose_kinematics::igus_rebel_chain;
use repose_robot_interface::CuSessionState;

#[cfg(feature = "control")]
/// Targets closer than this are the same command (Copper resends every cycle).
const SAME_TARGET: f64 = 1e-6;

#[cfg(feature = "control")]
/// What the bridge knows when a target arrives.
pub(crate) struct MoveContext<'a> {
    pub session: &'a CuSessionState,
    pub measured: Option<&'a CuCriStatus>,
    pub max_step: Option<Angle>,
}

#[derive(Default, Debug, Clone)]
pub(crate) struct MotionTracker {
    state: CuCriMotionState,
    outgoing: Option<[Angle; 6]>,
    changed: bool,
}

#[cfg(feature = "control")]
fn same(a: &[Angle; 6], b: &[Angle; 6]) -> bool {
    a.iter()
        .zip(b)
        .all(|(a, b)| (a.get::<degree>() - b.get::<degree>()).abs() < SAME_TARGET)
}

impl MotionTracker {
    pub(crate) fn state(&self) -> &CuCriMotionState {
        &self.state
    }

    pub(crate) fn take_changed(&mut self) -> bool {
        core::mem::take(&mut self.changed)
    }

    pub(crate) fn in_flight(&self) -> bool {
        matches!(
            self.state.status,
            CuCriMotionStatus::Pending | CuCriMotionStatus::Moving
        )
    }

    #[cfg(feature = "control")]
    /// Accept or refuse a target. A target equal to the current one is a
    /// no-op whatever its outcome, so a held setpoint never re-triggers.
    pub(crate) fn request(&mut self, target: [Angle; 6], ctx: &MoveContext<'_>) {
        if self.state.status != CuCriMotionStatus::Idle && same(&target, &self.state.target) {
            return;
        }
        self.changed = true;
        if self.in_flight() {
            self.state.refused = self.state.refused.saturating_add(1);
            return;
        }
        let refusal = if !ctx.session.control_acquired {
            Some(CuCriMotionRejection::NotInControl)
        } else if !ctx.session.motion_allowed {
            Some(CuCriMotionRejection::MotionNotAllowed)
        } else if !igus_rebel_chain().is_within_limits(&target.map(CuJointPosition::Revolute)) {
            Some(CuCriMotionRejection::JointLimits)
        } else {
            match (ctx.measured, ctx.max_step) {
                (None, _) => Some(CuCriMotionRejection::MotionNotAllowed),
                (Some(_), None) => None,
                (Some(measured), Some(limit)) => {
                    let limit = limit.get::<degree>();
                    let too_far = measured
                        .joints
                        .iter()
                        .zip(&target)
                        .any(|(m, t)| (t.get::<degree>() - m.get::<degree>()).abs() > limit);
                    too_far.then_some(CuCriMotionRejection::MaxStepExceeded)
                }
            }
        };
        self.state.target = target;
        self.state.detail = CuCriText::default();
        match refusal {
            Some(reason) => {
                self.state.status = CuCriMotionStatus::Rejected;
                self.state.rejection = reason;
            }
            None => {
                self.state.status = CuCriMotionStatus::Pending;
                self.state.rejection = CuCriMotionRejection::None;
                self.state.sequence = 0;
                self.outgoing = Some(target);
            }
        }
    }

    pub(crate) fn take_outgoing(&mut self) -> Option<[Angle; 6]> {
        self.outgoing.take()
    }

    pub(crate) fn mark_sent(&mut self, sequence: u32) {
        self.state.sequence = sequence;
        self.changed = true;
    }

    fn current(&self, sequence: u32) -> bool {
        self.in_flight() && self.state.sequence == sequence && sequence != 0
    }

    pub(crate) fn on_acknowledged(&mut self, sequence: u32) {
        if self.current(sequence) && self.state.status == CuCriMotionStatus::Pending {
            self.state.status = CuCriMotionStatus::Moving;
            self.changed = true;
        }
    }

    pub(crate) fn on_rejected(&mut self, sequence: u32, detail: &str) {
        if self.current(sequence) {
            self.state.status = CuCriMotionStatus::Rejected;
            self.state.rejection = CuCriMotionRejection::Controller;
            self.state.detail = CuCriText::new(detail);
            self.changed = true;
        }
    }

    /// Judge an in-flight move from a STATUS frame and the session.
    pub(crate) fn on_status(
        &mut self,
        status: &CuCriStatus,
        session: &CuSessionState,
        tolerance: Angle,
    ) {
        if !self.in_flight() {
            return;
        }
        let tolerance = tolerance.get::<degree>();
        if !session.control_acquired || !status.drives_ready() || status.joint_fault().is_some() {
            self.abort("control, drives or joint fault lost during move");
        } else if status
            .joints
            .iter()
            .zip(&self.state.target)
            .all(|(m, t)| (m.get::<degree>() - t.get::<degree>()).abs() <= tolerance)
        {
            self.state.status = CuCriMotionStatus::Reached;
            self.changed = true;
        }
    }

    /// Record an informational `EXEC*` frame.
    pub(crate) fn on_exec(&mut self, kind: &str, body: &str) {
        let mut text = [0u8; 32];
        let mut len = 0;
        for byte in kind.bytes().chain(*b" ").chain(body.bytes()).take(32) {
            text[len] = byte;
            len += 1;
        }
        self.state.last_exec = CuCriText::new(core::str::from_utf8(&text[..len]).unwrap_or(kind));
        self.changed = true;
    }

    pub(crate) fn on_disconnected(&mut self) {
        if self.in_flight() {
            self.abort("transport lost");
        }
        self.outgoing = None;
    }

    fn abort(&mut self, why: &str) {
        self.outgoing = None;
        self.state.status = CuCriMotionStatus::Aborted;
        self.state.rejection = CuCriMotionRejection::Aborted;
        self.state.detail = CuCriText::new(why);
        self.changed = true;
    }
}

#[cfg(all(test, feature = "control"))]
mod tests {
    use super::*;

    fn deg(values: [f64; 6]) -> [Angle; 6] {
        values.map(Angle::new::<degree>)
    }

    fn ready() -> CuSessionState {
        CuSessionState {
            control_acquired: true,
            drives_enabled: true,
            drives_ready: true,
            motion_allowed: true,
            ..Default::default()
        }
    }

    fn measured(values: [f64; 6]) -> CuCriStatus {
        CuCriStatus {
            joints: deg(values),
            kinstate: Some(0),
            ..Default::default()
        }
    }

    fn ctx<'a>(session: &'a CuSessionState, status: &'a CuCriStatus) -> MoveContext<'a> {
        MoveContext {
            session,
            measured: Some(status),
            max_step: Some(Angle::new::<degree>(10.0)),
        }
    }

    #[test]
    fn test_gates() {
        let status = measured([0.0; 6]);
        let target = deg([0.0, 0.0, 0.0, 0.0, 0.0, 5.0]);
        for (session, reason) in [
            (
                CuSessionState::default(),
                CuCriMotionRejection::NotInControl,
            ),
            (
                CuSessionState {
                    control_acquired: true,
                    ..Default::default()
                },
                CuCriMotionRejection::MotionNotAllowed,
            ),
        ] {
            let mut tracker = MotionTracker::default();
            tracker.request(target, &ctx(&session, &status));
            assert_eq!(tracker.state().rejection, reason);
            assert!(tracker.take_outgoing().is_none());
        }
        let session = ready();
        let mut tracker = MotionTracker::default();
        tracker.request(
            deg([0.0, 0.0, 0.0, 0.0, 0.0, 11.0]),
            &ctx(&session, &status),
        );
        assert_eq!(
            tracker.state().rejection,
            CuCriMotionRejection::MaxStepExceeded
        );
        let far = measured([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let mut tracker = MotionTracker::default();
        tracker.request(deg([0.0, 0.0, 0.0, 0.0, 0.0, 400.0]), &ctx(&session, &far));
        assert_eq!(tracker.state().rejection, CuCriMotionRejection::JointLimits);
    }

    #[test]
    fn test_move_lifecycle() {
        let session = ready();
        let status = measured([0.0; 6]);
        let target = deg([0.0, 0.0, 0.0, 0.0, 0.0, 5.0]);
        let mut tracker = MotionTracker::default();
        tracker.request(target, &ctx(&session, &status));
        assert_eq!(tracker.take_outgoing(), Some(target));
        tracker.mark_sent(7);
        // Resending the same target is a no-op; a different one is refused.
        tracker.request(target, &ctx(&session, &status));
        tracker.request(deg([0.0, 0.0, 0.0, 0.0, 0.0, 4.0]), &ctx(&session, &status));
        assert_eq!(tracker.state().refused, 1);
        assert!(tracker.take_outgoing().is_none());
        tracker.on_acknowledged(6);
        assert_eq!(tracker.state().status, CuCriMotionStatus::Pending);
        tracker.on_acknowledged(7);
        assert_eq!(tracker.state().status, CuCriMotionStatus::Moving);
        let tol = Angle::new::<degree>(0.5);
        tracker.on_status(&measured([0.0, 0.0, 0.0, 0.0, 0.0, 4.0]), &session, tol);
        assert_eq!(tracker.state().status, CuCriMotionStatus::Moving);
        tracker.on_status(&measured([0.0, 0.0, 0.0, 0.0, 0.0, 4.6]), &session, tol);
        assert_eq!(tracker.state().status, CuCriMotionStatus::Reached);
        // Held setpoint after arrival does not move again.
        tracker.request(target, &ctx(&session, &status));
        assert!(tracker.take_outgoing().is_none());
    }

    #[test]
    fn test_controller_rejection_and_abort() {
        let session = ready();
        let status = measured([0.0; 6]);
        let mut tracker = MotionTracker::default();
        tracker.request(deg([0.0, 0.0, 0.0, 0.0, 0.0, 5.0]), &ctx(&session, &status));
        tracker.take_outgoing();
        tracker.mark_sent(3);
        tracker.on_rejected(3, "not allowed");
        assert_eq!(tracker.state().rejection, CuCriMotionRejection::Controller);
        assert_eq!(tracker.state().detail.as_str(), "not allowed");

        let mut tracker = MotionTracker::default();
        tracker.request(deg([0.0, 0.0, 0.0, 0.0, 0.0, 5.0]), &ctx(&session, &status));
        tracker.take_outgoing();
        tracker.mark_sent(4);
        let lost = CuSessionState::default();
        tracker.on_status(&status, &lost, Angle::new::<degree>(0.5));
        assert_eq!(tracker.state().status, CuCriMotionStatus::Aborted);

        let mut tracker = MotionTracker::default();
        tracker.request(deg([0.0, 0.0, 0.0, 0.0, 0.0, 5.0]), &ctx(&session, &status));
        tracker.on_disconnected();
        assert_eq!(tracker.state().status, CuCriMotionStatus::Aborted);
        assert!(tracker.take_outgoing().is_none());
    }
}
