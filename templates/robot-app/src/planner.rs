//! Planner selection for `Plan` steps: every Repose planner behind one enum,
//! chosen by the `planner` key in copperconfig.ron.

use cu29::prelude::*;
use repose_collision::CuRobotCollisionModel;
use repose_joint_rrt_star::CuJointRrtStar;
use repose_kinematics::CuKinematicChain;
use repose_motion_planning::CuJointAStar;
use repose_motion_planning::CuJointDirect;
use repose_motion_planning::CuJointPathOptimizer;
use repose_motion_planning::CuJointPlanPath;
use repose_motion_planning::CuJointPlanRequest;
use repose_motion_planning::CuJointPlanner;
use repose_motion_planning::CuJointPrm;
use repose_motion_planning::CuJointRrtConnect;
use repose_motion_planning::CuJointStomp;
use repose_motion_planning::CuPlanStatus;
use serde::Deserialize;

/// Which planner `Plan` steps use (RON spelling in parentheses).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannerKind {
    /// Bidirectional RRT, fast feasibility-first (`rrt_connect`).
    #[default]
    RrtConnect,
    /// Asymptotically optimal RRT* with rewiring and shortcutting (`rrt_star`).
    RrtStar,
    /// Probabilistic roadmap (`prm`).
    Prm,
    /// A* on a joint-space lattice, deterministic (`a_star`).
    AStar,
    /// Stochastic trajectory optimization (`stomp`).
    Stomp,
    /// Gradient path optimizer, CHOMP-style (`path_optimizer`).
    PathOptimizer,
    /// Straight joint line, collision-checked (`direct`).
    Direct,
}

/// One of the Repose planners, with default parameters.
pub enum AnyPlanner {
    RrtConnect(Box<CuJointRrtConnect<6>>),
    RrtStar(Box<CuJointRrtStar<6>>),
    Prm(Box<CuJointPrm<6>>),
    AStar(Box<CuJointAStar<6>>),
    Stomp(Box<CuJointStomp<6>>),
    PathOptimizer(Box<CuJointPathOptimizer<6>>),
    Direct(Box<CuJointDirect<6>>),
}

macro_rules! each {
    ($self:expr, $p:ident => $body:expr) => {
        match $self {
            AnyPlanner::RrtConnect($p) => $body,
            AnyPlanner::RrtStar($p) => $body,
            AnyPlanner::Prm($p) => $body,
            AnyPlanner::AStar($p) => $body,
            AnyPlanner::Stomp($p) => $body,
            AnyPlanner::PathOptimizer($p) => $body,
            AnyPlanner::Direct($p) => $body,
        }
    };
}

impl AnyPlanner {
    pub fn new(
        kind: PlannerKind,
        chain: CuKinematicChain<6>,
        model: CuRobotCollisionModel<6>,
    ) -> CuResult<Self> {
        Ok(match kind {
            PlannerKind::RrtConnect => {
                Self::RrtConnect(Box::new(CuJointRrtConnect::new(chain, model, Default::default())?))
            }
            PlannerKind::RrtStar => {
                Self::RrtStar(Box::new(CuJointRrtStar::new(chain, model, Default::default())?))
            }
            PlannerKind::Prm => Self::Prm(Box::new(CuJointPrm::new(chain, model, Default::default())?)),
            PlannerKind::AStar => {
                Self::AStar(Box::new(CuJointAStar::new(chain, model, Default::default())?))
            }
            PlannerKind::Stomp => {
                Self::Stomp(Box::new(CuJointStomp::new(chain, model, Default::default())?))
            }
            PlannerKind::PathOptimizer => Self::PathOptimizer(Box::new(CuJointPathOptimizer::new(
                chain,
                model,
                Default::default(),
            )?)),
            PlannerKind::Direct => {
                Self::Direct(Box::new(CuJointDirect::new(chain, model, Default::default())?))
            }
        })
    }

    pub fn begin(&mut self, request: &CuJointPlanRequest<6>, seed: u64) -> CuResult<CuPlanStatus> {
        each!(self, p => p.begin(request, seed))
    }

    pub fn step(&mut self, budget: u32) -> CuResult<CuPlanStatus> {
        each!(self, p => p.step(budget))
    }

    pub fn write_best(&self, out: &mut CuJointPlanPath<6>) -> CuPlanStatus {
        each!(self, p => p.write_best(out))
    }
}
