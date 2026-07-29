//! The automatic physical judge (team doc §5.3): checks whether a candidate
//! action is physically feasible (IK reachable) and safe (Ruckig trajectory
//! within limits), labeling it Chosen or Rejected with no human involved.
//!
//! The real judge depends on the robot's IK solver and Ruckig integration,
//! which don't exist yet — see `judge-ffi`. `MockJudge` here is a working
//! stand-in so the rest of the pipeline can be built and tested now; it
//! implements the same `PhysicalJudge` trait a real judge will implement
//! later, so callers don't change when the swap happens.

use std::cell::Cell;

use grammar::Candidate;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    Chosen,
    Rejected,
}

pub trait PhysicalJudge {
    fn check(&self, candidate: &Candidate) -> Verdict;
}

/// Alternates Chosen/Rejected on each call, regardless of the candidate, so
/// pipeline code exercises both branches without any hardware or IK/Ruckig
/// dependency.
pub struct MockJudge {
    next_is_chosen: Cell<bool>,
}

impl MockJudge {
    pub fn new() -> Self {
        Self {
            next_is_chosen: Cell::new(true),
        }
    }
}

impl Default for MockJudge {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicalJudge for MockJudge {
    fn check(&self, _candidate: &Candidate) -> Verdict {
        let chosen = self.next_is_chosen.get();
        self.next_is_chosen.set(!chosen);
        if chosen {
            Verdict::Chosen
        } else {
            Verdict::Rejected
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use grammar::{ActionType, BBox};

    fn sample_candidate() -> Candidate {
        Candidate {
            object: "red block".to_string(),
            bbox: BBox {
                x: 0.0,
                y: 0.0,
                width: 1.0,
                height: 1.0,
            },
            confidence: 0.9,
            action_type: ActionType::Grasp,
        }
    }

    #[test]
    fn mock_judge_alternates_verdicts() {
        let judge = MockJudge::new();
        let candidate = sample_candidate();
        assert_eq!(judge.check(&candidate), Verdict::Chosen);
        assert_eq!(judge.check(&candidate), Verdict::Rejected);
        assert_eq!(judge.check(&candidate), Verdict::Chosen);
    }
}
