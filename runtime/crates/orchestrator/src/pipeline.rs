//! The wiring for one loop iteration (team doc §4): sampled candidates ->
//! grammar validation -> physical judge -> a logged preference pair.
//! A plain function, not inlined in `main`, so it's callable from tests.

use std::io;
use std::path::Path;

use grammar::validate;
use judge::{PhysicalJudge, Verdict};

use crate::pairs::{write_pair, PreferencePair};

/// Validates each raw candidate, judges it, and — if at least one Chosen and
/// one Rejected candidate are found — writes them as a preference pair to
/// `out_path`. Malformed candidates are dropped, not logged. Returns the
/// number of pairs written (0 or 1).
pub fn run(
    raw_candidates: &[String],
    judge: &impl PhysicalJudge,
    image_ref: &str,
    prompt: &str,
    out_path: &Path,
) -> io::Result<usize> {
    let mut chosen = None;
    let mut rejected = None;

    for raw in raw_candidates {
        let Ok(candidate) = validate(raw) else {
            continue;
        };
        match judge.check(&candidate) {
            Verdict::Chosen if chosen.is_none() => chosen = Some(candidate),
            Verdict::Rejected if rejected.is_none() => rejected = Some(candidate),
            _ => {}
        }
    }

    match (chosen, rejected) {
        (Some(chosen), Some(rejected)) => {
            let pair = PreferencePair::new(image_ref, prompt, chosen, rejected);
            write_pair(out_path, &pair)?;
            Ok(1)
        }
        _ => Ok(0),
    }
}
