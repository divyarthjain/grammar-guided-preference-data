use std::path::Path;

use judge::MockJudge;
use orchestrator::{pipeline, sampler};

fn main() {
    let candidates = sampler::stubbed_candidates();
    let judge = MockJudge::new();
    let out_path = Path::new("../data/preference_pairs/pairs.jsonl");

    match pipeline::run(&candidates, &judge, "frame_stub.png", "describe the scene", out_path) {
        Ok(written) => println!("wrote {written} preference pair(s) to {}", out_path.display()),
        Err(e) => eprintln!("pipeline run failed: {e}"),
    }
}
