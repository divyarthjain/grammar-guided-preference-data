use judge::MockJudge;
use orchestrator::{pipeline, sampler};

#[test]
fn stubbed_pipeline_writes_a_preference_pair() {
    let tmp = tempfile::tempdir().expect("create temp dir");
    let out_path = tmp.path().join("pairs.jsonl");
    let judge = MockJudge::new();
    let candidates = sampler::stubbed_candidates();

    let written = pipeline::run(&candidates, &judge, "frame_test.png", "describe the scene", &out_path)
        .expect("pipeline run should succeed");
    assert_eq!(written, 1);

    let contents = std::fs::read_to_string(&out_path).expect("read output file");
    let lines: Vec<&str> = contents.lines().collect();
    assert_eq!(lines.len(), 1);

    let value: serde_json::Value = serde_json::from_str(lines[0]).expect("valid JSON line");
    assert_eq!(value["image_ref"], "frame_test.png");
    assert!(value["chosen"].is_object());
    assert!(value["rejected"].is_object());
    assert!(value["timestamp"].is_string());
}
