//! Stub multi-sample generation (team doc §5.2). Real multi-sample llama.cpp
//! calls at varied temperature are future work; this returns a fixed set of
//! raw JSON candidate strings so the rest of the pipeline can be exercised
//! end-to-end now.

pub fn stubbed_candidates() -> Vec<String> {
    vec![
        r#"{"object":"red block","bbox":{"x":10.0,"y":20.0,"width":30.0,"height":30.0},"confidence":0.92,"action_type":"grasp"}"#.to_string(),
        r#"{"object":"table edge","bbox":{"x":0.0,"y":0.0,"width":200.0,"height":10.0},"confidence":0.4,"action_type":"avoid"}"#.to_string(),
    ]
}
