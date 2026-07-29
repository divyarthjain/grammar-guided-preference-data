//! Schema types and JSON-shape validation for model output (team doc §5.1).
//!
//! This validates that already-parsed JSON matches the required shape. It is
//! distinct from the actual GBNF grammar file (`schema.gbnf`, not yet
//! written) that constrains llama.cpp's token-level decoding so malformed
//! output can never be generated in the first place.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ActionType {
    Approach,
    Avoid,
    Grasp,
    Inspect,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BBox {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Candidate {
    pub object: String,
    pub bbox: BBox,
    pub confidence: f32,
    pub action_type: ActionType,
}

#[derive(Debug, thiserror::Error)]
pub enum ValidationError {
    #[error("malformed candidate JSON: {0}")]
    Malformed(#[from] serde_json::Error),
    #[error("confidence {0} out of range [0.0, 1.0]")]
    ConfidenceOutOfRange(f32),
}

pub fn validate(raw_json: &str) -> Result<Candidate, ValidationError> {
    let candidate: Candidate = serde_json::from_str(raw_json)?;
    if !(0.0..=1.0).contains(&candidate.confidence) {
        return Err(ValidationError::ConfidenceOutOfRange(candidate.confidence));
    }
    Ok(candidate)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_candidate_parses() {
        let json = r#"{"object":"red block","bbox":{"x":1.0,"y":2.0,"width":3.0,"height":4.0},"confidence":0.9,"action_type":"grasp"}"#;
        let candidate = validate(json).expect("should parse");
        assert_eq!(candidate.object, "red block");
        assert_eq!(candidate.action_type, ActionType::Grasp);
    }

    #[test]
    fn malformed_json_is_rejected() {
        let json = r#"{"object": "red block", "bbox": "not an object"}"#;
        assert!(matches!(validate(json), Err(ValidationError::Malformed(_))));
    }

    #[test]
    fn out_of_range_confidence_is_rejected() {
        let json = r#"{"object":"red block","bbox":{"x":0.0,"y":0.0,"width":1.0,"height":1.0},"confidence":1.5,"action_type":"none"}"#;
        assert!(matches!(
            validate(json),
            Err(ValidationError::ConfidenceOutOfRange(_))
        ));
    }
}
