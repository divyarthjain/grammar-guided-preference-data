//! Preference-pair logging (team doc §5.4): the sole interface between the
//! Rust runtime and the Python training package. One JSON object per line,
//! appended to `data/preference_pairs/*.jsonl`.

use std::fs::OpenOptions;
use std::io::{self, Write};
use std::path::Path;

use grammar::Candidate;
use serde::Serialize;
use time::format_description::well_known::Rfc3339;
use time::OffsetDateTime;

#[derive(Debug, Serialize)]
pub struct PreferencePair {
    pub image_ref: String,
    pub prompt: String,
    pub chosen: Candidate,
    pub rejected: Candidate,
    pub timestamp: String,
}

impl PreferencePair {
    pub fn new(
        image_ref: impl Into<String>,
        prompt: impl Into<String>,
        chosen: Candidate,
        rejected: Candidate,
    ) -> Self {
        let timestamp = OffsetDateTime::now_utc()
            .format(&Rfc3339)
            .expect("RFC3339 formatting of the current time should not fail");
        Self {
            image_ref: image_ref.into(),
            prompt: prompt.into(),
            chosen,
            rejected,
            timestamp,
        }
    }
}

pub fn write_pair(path: &Path, pair: &PreferencePair) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let line = serde_json::to_string(pair)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
    let mut file = OpenOptions::new().create(true).append(true).open(path)?;
    writeln!(file, "{line}")
}
