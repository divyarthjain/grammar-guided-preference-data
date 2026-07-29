---
type: Reference
title: Glossary
description: Terms used throughout this project
tags: [glossary]
status: draft
sources: [Grammar_Guided_Preference_Data_Team_Doc.docx]
generated: { by: claude-code/claude-sonnet-5, at: 2026-07-30T00:00:00Z }
---

Terms marked from the team doc §11; the last four were added from the
model-choice research (see [architecture.md](architecture.md)).

| Term | Plain-English meaning |
|---|---|
| VLM (Vision-Language Model) | An AI model that can look at an image and answer questions about it, or describe it, in text. |
| MiniCPM-V | OpenBMB's small VLM family. Version numbering does not track parameter count — e.g. "4.6" (~1.3B params) is smaller than "4.5" (~8B). Currently this project's working assumption for the model, pending an empirical grounding test. |
| GGUF | A file format for storing model weights that llama.cpp (our inference engine) can run efficiently, including on CPUs and Apple Silicon. |
| llama.cpp | Software that runs language/vision models locally and efficiently, on many kinds of hardware (phones, laptops, Jetson boards, etc.). |
| GBNF | A grammar format llama.cpp supports, used to force a model's output into an exact, guaranteed structure while it's generating text. |
| Grammar-constrained decoding | The technique of using a grammar (like GBNF) to make it impossible for the model to output anything that doesn't match a required format. |
| IK (Inverse Kinematics) | The math/software that figures out what joint angles a robot arm/leg needs in order to reach a target position. |
| Ruckig | A software library that generates smooth, jerk-limited motion trajectories in real time, respecting speed/acceleration limits. |
| PID control | A classic feedback-control method that continuously adjusts an actuator to track a target, correcting for error over time. |
| DPO (Direct Preference Optimization) | A training method that teaches a model to prefer one output over another, given pairs of (worse, better) examples — without needing a perfect "correct answer". |
| Preference pair | One training example for DPO: the same input, with one output marked chosen (better) and another marked rejected (worse). |
| WSD scheduler (Warmup-Stable-Decay) | The learning-rate schedule MiniCPM's creators use for training; the "decay" stage is where cheap specialization/fine-tuning happens most effectively. |
| Micro-anneal / decay-stage fine-tune | A short, cheap training pass on a small amount of targeted data, applied to an already-trained model, instead of retraining everything from scratch. |
| RLAIF (Reinforcement Learning from AI Feedback) | A family of techniques where an AI system (or, in our case, a robot's own outcomes) provides the "reward" signal used to improve a model, instead of a human. |
| Confidence calibration | Whether a model's reported confidence score (e.g. 0.9) actually matches how often it's correct (e.g. right 90% of the time). |
| Grounding (visual grounding) | The task of connecting a word or phrase to a specific location/object in an image — e.g. correctly drawing a box around "the red block." This is the crux of the model-choice open question: it's distinct from general VQA/captioning ability. |
| RefCOCO / RefCOCO+ / RefCOCOg | Standard benchmark datasets for evaluating visual grounding — how accurately a model can localize an object given a referring text description. Used in this project's research to compare candidate VLMs; notably, OpenBMB has not published RefCOCO-style numbers for any MiniCPM-V version. |
| Moondream2 | A small (~1.9B) VLM from vikhyatk, independent of OpenBMB, with native point/detect grounding modes and official llama.cpp support. The evaluated fallback if MiniCPM-V's grounding proves inadequate. |
| PaliGemma / PaliGemma2 | Google's VLM family, pretrained explicitly with location tokens for grounding — the strongest published grounding evidence of any model considered, but with no working llama.cpp support as of this research. |
