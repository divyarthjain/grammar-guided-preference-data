# What Is This Project? (Explained Very Simply)

## The problem, in one sentence

We want a robot to get better at deciding what to do when it looks at something — without a human sitting there for months telling it "yes, good" or "no, bad" for every single thing it tries.

## An analogy first

Imagine you're teaching a kid to shoot a basketball. The normal way: the kid shoots, you watch, you say "good shot" or "bad shot," and they slowly learn. That works, but it needs YOU there the whole time, watching every single shot. That doesn't scale — you can't watch a million shots.

Now imagine instead: the kid shoots, and the hoop itself tells them whether it went in. No human needed. The kid can practice all day and night, and you only need to check in occasionally to see how they're improving.

That's the whole idea of this project, except the "kid" is an AI, the "shot" is a robot deciding what to do with something it sees, and the "hoop" is physics itself.

## The big idea, step by step

1. **A robot has a camera.** It looks at a scene — say, some objects on a table.
2. **An AI looks at the picture and guesses a few things it could do.** For example: "that's a red block, I should grab it" or "that's the table edge, I should avoid it." It makes several different guesses per picture, not just one.
3. **Instead of asking a human "is this guess good?", we ask physics.** We literally try the move — can the robot's arm/leg actually reach that spot? Does it stay balanced? Does it break anything? Physics doesn't lie and doesn't need paying or sleeping.
4. **If the move actually works, we mark it "good." If it fails, we mark it "bad."** No human ever looked at it. The robot's own body did the grading.
5. **We save every good-vs-bad pair as a lesson.** Over time, thousands of these pile up automatically, just from the robot operating normally.
6. **Every so often, the AI studies all its saved lessons and updates itself** to be a little better at guessing next time. Then it goes back to step 2, slightly smarter than before. Repeat forever.

That's it. That's the whole trick: **let the robot's own physical success or failure be the teacher, instead of a human.**

## "Okay, but why does that even matter?"

Because normally, teaching an AI like this needs a giant pile of pictures that a human has already labeled ("this is a cup," "this is graspable," etc.). That takes forever and costs a lot, and nobody has enough labeled pictures of *this specific robot in this specific room*. This project skips that entirely — the data invents itself while the robot works.

## "This sounds risky to try on a real robot — what if it breaks something?"

Great instinct — that's exactly why we don't try new, untested ideas on the real robot first. Instead, we built a **video-game version of the robot** — a realistic physics simulation (using free, open-source software called MuJoCo) that behaves like the real thing without any real hardware involved. We can throw thousands of bad ideas at the simulated robot, watch it (sometimes literally) fall over or fail, and none of it costs anything or breaks anything real.

### How the simulation specifically works

See `diagrams/mujoco-simulation.excalidraw` for the picture version of this:

1. A pretend object shows up (we made up a fake "red block" and a fake "table edge" for testing, since we don't have a real camera hooked up yet).
2. The simulated robot's leg tries to physically reach toward it, using real physics math (this is called "inverse kinematics" — a fancy way of saying "figure out how to bend the joints to reach a target point").
3. Three quick reality checks happen: *Can it actually reach that far? Are the joints bending within their real limits (not snapping backward)? Does the robot stay standing up, or does it tip over?*
4. Based on those checks, it's marked GOOD or BAD.
5. That gets written down as one more lesson — and this whole thing is genuinely watchable: you can open a window and see the little robot leg move around in real time while this happens, roughly twice a second.

## "How do I actually see it?"

Run one command (`uv run mjpython -m simulation.mujoco_judge` from the `simulation/` folder) and a window pops up showing the simulated hexapod robot. Its leg visibly moves toward each fake target, over and over, forever, while a file on disk fills up with the "lessons" it's generating. Close the window to stop it.

## Quick glossary (in plain words)

| Term | What it actually means |
|---|---|
| **VLM** (Vision-Language Model) | The "AI" that looks at a picture and describes what it sees, in words. |
| **Hexapod** | A robot with 6 legs, like a big mechanical spider/insect. |
| **MuJoCo** | Free software that fakes real-world physics — gravity, joints, collisions — so we can test things safely on a computer before touching a real robot. |
| **IK (Inverse Kinematics)** | The math for "given where I want my hand/foot to end up, how much should I bend each joint to get there?" |
| **Judge / Physical Judge** | The part of our system that decides GOOD or BAD, using physics instead of a human. |
| **Preference pair** | One saved lesson: "this move was good, this other move was bad, for the same picture." |
| **Training / Fine-tuning / "the anneal"** | The occasional process where the AI actually studies its pile of saved lessons and updates itself to be smarter. |
| **Checkpoint** | A saved snapshot of "the AI as it currently is," taken every so often so we can compare how much smarter it got over time. |

## The one-sentence summary you can say out loud to anyone

> "We built a robot that grades its own homework using physics instead of a person, and we tested the whole idea safely in a simulation before ever touching a real robot."
