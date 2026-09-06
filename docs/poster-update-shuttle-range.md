# Poster update #2 — shuttling from inside the opposing alliance zone

**Reading order: `poster-software-brief.md`, then `poster-update-shuttle-arc.md`, then this.** This
document assumes update #1 (shuttle passes fire flat and aim at the carpet) and covers a second
change made the same day, 2026-08-31. Where the two disagree on a number, this one wins.

Every number below was produced by running the actual solver with the compiled constants, sweeping
the robot's position along the full length of the opposing alliance zone's back wall. Nothing is
estimated.

---

## 1. What changed, in one paragraph

✍️ **We wanted the robot to be able to pass fuel home while it's still collecting it deep in the
other alliance's zone — the longest shot it is ever asked to make. Two changes got it there. The
aim point moved a meter toward midfield, which takes a meter off every pass. Then, rather than
guessing at a flywheel speed limit, we solved for the shot the robot would actually have to make
from the worst position on the field — pressed against the far wall of the opposing zone, driving
sideways at full speed — and raised the ceiling to cover it with margin. The answer was 69 flywheel
rotations per second; we set the limit to 80.**

## 2. What changed, in the code

| File | Change |
|---|---|
| `Constants.java` | Shuttle aim point moved 1 m toward midfield. Blue goes from `TAG_26.x − 0.25` (x = 3.77 m) to `TAG_26.x + 0.75` (x = 4.77 m); red mirrors it. |
| `Constants.java` | `FLYWHEEL_MAX_REV_PER_SEC` 70 → 80. |

The sideways offset in `ShuttleMode` is unchanged: the pass still lands 2.5 m to whichever side of
the field the robot is already on. That offset is what keeps the ball clear of the hub structure —
so moving the aim point level with the hub does not mean landing on it.

## 3. Where the 80 came from

This is the part worth putting on the poster, because it is a design decision with a derivation
rather than a number someone picked.

The worst shot the robot can be asked to take is from the far side of the opposing alliance zone.
So that case was solved directly: turret ~0.46 m (1.5 ft) off their back wall, swept along the
entire width of that wall, at a range of drive speeds.

| Worst case along the opposing back wall | Old aim point | New aim point |
|---|---|---|
| Shot distance | 12.56 m | **11.58 m** |
| Stationary | 64.9 RPS | **61.6 RPS** |
| Driving at 2 m/s | 66.7 RPS | 63.6 RPS |
| Driving at 3 m/s | 68.0 RPS | 65.0 RPS |
| Driving at 4 m/s | 69.6 RPS | 66.6 RPS |
| Driving at full 5.44 m/s | 72.2 RPS | **69.3 RPS** |

Two things fall out of this table, and both are better poster material than the ceiling itself:

✍️ **Driving costs speed.** A full-speed sideways sprint adds about **8 RPS** to the same shot taken
standing still. The shooter has to cancel the robot's own sideways velocity *and* reach the target,
and both come out of the same flywheel. This is the shoot-on-the-move vector diagram (figure 2 in
the brief's figure list) expressed as a price.

✍️ **Moving the aim point a meter bought back most of that.** Shortening every pass by 1 m saves
about 3 RPS — roughly a third of what full-speed driving costs. The new aim point standing still
(61.6 RPS) is cheaper than the old aim point standing still (64.9 RPS) by more than the whole
change is worth in tuning effort.

**80 is not the requirement; 69.3 is.** The margin covers two things: the drag compensation term
(`SPEED_PER_METER`, still a first guess at 0.35 m/s per meter of range) rising when it gets tuned
against real shots, and staying at 80% of a Kraken X60's 100 RPS free speed — the flywheel is
direct-drive, so motor RPS is flywheel RPS.

## 4. The honest limitation — the hood, not the flywheel, is now the limit

This belongs on the poster if the poster claims shoot-on-the-move at all. It is the kind of detail
a technical reviewer will respect and an overclaim they will catch.

A shuttle pass is already pinned at the hood's flat stop of 42° (update #1). Cancelling sideways
motion is exactly what asks the solver for a *flatter* shot — and there is nothing flatter left.

- Above roughly **1.8 m/s** of lateral speed, the solution wants a hood angle past 42°.
- At full 5.44 m/s it wants **47.9°**, nearly 6° beyond the mechanism's travel.
- The hood clamps at 42° and the pass lands short.

The robot does not hide this. The solver raises its `rackClamped` flag whenever the motion
corrections push the final hood command outside its travel, and that flag is published to the
dashboard live. So the failure is visible rather than mysterious.

✍️ **Suggested framing:** *"Raising the flywheel limit moved the bottleneck rather than removing
it. The shooter can now supply enough speed to shuttle from anywhere on the field — but past about
1.8 m/s of sideways motion the hood physically runs out of travel before the math does, and the
robot says so on the dashboard instead of quietly missing."*

## 5. Figure suggestion — overhead field map

Update #1 asks for a **side view** (arc shape, hood angle). This change wants the complementary
**overhead view**, and the two together make a strong pair: one shows *what the shot looks like*,
the other shows *where on the field it happens*.

Draw the field to scale, 16.54 × 8.07 m:

- **Robot** at the far side of the opposing alliance zone, turret 0.46 m off their back wall.
  Label it *"collecting in their zone."*
- **The pass**, an arrow spanning **11.58 m** across the field to the aim point.
- **Aim point** at x = 4.77 m, offset **2.5 m to the side of the hub** — draw the hub footprint so
  it is obvious the ball lands *beside* it, on open carpet, not on it. Show the mirrored aim point
  faintly on the other side and note that the robot picks whichever side it is already on.
- **The 1 m shift**, as a short annotated arrow from the old aim point (x = 3.77) to the new one
  (x = 4.77), labeled *"−1 m of range needed."*
- **A small inset bar**: 61.6 RPS standing still vs 69.3 RPS at full speed vs the 80 RPS ceiling.
  Three bars makes the headroom argument in one glance.

If there is only room for one new graphic in the software column, this is a better use of space
than a second equation.

## 6. Brief lines this change makes stale

| Brief line | Currently says | Fix |
|---|---|---|
| ~384 (key numbers) | "Flywheel max commanded speed \| 70 rotations/sec" | **80 rotations/sec.** |
| ~131 (Step 6) | "`speedClamped` (wanted more than 70 RPS)" | 80 RPS. |
| ~380–381 (key numbers) | — | Add a row: "Longest shuttle pass (opposing back wall) \| 11.6 m, 61.6 RPS stationary". |

Nothing else moves. The math, the vector-subtraction section, the tuning workflow, auto-unjam, and
localization are all untouched by this change.

## 7. Accuracy guardrails (in addition to update #1's list)

- **Do not** say the robot "can shuttle from anywhere on the field" without the hood caveat from
  section 4. Standing still that is true; driving hard sideways it is not.
- **Do not** present 80 RPS as a measured requirement or as the flywheel's maximum capability. It
  is a commanded ceiling with deliberate margin; the measured worst case is 69.3 RPS and the
  motor's free speed is 100 RPS.
- **Do not** say the ceiling was raised "because the shots were falling short." It was raised
  because the worst case was solved for and found to exceed the old limit — the change was
  predictive, not reactive. That distinction is the whole point of having a physics solver, and it
  is worth making explicitly.
- **Do not** claim any of this was validated on a real field. As of 2026-08-31 it is verified by
  unit test and by running the solver; no shuttle pass has been thrown from the opposing zone.
- The 1.8 m/s hood-clamp threshold is a solver result at the current calibration, not a measured
  handling limit. Phrase it as "around 1.8 m/s," not as a spec.
