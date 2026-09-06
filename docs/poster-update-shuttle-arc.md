# Poster update — shuttle shots now fire flat, and aim at the floor

**Read this alongside `poster-software-brief.md`.** That brief is still accurate about the hub
shot; this document covers one change made on 2026-08-31 that the brief predates, lists the exact
lines in the brief that are now stale, and supplies the verified numbers for a new figure.

Everything below was read out of the code or produced by running the actual solver with the
compiled constants. Nothing here is estimated.

---

## 1. What changed, in one paragraph

✍️ **Shooting into the hub and shuttling across the field are different problems, and the robot now
solves them differently. A hub shot has to arrive *descending* so the ball drops into the goal
instead of skimming the rim, so the solver deliberately aims steeper than the most efficient angle.
A shuttle pass is thrown to the carpet, where arriving steeply buys nothing — so the shuttle path
pins the hood at its flattest setting and aims at the floor instead of at goal height. The flatter
arc needs less flywheel speed at the same range, which extends the robot's usable shuttle distance
by about a meter and a half, and it spends less time in the air.**

## 2. What changed, in the code

Three edits, all small:

| File | Change |
|---|---|
| `AimSolver.java` | New `ArcPolicy` enum: `DESCENT_MARGIN` (the existing behavior) and `FLATTEST`. Under `FLATTEST` the solver skips the minimum-energy + descent-margin calculation entirely and pins the launch angle to the flattest the hood can reach. |
| `ShooterSubsystem.java` | New `calculatePhysicsShuttleActions()` alongside the existing `calculatePhysicsShooterActions()`. Same solver, same corrections, different arc policy. |
| `Constants.java` | Shuttle target Z changed from AprilTag height (1.19 m) to `0.0` — the floor. The aim point also moved 1 m toward midfield, and the flywheel ceiling went from 70 to 80 RPS. See section 6. |
| `ShuttleMode.java` | Calls the new shuttle method. |

One smaller correctness fix rode along: the `rackClamped` flag is now also raised when the
shoot-on-the-move and feeder corrections push the *final* hood command outside its travel. Before,
only the pre-correction angle was checked, so the mechanism could silently clamp a command without
the flag noticing. This matters more now, because a pinned-flat shot is already sitting on the
hood's high stop — driving away from the target is what pushes it past.

**Why the target height mattered.** The solver takes a real 3-D target and solves for where the
ball actually comes down. The shuttle target had been reusing the AprilTag's height, which told the
solver to land the pass 1.19 m in the air. A ball aimed to be 1.19 m up at the target point is
still descending past that height on its way to the carpet, so every pass landed long of where it
was aimed. Aiming at z = 0 is what makes the ground the ball's actual landing point.

## 3. Lines in `poster-software-brief.md` that are now stale

| Brief line | Currently says | Fix |
|---|---|---|
| ~150–151 | "the physics solver (`calculatePhysicsShooterActions`, used by the primary shooting **and shuttling** modes)" | Shuttling now uses `calculatePhysicsShuttleActions`. Both are the same solver; only the arc policy differs. Still one physics solver, not two. |
| ~130–131 | "`rackClamped` (physics wanted an angle the hood can't reach)" | Still true, and now also catches the case where motion compensation pushes the command out of range. Optional to mention; the one-line description is not wrong. |
| ~46–57 (Step 1) | Describes the descent margin as *the* way the launch angle is picked | Add one sentence: that's the hub policy; the shuttle policy pins flat. This is the single best addition — it turns one rule into a design decision with a stated reason. |
| ~350–351 (figure list) | Figure 1 is "Ballistic arc diagram… annotate θ_min, the +12° descent margin" | Upgrade this figure to show **both** arcs. See section 4. |
| ~380–381 (key numbers) | "Hood (rack) angle range 15°–42° → launch angle 48°–75°" | Still correct. Add a row: "Shuttle hood angle: pinned at 42° (48° launch)". |

Everything else in the brief — the math, the vector-subtraction explanation, the tuning section,
auto-unjam, localization — is unaffected.

## 4. The rack-angle figure (this is the thing worth drawing)

The ask is a visual of *what the hood angle is, and where*. There are two good forms; the first is
the poster figure, the second is a supporting strip if there's room.

### 4a. Two-arc side view (replaces figure 1 in the brief's figure list)

Side elevation, robot at left, drawn to scale. One diagram, two trajectories from the same launch
point:

- **Launch point** at 0.4826 m above the carpet. Mark it.
- **Hub arc** (label: *goal shot — 61° launch, hood 29°*): from a 8 m stand-off, arcing up and
  arriving at the goal **descending**, entering at z = 1.574 m. Annotate the descent margin: draw
  the minimum-energy angle as a faint dashed ray and the actual launch as a solid one, with the
  12° wedge between them shaded. This is the annotation the brief already asked for — keep it.
- **Shuttle arc** (label: *shuttle pass — 48° launch, hood pinned at 42°*): from the same launch
  point, a visibly flatter and longer arc landing **on the carpet** at ~10 m. Ending on the floor
  rather than in a goal is the whole point of the second arc — make the landing point obviously at
  z = 0.
- **Hood range gauge**, inset: a quarter-circle showing the hood's full 15°–42° travel, with the
  two shots marked on it and the note `launch = 90° − hood`. This is what makes "hood angle 42°"
  legible to someone who assumes bigger angle means higher shot — it does not; 42° is the *flat*
  end.

The counterintuitive mapping (`launch = 90° − hood`) is the thing most readers will get backwards.
Call it out in the caption, not just the gauge.

### 4b. Hood angle vs. distance (small line chart, optional)

Two lines over 2–14 m: hub hood angle rising and flattening out around 30°, shuttle hood angle a
flat line at 42°. It makes the "one is solved, one is pinned" contrast instantly readable. Data in
section 5.

## 5. Verified numbers for the figures

Produced by running `AimSolver` with the compiled constants (real gravity 9.80665 m/s², drag
compensation 0.35 m/s per meter, flywheel RPS offset 2, robot stationary, launch height 0.4826 m).
Distances are horizontal ground distance from the launch point.

**Hub shot** — target z = 1.574 m, `DESCENT_MARGIN` policy:

| Distance | Hood | Launch angle | Launch speed | Flywheel | Flight time |
|---|---|---|---|---|---|
| 2 m | 18.7° | 71.3° | 6.99 m/s | 31.2 RPS | 0.89 s |
| 4 m | 25.4° | 64.6° | 9.03 m/s | 39.7 RPS | 1.03 s |
| 6 m | 27.9° | 62.2° | 10.98 m/s | 47.9 RPS | 1.17 s |
| 8 m | 29.1° | 60.9° | 12.79 m/s | 55.4 RPS | 1.29 s |
| 10 m | 29.9° | 60.1° | 14.50 m/s | 62.6 RPS | 1.38 s |

**Shuttle pass** — target z = 0 (carpet), `FLATTEST` policy:

| Distance | Hood | Launch angle | Launch speed | Flywheel | Flight time |
|---|---|---|---|---|---|
| 6 m | 42.0° | 48.0° | 9.53 m/s | 41.8 RPS | 0.94 s |
| 8 m | 42.0° | 48.0° | 11.45 m/s | 49.8 RPS | 1.04 s |
| 10 m | 42.0° | 48.0° | 13.22 m/s | 57.2 RPS | 1.13 s |
| 12 m | 42.0° | 48.0° | 14.89 m/s | 64.2 RPS | 1.20 s |

**The before/after comparison**, if you want a single number for the poster — same 12 m pass:

| | Old (descent margin, aimed at tag height) | New (flattest, aimed at the floor) |
|---|---|---|
| Hood | 31.3° | 42.0° |
| Launch angle | 58.7° | 48.0° |
| Flywheel | 68.5 RPS | 64.2 RPS |
| Flight time | 1.45 s | 1.20 s |

✍️ **A 12-meter shuttle pass now takes 64 flywheel RPS instead of 68, and spends 1.20 seconds in
the air instead of 1.45.** At the old 70 RPS flywheel cap, that alone moved the longest pass the
robot could physically make from about 12.4 m to about 13.7 m.

## 6. Follow-on change — see update #2

A second change made the same day moved the shuttle aim point and raised the flywheel ceiling from
70 to 80 RPS. It is documented in **`poster-update-shuttle-range.md`**, which supersedes the 70 RPS
figure wherever it appears. Read that one second; it assumes this one.

## 7. Accuracy guardrails (add these to the brief's existing list)

- **Do not** say the shuttle change was validated on the real robot. It is verified by unit test —
  the solver's own trajectory, integrated forward, lands on the target — and the numbers above come
  from running the real solver. It has not been shot on a field as of 2026-08-31.
- **Do not** say the robot "chooses" or "learns" the arc per shot. It does not; the arc policy is
  fixed per mode — hub mode always uses the descent margin, shuttle mode always pins flat.
- **Do not** call 42° the "high" or "raised" hood position in prose without explaining the
  inversion. 42° is the hood's maximum angle *and* its flattest shot. Written carelessly this reads
  as the opposite of what happens.
- The ~12.4 m → ~13.7 m range figure is a **no-drag-model** result with the empirical drag
  compensation applied, at the current calibration. It is a fair comparison between the two
  policies, not a measured maximum range. Phrase it as "about a meter and a half further," not as a
  spec.
