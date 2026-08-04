# Interview Explainer: Predictive Maintenance & Anomaly Detection Project

This document explains the entire project from scratch — as if you're
telling someone who has never seen it before. Read it top to bottom once,
then use the "likely questions" section at the end to rehearse.

---

## 1. The one-sentence version

*"I built a simulated electric actuator in MATLAB/Simulink, made it develop
two realistic mechanical/electrical faults over time, and then wrote code
that automatically detects those faults from the current signal alone —
the same idea real factories use to catch equipment failures before they
cause downtime."*

---

## 2. Why this problem matters (the business case)

In factories, actuators (motors that move things — arms, valves, conveyor
parts) wear out gradually before they fail completely. Two ways to handle
this:

- **Reactive maintenance:** wait until it breaks, then fix it. Expensive —
  unplanned downtime stops production.
- **Predictive maintenance:** watch signals from the equipment (current,
  vibration, temperature) and catch the *early signs* of wear before
  complete failure. Cheaper — you schedule the fix instead of being
  surprised by it.

This project is a small, self-contained demonstration of the predictive
maintenance idea: simulate the equipment, simulate it degrading, and prove
you can catch the degradation automatically and quickly.

---

## 3. What "the actuator" actually is

An actuator here means a small electric motor with a load attached — think
of a motor turning a shaft that has some resistance to spinning it (bearing
friction) plus something it has to push against (load torque). It has two
things going on at once, physically:

- **Electrical side:** voltage goes in, current flows, that current
  produces torque
- **Mechanical side:** torque spins the shaft, but friction and load fight
  against that spin

These two sides are coupled: more current → more torque → faster spin. But
also, faster spin → more "back-EMF" (a voltage the spinning motor generates
that opposes the current) → limits how much current keeps flowing. That
electrical/mechanical feedback loop is what makes a motor a *dynamic
system*, not just a simple on/off switch.

### The two equations that describe it

You don't need to derive these in an interview, but you should be able to
explain what each term means:

**Electrical:** `L * di/dt = Vin - R*i - Ke*omega`

> "The rate of change of current depends on the voltage applied, minus the
> voltage lost to resistance, minus the back-EMF voltage from the motor
> spinning."

**Mechanical:** `J * domega/dt = Kt*i - B*omega - Tload`

> "The rate of change of speed depends on the torque produced by the
> current, minus friction losses (which get worse the faster it spins),
> minus whatever load it's fighting against."

Where: `L`=inductance, `R`=resistance, `Ke`=back-EMF constant, `J`=inertia,
`Kt`=torque constant, `B`=friction/damping, `Tload`=load torque, `i`=current,
`omega`=speed.

---

## 4. Why I didn't use Simscape (the physical-modeling toolbox)

MATLAB has a toolbox called Simscape that lets you build motors out of
realistic physical component blocks (resistors, inductors, mechanical
rotational elements) that behave like real circuits. The original spec
suggested using it.

**I chose not to**, and built the two equations above directly instead,
using ordinary Simulink math blocks (Gain, Sum, Integrator). Why:

- I was building the entire model via a MATLAB *script*
  (`add_block`/`add_line`) rather than dragging blocks in the GUI, because
  manual GUI wiring without live visual feedback is unreliable — one
  misclick creates a phantom connection you don't notice until the
  simulation breaks.
- Simscape blocks are much harder to wire correctly via script — they use
  special "physical signal" ports and need solver configuration that
  ordinary Simulink blocks don't.
- The two faults I needed (a friction increase, a resistance increase) are
  just numbers changing in the equations above — you don't need a full
  physical circuit simulator to represent "this number goes up at this
  time."

**If asked "isn't that less realistic?"** — the honest answer: yes,
slightly, in the sense that Simscape would also model things like magnetic
saturation. But the equations I used capture the actual dynamic behavior
that matters for this project (how current and speed respond to a
parameter change), and prioritizing a build method I could verify reliably
was the right tradeoff for a project built entirely through
script-generated models.

---

## 5. The two faults, explained physically

### Fault A — Bearing wear (mechanical friction), injected at t = 8 seconds

As a bearing wears out, it doesn't spin as freely — there's more friction.
In the model, this means the friction coefficient `B` jumps from its
normal value to +150% of normal, instantly, at t=8s (representing "the
wear has now progressed to a noticeable point," not a gradual wear-in).

**What happens physically:** more friction means the motor has to work
harder (draw more current) to keep spinning at the same speed. But it
still can't fully keep up, so speed also drops a bit. In the simulation:
current rose from ~4A to ~6A, and speed dropped from ~420 rad/s to ~380
rad/s — both in the physically correct direction.

### Fault B — Stator thermal aging (resistance increase), injected at t = 14 seconds

As motor windings heat up over time (thermal aging), their electrical
resistance increases. In the model, resistance `R` jumps to +80% of
normal at t=14s.

**What happens physically:** higher resistance means more voltage gets
"wasted" as heat in the windings instead of driving current, so for the
same supply voltage, less current flows. Less current means less torque,
so speed drops too. In the simulation: current dropped from ~6A back to
~4A, and speed dropped further to ~300 rad/s — again, the correct
direction.

**Why inject faults as instant steps instead of gradual ramps?** Simplicity
and clarity for a portfolio-scale demo — it makes the "before/after" signal
change unambiguous, which makes verifying the detection pipeline much
easier. A production system would more likely see gradual degradation, but
the detection principle (watch a statistic, alarm when it crosses a
threshold) works the same either way.

---

## 6. How the model was actually built (the script)

`build_model.m` doesn't drag-and-drop anything — it calls MATLAB functions
that programmatically create blocks and wires:

- `add_block(...)` creates a block (a Gain, a Sum, an Integrator, etc.) at
  a given position
- `add_line(...)` connects one block's output port to another block's
  input port

Two integrators form the heart of the model — one integrates `di/dt` to
produce current, the other integrates `domega/dt` to produce speed — with
gain, product, and sum blocks wired around them implementing the two
equations from Section 3.

The two faults are each built the same pattern: a `Step` block (outputs 0,
then jumps to a new value at a specific time) added onto a `Constant`
block (the nominal value) via an `Add` block, producing a *time-varying*
parameter that feeds into the rest of the model.

---

## 7. The signal processing / detection pipeline, step by step

This lives in `run_predictive_diagnostics.m`. In order:

1. **Run the simulation**, get back the logged current signal.
2. **Resample to a fixed rate (10 kHz).** The simulation solver
   (`ode45`) takes variable-size time steps — smaller steps when things
   change fast, bigger steps when things are calm. That's efficient for
   simulation, but FFT and rolling-window statistics both need evenly
   spaced samples. So the raw signal gets linearly interpolated onto a
   fixed 10,000-samples-per-second grid first.
3. **Compute rolling RMS** (root-mean-square) over a 0.5-second window.
   RMS is a way of measuring the "typical magnitude" of a signal that's
   wiggling around zero or around some average — squaring makes
   everything positive, averaging smooths out noise, square-rooting
   brings it back to the original units (Amps). A rising RMS means "the
   current is running higher than it used to," which is exactly the
   fault signature we're looking for.
4. **Establish a healthy baseline.** Look at the RMS signal during a
   window where nothing has gone wrong yet (t = 2 to 6 seconds — starting
   at 2s instead of 0s to skip the simulation's initial startup
   transient). Compute its mean and standard deviation.
5. **Set a 3-sigma threshold.** This is a standard statistical
   process-control idea: if a signal is behaving normally, it should
   rarely stray more than 3 standard deviations from its mean. So:
   `threshold = healthy_mean + 3.5 * healthy_std`. If the signal crosses
   that line, it's no longer behaving like the healthy baseline —
   flag it as an anomaly.
6. **Find the first crossing** after the baseline period — that's your
   alarm time. Report how long after the actual fault it took to notice
   (the "detection latency").
7. **FFT comparison** (extra diagnostic view, not part of the alarm
   logic): compare the frequency content of the current signal in a
   healthy window vs. each faulted window. This is meant to help visualize
   how the fault reshapes the signal, not to trigger the alarm itself.

---

## 8. The two real bugs — this is the part interviewers care about most

Anyone can show a working result. Being able to explain *what broke* and
*how you found and fixed it* demonstrates actual understanding — this is
worth memorizing well.

### Bug 1: Variables vanished between scripts

**What happened:** `build_model.m` created the Simulink model and also
defined a bunch of variables (the motor parameters — resistance,
inductance, etc.) directly in MATLAB's "base workspace" (the pool of
variables you can see when you type `whos`). The Simulink blocks refer to
these variables by name internally. Then, when I ran the *next* script
(`run_predictive_diagnostics.m`), it started with a `clear` command —
standard practice to start clean — which wiped out those variables. So
when the simulation tried to run, every block that needed a parameter
value failed, because the variable it needed no longer existed.

**The fix:** instead of relying on variables surviving in the workspace
between separate script runs, `build_model.m` now saves them to a `.mat`
file. Every script that needs to run the simulation loads that file
explicitly first. Now nothing depends on what state the workspace happened
to be in before.

**Why this is a good bug to talk about:** it's a classic "state
management" bug — the kind that happens all the time in real software when
two pieces of code implicitly depend on shared state instead of explicitly
passing what they need to each other.

### Bug 2: The alarm fired before the fault happened

**What happened:** The very first working version of the detector reported
an anomaly at t = 7.765 seconds — but the fault wasn't injected until
t = 8.0 seconds. That's physically impossible for something that's
supposed to detect a fault *after* it occurs.

**Root cause, in two parts:**
1. My first baseline window included the simulation's startup transient
   (current spikes very high in the first fraction of a second before
   settling down) — this made the "normal" variation look much bigger than
   it really was, so the very first threshold was way too loose and never
   caught anything at all. I fixed that by starting the baseline window a
   bit later (t = 2s instead of t = 0s), after the transient had settled.
2. After fixing that, the threshold got tighter — but then it triggered
   *early*. The reason: MATLAB's rolling-average function (`movmean`) by
   default centers its window around each point — meaning the "current"
   value at time t is actually an average that includes some samples
   *from the future* (up to half a window-width ahead). With a 0.5-second
   window, that's up to 0.25 seconds of "looking ahead" — which is exactly
   why the alarm at 7.765s makes sense: 8.0 - 0.25 ≈ 7.75.

**The fix:** change the rolling window to be *causal* — only look at
samples up to and including the current time, never after it. In MATLAB
this is done with `movmean(x, [window_size-1, 0])` instead of
`movmean(x, window_size)`. After that fix, the alarm moved to t = 8.024
seconds — just after the fault, which is the only physically valid
outcome for a real-time detector.

**Why this is a good bug to talk about:** it's subtle — the code ran
without any error message both before and after the fix. The only way to
catch it was checking whether the *result made physical sense*, not just
whether the code executed. That's an important engineering habit: "it ran"
is not the same as "it's correct."

---

## 9. The severity sweep — why one successful test isn't enough

After fixing both bugs, I had one working example: at the exact spec
severity (+150% friction increase), detection worked with a small,
sensible latency. But one data point could be a coincidence — what if it
only happens to work at that exact severity?

To check, I reran the entire pipeline at 9 different severity levels
(from a mild +10% friction increase up to a severe +300% increase) and
looked for a *consistent trend*, not just "does it still work."

**Result:** as severity increases, detection latency shrinks smoothly
(from 0.086s at the mildest level down to 0.017s at the most severe), and
the peak RMS current after the fault rises smoothly too. Both trends make
physical sense — a bigger fault produces a bigger, more obvious signal
change, so it crosses the threshold faster. Seeing that clean, monotonic
relationship across 9 independent tests is much stronger evidence that the
detector is actually working correctly, rather than getting lucky once.

**Honest limitation to mention if asked:** because this simulation has no
random sensor noise, the "normal" variation in the healthy baseline is
extremely tiny — which makes the statistical threshold very sensitive.
Every single severity level I tested, even the mildest +10%, got detected.
A more realistic version with actual sensor noise added would likely show
a point below which faults are too small to reliably distinguish from
noise — this version doesn't demonstrate that boundary, and I say so
plainly in the write-up rather than hiding it.

---

## 10. What's NOT done (say this proactively, don't wait to be caught)

- The detector only reports the *first* alarm per run. The second fault
  (thermal, at t=14s) does show up in the signal, but the pipeline as
  written doesn't separately flag it as a second event once the first
  alarm has fired.
- No sensor noise is modeled, which — as above — means the severity sweep
  never shows a genuine "too small to detect" case.
- The FFT view is for visualization only; it doesn't feed into the
  detection decision itself.

Interviewers generally respond very well to a candidate who names their
own project's limitations clearly, rather than a candidate who has to be
asked "but what about X?"

---

## 11. Likely interview questions and how to answer them

**"Walk me through your project."**
→ Use the one-sentence version (Section 1), then the pipeline steps
(Section 7), then lead with the bugs (Section 8) — that's the most
technically interesting part.

**"Why didn't you use Simscape since the spec asked for it?"**
→ Section 4's answer, framed as a deliberate build-reliability tradeoff,
not a shortcut.

**"How do you know your detector actually works, and isn't just tuned to
one lucky case?"**
→ Section 9 — the severity sweep and the monotonic trend.

**"What was the hardest bug?"**
→ Bug 2 (Section 8) — emphasize that it produced no error message, and
that you only caught it by questioning whether the *timing* made physical
sense, not just whether the code ran.

**"What would you do differently / what's next?"**
→ Section 10, verbatim.

**"What's a 3-sigma threshold and why 3.5 and not 3?"**
→ It's a statistical process-control idea: values within 3 standard
deviations of the mean are considered "normal" for a Gaussian-like
process; going further out is statistically unlikely under normal
operation. Using 3.5 instead of 3 makes the threshold slightly more
conservative — fewer false alarms — while still keeping it based on the
same idea.

**"Why resample to a fixed sample rate before doing FFT?"**
→ Because the solver (`ode45`) uses variable time steps, and both FFT and
a rolling-window RMS assume evenly spaced samples in time — otherwise the
math (which implicitly assumes a constant time interval between points)
gives meaningless results.
