# Technology & Innovation Management Rationale

## The problem this addresses

Unplanned actuator failure is a reactive-maintenance cost: equipment runs
until it breaks, production stops without warning, and repair happens under
time pressure — often at higher cost than a scheduled fix would have been.
This is a well-documented pattern in manufacturing operations: the cost of
an unplanned stoppage is not just the repair itself, but the lost
production time, the disrupted schedule for everything downstream of that
line, and the risk of secondary damage if a degrading component is run to
full failure instead of caught early.

## What this project demonstrates, in management terms

This project is a small-scale proof of concept for shifting a specific
piece of equipment from **preventative** maintenance (fixed schedule,
regardless of actual condition) to **predictive** maintenance (act based on
actual condition signals). The technical pipeline built here — simulate
degradation, monitor a signal, alarm automatically when the signal departs
from its healthy baseline — is the same core loop used in real condition-
monitoring systems, just at a scale that can be verified end-to-end in a
portfolio project.

The value case has three parts:

1. **Earlier warning reduces unplanned downtime.** In this project, the
   detector caught a fault within 0.024 seconds of it occurring, at the
   spec's nominal severity — and even at the mildest severity tested
   (+10%), it caught it within 0.086 seconds. Translated to a real
   maintenance context: the earlier a degrading condition is flagged, the
   more lead time there is to schedule a repair during planned downtime
   instead of reacting to a failure during production.

2. **Severity-aware detection supports prioritization, not just
   detection.** The severity sweep (Section 5 of the technical report)
   shows detection latency and signal magnitude both scale predictably
   with how severe the underlying fault is. In a fleet of multiple
   actuators, this kind of signal could in principle support triaging —
   which unit's fault is more advanced and should be prioritized for
   maintenance first — rather than treating every alarm identically.

3. **Automated detection reduces reliance on manual inspection.** The
   whole point of a statistical threshold learned from a healthy baseline
   is that no one has to be watching a live trace and guessing whether
   something looks "off." The system flags it itself.

## Honest scope of the claim

This is a simulation-based proof of concept, not a production monitoring
system, and the cost-benefit argument above is the *shape* of the value
case a real deployment would need to make with real data, not a validated
number. Specifically:

- Real production data would be needed to establish what a realistic
  "cost of unplanned downtime per hour" actually is for a specific line,
  before any latency number here translates into a dollar figure.
- This model has no sensor noise (see the technical report's limitations
  section), so the very tight detection threshold observed here would not
  directly transfer to a noisy real-world signal without retuning.
- A real deployment decision would also need to weigh false-alarm cost
  (unnecessary inspections triggered by a threshold that's too sensitive)
  against missed-detection cost — a tradeoff this project doesn't attempt
  to quantify.

## Why this belongs in a Technology & Innovation Management portfolio

The technical pipeline (signal processing, fault simulation, statistical
detection) is the *mechanism*. The management rationale above is the
*justification for why the mechanism matters* — connecting a specific
engineering capability (detect a fault X seconds after it starts) to a
specific business lever (reduce unplanned downtime, support maintenance
prioritization). Being able to move between those two levels — build the
technical thing, and explain in plain terms why an organization would
actually want it — is the core skill this portfolio project is meant to
demonstrate.
