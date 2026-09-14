# ROBLOX Luau propNav

A proportional-navigation guidance model ported from
[gedeschaines/propNav](https://github.com/gedeschaines/propNav) (Gary E. Deschaines'
`propNav.py`, a 3-DOF point-mass kinematic missile flyout model).

### Guidance (`PropNav.Law`)

All six laws from `propNav.py`'s `Amslc`, ported function-for-function:

| Law | Source concept |
|---|---|
| `PurePN` | Pure proportional navigation (propNav's default) |
| `TruePN` | True PN, with guidance command preservation (`applyGCP`, ref [3] eq 45) |
| `ZEM` | Zero-effort-miss |
| `APPN` | Augmented Pure PN — feeds forward target acceleration |
| `ATPN` | Augmented True PN |
| `AZEM` | Augmented ZEM |

### Integration and termination

- RK4 (propNav's `RK4_Solver.py`), `Engagement:rk4`.
- Adaptive step size shrinks approaching the endgame — `Engagement:stepSize`, ported
  from propNav's `delT` — down to `TimeStep/50` inside `MinMissDist`.
- Stop condition — `Engagement:shouldStop`, ported from propNav's `Stop` — ends the run
  either at `StopTime` or the instant the range stops closing (sign change on range
  rate), which is the point of closest approach.
- Peak-g telemetry excludes the last 100 m of range: every PN law divides by range
  somewhere, so commanded acceleration diverges in the final milliseconds regardless of
  miss distance, and including it would make every run read as "hit the airframe
  limit."

## Attribution

Guidance math and physical model translated from
[gedeschaines/propNav](https://github.com/gedeschaines/propNav) (`propNav.py`,
Gary E. Deschaines). 
