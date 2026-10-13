# Troubleshooting

Symptoms, root causes, and recipes for the failure modes this controller
hits in practice. Read [controller.md](controller.md) for the underlying
math and [interface.md](interface.md) for the debug topic layout
referenced throughout.

## Signature 1: vehicle freezes mid-maneuver, `/debug/data` stops updating

**Logs** (from the motion_control terminal):
```
[motion_control] [ERROR] cannot find closest waypoint
[motion_control] [ERROR] trajectory_tracker run failed
```
emitted continuously at the 2 s throttle interval.

**`/debug/data`** values stop changing — the last successful values
stay frozen because the failure path doesn't update `ey_debug` etc.

**`/control/command/control_cmd`**: lateral and longitudinal fields are
all zero; `is_defined_rear_steering = false`.

**`/pacmod/motor_vel_cmd`**: `command = 0.0`.

### Root cause

`findNearestIndex` couldn't find any trajectory point within 2 m of the
filtered ego pose. The tracker's `run()` returns false, and
`MotionController::onTimer()` publishes zeros and the throttled error
message.

The 2 m budget is taken up by some combination of:

- **Lateral tracking drift.** The ego has wandered laterally from the
  trajectory faster than the lateral feedback can pull it back. Check
  `/debug/data[0]` (`ey`) on the run leading up to the freeze — if it
  ramped past 1.5–1.8 m before things stopped updating, this is the
  cause. Most common during reverse OBLIQUE crab when the crab
  feedforward sign is wrong (see [controller.md](controller.md)
  "Why `+betar` in reverse") or the lateral gain is too low for the
  scenario.
- **Trajectory truncated behind the ego.** The planner has cropped the
  trajectory to a window ahead of where it last sampled the ego pose,
  and the ego has since moved past the start of that window. The first
  point of the trajectory is now ~5 m behind the ego while subsequent
  points fan ahead. Inspect:
  ```bash
  ros2 topic echo /planning/scenario_planning/trajectory --field points[0].pose.position
  ros2 topic echo /localization/kinematic_state --field pose.pose.position
  ```
  If they differ by >5 m, the planner's deviation gate is the suspect,
  not motion_control.

### Recipes

**Confirm the abort happened mid-maneuver, not at startup:**
```bash
ros2 bag record /debug/data /control/trajectory_follower/control_cmd \
                /localization/kinematic_state /planning/scenario_planning/trajectory \
                /planning/reverse_state /fms/maneuver_type
```
Then post-mortem the bag — the failure is at the timestamp where
`/debug/data` stops changing.

**Widen the search radius (temporary debug, not a fix):** the radius
is hard-coded to 2.0 m at `trajectory_tracker.cpp:85`. Bumping it to
4.0 (or 5.0) lets the controller keep running while you isolate the
upstream cause — but if the ego is 4 m off the trajectory you have a
real tracking failure, not a tolerance problem.

**Rule out the unit bug:** the `findNearestIndex` comparison was
previously squared-vs-linear, shrinking 2.0 m to ~1.41 m. That's
fixed in the current code (the variable is `sq_dist` compared against
`max_sq_dist = max_dist * max_dist`). If you've been pulling in old
patches, double-check this is on.

## Signature 2: vehicle drifts off the planned crab path

**Symptom**: during an OBLIQUE / Q_TURN maneuver the body yaw stays
constant (or rotates correctly for Q_TURN), but the ego position
drifts in the wrong direction relative to the planned tangent.
Typical end state: vehicle ends up several metres from the planned
LC_END, then trips Signature 1 above.

### Root cause: crab feedforward sign

If the four-axle commands look like they're trying to **counter** the
planned crab rather than execute it, the sign of the `betar` term in
the reverse branch of `calcSteerCmd` is the first suspect.

Diagnose:

```bash
# during the LC span, watch fw and rw — they should be the SAME sign
# and similar magnitude for a true 4WS crab walk
ros2 topic echo /control/command/control_cmd \
  --field lateral.steering_tire_angle,lateral.rear_steering_tire_angle
```

Compare against the planned crab. The planned crab angle is roughly:

```
crab_angle = body_yaw_at_point − path_tangent_at_point
```

The wheels should be at that angle (both front and rear). If they're
**at the opposite sign**, you're seeing the sign-flip bug — verify
that `calcSteerCmd`'s reverse branch reads `+ betar`, not `− betar`.
See the inline comment in `trajectory_tracker.cpp` and the
"Why `+betar` in reverse" section in [controller.md](controller.md).

### Root cause: gain magnitude

If the wheels are the right **sign** but smaller than the planned
crab angle, the lateral feedback is fighting the feedforward —
either `trk_ky_reverse` is too high (opposing crab feedforward
aggressively) or `delta_limit` is clipping the base steer.

Diagnose:

- `/debug/data[1]` (pre-clamp `delta`) vs `/debug/data[2]` (post-clamp).
  If they differ during the span, `delta_limit` is the binding
  constraint — raise it or accept a slower crab.
- `/debug/data[0]` (`ey`) sign during the maneuver. The feedforward
  term `± betar` and the feedback term `−ky/vx · ey` should agree —
  if they don't, lower `trk_ky_reverse` until the feedforward
  dominates the first half of the maneuver and feedback only
  cleans up the tail.

## Signature 3: oscillatory steering, vehicle "saws" along the path

**Symptom**: lateral error oscillates rhythmically (visible on
`/debug/data[0]`), accompanied by high-frequency oscillation on
the steer commands (`axis1..4` flipping sign at 5–10 Hz).

### Likely cause

Lateral gain too high relative to closed-loop response. With the
controller running at 50 Hz, the ego inertia and pacmod actuator lag
limit the practical bandwidth to a few Hz.

Diagnose: lower `trk_ky` / `trk_ky_reverse` to half the current
value; if the oscillation magnitude scales linearly with the gain,
gain is the cause. Final tuning is at the smallest gain that gives
acceptable steady-state error.

If oscillation persists at very low gain, suspect the
**third-order tracking differentiator** on `ey` (ω hard-coded to 5
in the tracker constructor). Lowering that ω would smooth `ey_dot`
at the cost of phase lag, but `kvy` is currently 0 by default, so
this is rarely the proximate cause.

## Signature 4: gain explodes / wheels saturate at startup

**Symptom**: immediately after engage, the four-axle commands jump
to ±`max_steering_angle` and stay clamped. Vehicle either doesn't
move or moves in a wild arc.

### Root cause

`vx_safe` is the SPEED divisor in the proportional gains. At
standstill the unfiltered `vx` is 0, so `vx_safe = max(|vx|, vx_min)`
clamps to `vx_min` (default 0.1–0.2 m/s). At the typical cruise gain
this gives `ky / vx_min = ky · 5..10`, which combined with the
startup `ey` (the gap between ego pose and the trajectory start)
produces a saturated delta.

Mitigations:

- Engage when the ego is **already on** the trajectory — `ey` near
  zero means even a low `vx_min` doesn't explode the term.
- If startup `ey` is unavoidable (e.g. the trajectory starts a few
  metres ahead of the ego), increase `trk_vxmin` to 0.5 or 1.0
  during commissioning so the gain isn't divided by a near-zero
  number. Restore it once tuning is done.
- Verify `param.is_forward` matches the current gear at engage —
  using the reverse gains for forward motion (or vice versa) at
  startup produces this same signature.

## Signature 5: high-level Control has correct sign but pacmod sends zeros

**Symptom**: `/control/command/control_cmd` carries non-zero
`steering_tire_angle`, `rear_steering_tire_angle`, and `velocity`,
but `/pacmod/steering_cmd` and `/pacmod/motor_vel_cmd` are zero;
vehicle doesn't move.

### Root cause

`upper_test.enable == false`. The pacmod direct-command publish is
gated on this flag (see `motion_controller.cpp:122..152`), while the
high-level `Control` to `vehicle_cmd_gate` ignores it. The flag is
set via the `/srv_upper_test` service — if the operator app hasn't
called the service yet, the flag is false and pacmod sees zeros.

Diagnose:

```bash
ros2 service call /srv_upper_test pacmod_msgs/srv/UpperTest "{enable: true}"
```

(or use the operator app's enable button). Then watch
`/pacmod/steering_cmd` — it should carry the four axis commands
within a cycle.

## Signature 6: reverse maneuver starts forward, then reverses

**Symptom**: the system commands forward motion briefly at engage,
then switches to reverse a few cycles later.

### Root cause

`/planning/reverse_state` is **transient_local**, but if the
publisher (`fms_path_converter`) hasn't yet processed the first
trajectory at the moment motion_control starts, no message has
been latched and the controller starts with `is_forward = true`
(the default).

This is mostly cosmetic — within ~100 ms the latched value arrives
and `is_forward` flips. If it bothers downstream behavior
(e.g. wrong gear command at startup), confirm the FMS chain is
fully up before engaging Auto. Alternatively, change the launch's
`is_forward` default to `false` if the application is always
reverse-first.

## General debug starter pack

```bash
# 1) Watch the controller state
ros2 topic echo /debug/data --field data

# 2) Watch the high-level control output
ros2 topic echo /control/command/control_cmd \
  --field lateral.steering_tire_angle,lateral.rear_steering_tire_angle,\
longitudinal.velocity

# 3) Confirm the planner is feeding a sane trajectory
ros2 topic echo /planning/scenario_planning/trajectory \
  --field points[0].pose.position,points[0].longitudinal_velocity_mps

# 4) Confirm reverse_state is what you expect
ros2 topic echo /planning/reverse_state \
  --qos-durability transient_local --qos-reliability reliable

# 5) Record everything for post-mortem
ros2 bag record /debug/data \
  /control/command/control_cmd \
  /control/trajectory_follower/control_cmd \
  /localization/kinematic_state \
  /planning/scenario_planning/trajectory \
  /planning/reverse_state \
  /fms/maneuver_type \
  /pacmod/steering_cmd /pacmod/motor_vel_cmd /pacmod/shift_cmd \
  /vehicle/status/steering_status /vehicle/status/velocity_status
```

When opening a ticket, attach the bag and the rough wall-clock
timestamp of the misbehavior — the bag's `t=0` is bag-start, not
engage, so the timestamp anchors the analysis.
