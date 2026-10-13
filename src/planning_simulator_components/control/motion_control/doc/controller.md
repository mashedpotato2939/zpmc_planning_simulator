# Controller

This document describes the math inside `TrajectoryTracker::run()` — the
single control law that produces the four-wheel-steer angles and the
longitudinal velocity command each cycle. It also covers the
trajectory preprocessing step, the reverse-mode sign convention, and
the crab-feedforward used for OBLIQUE / Q_TURN maneuvers.

Source: `src/control/trajectory_tracker.cpp` and
`include/control/trajectory_tracker.hpp`.

## High-level structure

`run()` is the entry point invoked from `MotionController::onTimer()`.
It executes the following pipeline:

```
                 ┌────────────────────────────────┐
                 │ checkData                      │ → false ⇒ return false
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ traj_preprocess                │
                 │  • cumulative arc length s     │
                 │  • |longitudinal_velocity_mps| │
                 │  • body yaw (forward) OR       │
                 │    motion-direction yaw, π-    │
                 │    flipped (reverse)           │
                 │  • per-point curvature κ       │
                 │  • per-point heading_rate ω    │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ update_differentiator(odom)    │
                 │  • 3rd-order tracking diff on  │
                 │    x, y, yaw → x1 (filtered)   │
                 │    + x2 (velocity)             │
                 │  • build odom_filter           │
                 │    (π-flipped in reverse)      │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ findNearestIndex(2.0, π/4)     │ → empty ⇒ return false
                 │  • read velocity_cmd at        │
                 │    nearest + 10                │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ calc_preview_point(t_pred)     │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ errorCalc(preview_odom)        │ → false ⇒ return false
                 │  • findNearestIndex(4.0, π/4)  │
                 │  • lateral error ey            │
                 │  • yaw error eyaw              │
                 │  • crab feedforward betar      │
                 │    (= tan(body_yaw − tangent)) │
                 │  • curvature feedforward κ     │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ third_order_diff on ey → eẏ    │
                 └─────────────────┬──────────────┘
                                   ▼
                 ┌────────────────────────────────┐
                 │ calcSteerCmd                   │
                 │  • δ  = f(ey, eẏ, betar, vx,…) │
                 │  • δ' = f(eyaw, κ)             │
                 │  • clamp → axis_*_cmd          │
                 │  • flip per-axis if reverse    │
                 └─────────────────┬──────────────┘
                                   ▼
                          cmd populated, return true
```

If any of the three `return false` paths fire, the cycle produces no
command and `MotionController` substitutes zeros. The most operationally
common one is the **first `findNearestIndex` in the main loop** —
covered in [troubleshooting.md](troubleshooting.md).

## Trajectory preprocessing (`traj_preprocess`)

Runs every cycle on the latest trajectory message; cheap because the
trajectory size is bounded (~100–200 points). Three things happen:

1. **Cumulative arc length** `s_array[i]` so curvature and yaw rate can
   use central differences.
2. **Velocity absolute-value-ification.** All `longitudinal_velocity_mps`
   fields are replaced with `|v|`. Reverse direction is encoded in
   `param.is_forward`, not in the sign of the velocity, so downstream
   sign handling stays local to `calcSteerCmd`.
3. **Per-point yaw transform.**
   - **Forward**: leave `pose.orientation` alone — the trajectory
     already encodes the body yaw.
   - **Reverse**: `yaw → yaw + π`, **overwrite** `pose.orientation`.
     After this, the per-point `pose.orientation` is the **motion
     direction** (i.e. where the vehicle is traveling), not the
     body-facing direction. The ego pose is flipped the same way in
     `run()` (see below), so error calculations are done in a
     consistent motion-direction frame.

After the yaw transform, `traj_preprocess` recomputes per-point
**curvature** as a central difference of the (post-flip) yaw over s,
then writes **`heading_rate_rps[i] = |v_i| · κ_i`** back into the
trajectory point. `errorCalc` reads this back as the feed-forward
curvature term.

> Note: this function intentionally **does not** re-derive
> `lateral_velocity_mps`. For OBLIQUE / Q_TURN spans, the upstream
> velocity smoother zeroes that field and re-injects the constant body
> yaw via `pose.orientation`. `crabAt()` recovers the crab angle from
> the body-yaw / path-tangent gap. See "Crab feedforward" below.

## Ego pose in reverse

`run()` builds an `odom_filter` from the tracking-differentiator's
filtered ego state (`state_x_.x1`, `state_y_.x1`, `state_yaw_.x1`). In
reverse it applies the **same π-flip** that `traj_preprocess` applied
to the trajectory:

```cpp
if (!param.is_forward) {
    q.setRPY(0, 0, yaw + M_PI);                  // ego yaw → motion direction
    odom_filter.twist.twist.linear.x = -current_vx;
    odom_filter.twist.twist.linear.y = -current_vy;
}
```

Net effect: in reverse, **everything passed to `findNearestIndex`,
`calc_preview_point`, and `errorCalc` is in the same motion-direction
frame**. The body-vs-motion duality is resolved exactly once, here.

## Nearest-point search (`findNearestIndex`)

```cpp
findNearestIndex(points, query, max_dist, max_yaw)
```

Linear scan; returns the index of the trajectory point closest to
`query.position` such that the **squared** distance ≤ `max_dist²`. The
yaw check is implemented but currently commented out — only the
distance filter is enforced.

Two call sites:

| caller | max_dist | failure consequence |
|---|---|---|
| `run()` (line 85) | 2.0 m | tracker returns false; controller emits zero command |
| `errorCalc()` (line 387) | 4.0 m | `errorCalc` returns false; tracker returns false; same as above |

A previous version compared **squared** distance against the
**linear** threshold, which silently shrunk the radius to `sqrt(max_dist)`
(2.0 m → ~1.41 m, 4.0 m → 2.0 m). That's been fixed; the threshold now
behaves as documented.

## Preview point (`calc_preview_point`)

Forward-extrapolates `odom_filter` by `predicted_ts` seconds using
body-frame longitudinal/lateral velocity and yaw rate:

```
x_preview   = x + vx·t·cos(yaw) − vy·t·sin(yaw)
y_preview   = y + vx·t·sin(yaw) + vy·t·cos(yaw)
yaw_preview = yaw + ωz·t
```

With `predicted_time = 0.3 s` (default) and cruise velocity ~3 m/s,
that's ~1 m of look-ahead. Setting `predicted_time = 0` reduces the
controller to feedback-only.

## Error calculation (`errorCalc`)

Given the preview point and the (preprocessed) trajectory, computes:

| symbol | meaning | source |
|---|---|---|
| `ey` | lateral error from preview pose to projected target | `computeLateralError` on the line between two adjacent trajectory points |
| `eyaw` | yaw error (preview vs. interpolated trajectory yaw) | `angleDiff(preview_yaw, target_yaw)` |
| `beta_t` | crab feedforward at projected target | linear interp of `crabAt(idxP)` and `crabAt(idxQ)` |
| `kappa` | curvature feedforward at projected target | linear interp of `heading_rate_rps / |v|` |

Projection is onto the chord between the nearest trajectory point and
its neighbor (chosen ahead-or-behind by the dot product against
`yaw_s`'s direction). The lateral-error sign convention is:

```
ey > 0  ⇔  target is to the LEFT of the preview pose  ⇔  ego is to the RIGHT of the path
```

The yaw / angle difference sign convention is `angleDiff(a, b) = b − a`
normalized to (−π, π].

### Crab feedforward (`crabAt`)

```cpp
double crabAt(pts, k) {
    return std::tan(angleDiff(tangent_at_k, body_yaw_at_k));
}
```

i.e. `tan(body_yaw − tangent)`, with `body_yaw` taken from the
**preprocessed** trajectory's `pose.orientation`. In reverse that's the
motion direction, not the body-facing direction, but `crabAt` doesn't
need to know — both forward and reverse paths see "yaw at point k"
versus "geometric tangent at point k" and the trigonometric meaning is
the same.

The number returned is the **tangent of the angle by which the path
tangent diverges from the per-point body / motion yaw**. For
OBLIQUE crab spans this is non-zero — that's exactly the crab angle
the controller must steer the wheels to.

## Steering law (`calcSteerCmd`)

`calcSteerCmd(ey, ey_dot, eyaw, betar, kappar, wr, vx)` produces the
four per-axle commands. It splits the steer into a **base** term `delta`
(applied to all four wheels — true crab component) and a **differential**
term `d_delta` (added/subtracted across axles to produce a yaw rate).

Common terms (both directions):

```
vx_safe = max(|vx|, vx_min)              // protect proportional gains at low speed
d_deltar = atan(L · κ / 2)               // pure curvature feedforward
```

`|vx|` is essential — in reverse `vx` is negative and a `max(vx, vx_min)`
would always collapse to `vx_min`, inflating gain magnitudes by
`cruise_speed / vx_min` (typically 24×) and producing bang-bang steering.

### Forward branch

```
delta   = +ky/vx_safe · ey + kvy/vx_safe · ey_dot − betar + k_com·vx·ωz
d_delta = +kyaw/vx_safe · eyaw + d_deltar
```

### Reverse branch

```
delta   = −ky_reverse/vx_safe · ey − kvy/vx_safe · ey_dot + betar − k_com·vx·ωz
d_delta = +kyaw_reverse/vx_safe · eyaw + d_deltar
```

The relative signs of `ky`, `kvy`, `k_com` are flipped vs. forward (so
that for an `ey > 0` measured in the motion-direction frame the
correction goes the right way after the per-axis flip below). The
**crab feedforward `betar` is flipped too** (forward uses `− betar`,
reverse uses `+ betar`) — see the rationale in the per-axis flip
section below.

### Axis mapping

```cpp
axis2 = delta_l + d_delta_l / wheel_base_ratio    // rear-inner (or front, depending on platform)
axis3 = delta_l − d_delta_l / wheel_base_ratio
axis1 = delta_l + d_delta_l
axis4 = delta_l − d_delta_l
```

`wheel_base_ratio` is the front-axle wheelbase divided by the rear-axle
wheelbase (50/116 on the IGV ≈ 0.431). The differential is scaled
asymmetrically across the two axle pairs so the resulting body yaw
rate is consistent regardless of which axle pair you read.

Pure crab is `d_delta = 0`: all four axes equal `delta_l`. Pure Ackermann
turning (no crab) is `delta = 0`: axis1/4 carry `±d_delta_l`, axis2/3
carry `±d_delta_l / wheel_base_ratio`.

### Reverse: per-axis sign flip

After the axis mapping, the reverse branch negates **all four** outputs:

```cpp
if (!is_forward) {
    cmd.axis1 = −cmd.axis1;
    cmd.axis2 = −cmd.axis2;
    cmd.axis3 = −cmd.axis3;
    cmd.axis4 = −cmd.axis4;
}
```

The reason: the per-axle hardware semantics ("positive command turns
the wheel left **relative to body forward**") are invariant of which way
the vehicle is moving. The control law, however, was written in the
**motion-direction frame** (because of the π-flip in `traj_preprocess`
and the matching ego π-flip in `run()`). In motion-direction language,
forward and reverse are identical; in body-frame language, a positive
"left of motion" command is a negative "left of body forward" command
when motion is reverse. Hence the flip.

### Why `+betar` in reverse

`crabAt` returns the same numerical value in forward and reverse for
the same geometric crab geometry. After the per-axis flip, the
crab-feedforward contribution in `axis_*_cmd` is:

```
forward axis = + (… − betar)            (no axis flip)
reverse axis = − (… ± betar)            (axis flip)
```

For both modes to produce a positive `axis_*_cmd` when the geometry
demands a positive (= body-left) wheel angle, the inner sign of
`betar` must be **opposite** between the branches:

| branch | inner expression | after axis flip (if any) |
|---|---|---|
| forward | `… − betar` | `+ (… − betar)` |
| reverse | `… + betar` | `− (… + betar) = … − betar` |

so both end up with `… − betar` in the final axis command and the crab
direction is consistent in body coordinates. This is the cancellation
arithmetic referenced in the inline comment above the reverse branch.

If you remove the `+betar` (i.e. use `− betar` in both branches), the
sign of the crab steer in reverse OBLIQUE flips and the vehicle drifts
**away from** the planned crab path — exactly the failure mode
documented in [troubleshooting.md](troubleshooting.md) under "Vehicle
drifts off the planned crab path".

## Clamps

After mixing, each axis is hard-clamped to `±max_steering_angle`
(~45° on the IGV). The pre-clamp `delta_l` and `d_delta_l` are
**also** clamped (to `delta_limit` and `d_delta_limit`) so the
clamping at the axis level is mostly defensive — if `delta_limit`
and `d_delta_limit` are set to sensible values, the per-axis clamps
rarely fire.

## Differentiator (`third_order_tracking_differentiator`)

A linear third-order tracking differentiator runs on the raw ego x, y,
and yaw signals, and again on `ey` after `errorCalc`. Pole placement
is a triple pole at `s = -ω`:

```
x1(k+1) = x1(k) + dt·x2(k)
x2(k+1) = x2(k) + dt·x3(k)
x3(k+1) = x3(k) + dt·u(k)
u(k)    = k1·(meas − x1) − k2·x2 − k3·x3
       k1 = ω³,  k2 = 3ω²,  k3 = 3ω
```

`x1` is the filtered measurement, `x2` is the velocity estimate, `x3`
is the acceleration estimate. The bandwidth `ω` defaults to 10 rad/s
for the pose differentiator and is **explicitly overwritten** to 5
rad/s for the `ey` differentiator inside the constructor. The pose
differentiator's gains are recomputed in `setParameter()` from the
configured `trk_omega`; the `ey` differentiator's gains are **not**
parameterized and are pinned at ω = 5.

## Debug output

`MotionController::onTimer()` reads six fields from the tracker and
publishes them as `/debug/data` (`std_msgs/Float32MultiArray`) on every
cycle:

| index | source | meaning |
|---:|---|---|
| 0 | `ey_debug` | lateral error in m (signed, +left of motion) |
| 1 | `delta_debug` | pre-clamp base steer command (rad) |
| 2 | `deltal_debug` | post-clamp base steer command (rad) |
| 3 | `eyaw_debug` | yaw error in rad |
| 4 | `d_delta_debug` | pre-clamp differential steer (rad) |
| 5 | `d_deltal_debug` | post-clamp differential steer (rad) |

These are **the** primary diagnostic surface for the controller — see
[troubleshooting.md](troubleshooting.md) for how to read each one.
