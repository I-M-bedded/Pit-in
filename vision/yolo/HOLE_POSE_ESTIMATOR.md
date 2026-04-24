# Hole Pose Estimator

## Overview

This document summarizes the current online hole-pose pipeline used by:

- `vision/vision_node.py`
- `vision/yolo/hole_pose_estimator.py`
- `vision/yolo/world_tracker.py`

The estimator returns the 3-D position of the center hole in the camera frame.
`VisionNode` then transforms that position into the robot/world frame and
`WorldTracker` stabilizes it with an EMA in world coordinates.

The current estimator is tiered:

- Tier 1: paired center detections, inner-hole conic recovery
- Tier 2a: single reliable center detection fallback
- Tier 2b: final fallback using hybrid 3-D hole anchors and row geometry


## Full Runtime Structure

Per frame, the online pipeline is:

1. `VisionNode` grabs the current RGB frame from RealSense.
2. `VisionNode` queries FK and computes:
   - `cam_to_world in R^(3x3)`
   - `cam_origin_world in R^3`
3. `VisionNode` asks `WorldTracker` for the current filtered world estimate.
4. If that estimate exists, `VisionNode` projects it into the current image to
   produce `prior_xy`.
5. `HolePoseEstimator.estimate(img_bgr, prior_xy)` returns:
   - `center_xy`: center-hole pixel
   - `pose["tvec_m"]`: camera-frame 3-D position
   - `confidence`
   - `source`
6. `WorldTracker.update(...)` transforms camera-frame `tvec_m` into world space
   and applies EMA.
7. `VisionNode` publishes the filtered world position.


## Object Representation

Each detection contains:

- `kp_0`: detector center keypoint
- 8 boundary keypoints
- keypoint confidences
- optional ellipse fit from boundary points

For each class, the object model is a planar 9-point template:

- point 0: object center
- points 1..8: boundary samples on a circle or slot

Current class geometry:

- `center_hole_inner`: circle, diameter `0.026 m`
- `center_hole_outer`: slot, size `0.072 x 0.040 m`
- `guide_hole_inner`: circle, diameter `0.026 m`
- `guide_hole_outer`: circle, diameter `0.040 m`


## Camera Model

The standard pinhole model is used:

```text
u = fx * X / Z + cx
v = fy * Y / Z + cy
```

for a camera-frame 3-D point:

```text
t = [X, Y, Z]^T
```


## Ellipse and Conic Recovery

When enough boundary keypoints survive, an ellipse is fit in image space.

Let the fitted ellipse be converted into a conic matrix:

```text
[x y 1] C [x y 1]^T = 0
```

Then the back-projected cone in normalized camera coordinates is:

```text
Q = K^(-T) C K^(-1)
```

where `K` is the camera intrinsic matrix.

After eigen-decomposition:

```text
Q = V diag(lambda1, lambda2, lambda3) V^T
```

the current implementation uses the closed-form circle back-projection rule:

```text
s1 = sqrt((lambda2 - lambda1) / (lambda3 - lambda1))
s3 = sqrt((lambda3 - lambda2) / (lambda3 - lambda1))
n  = s1 * v3 +/- s3 * v1
```

This produces:

- `corrected_xy`: perspective-corrected projected circle center
- `normal_camera`: plane normal candidate


## Closed-Form Circle Pose

For true circle targets, the estimator uses `conic_to_pose(...)`.

Depth is approximated from the major semi-axis:

```text
f_eff = sqrt(fx * fy)
Z ~= f_eff * r / a_major_px
```

where:

- `r` is the real circle radius
- `a_major_px` is the fitted major semi-axis in pixels

Then the corrected image center is back-projected:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
t = [X, Y, Z]^T
```

This branch does not use PnP.


## PnP Solve

Whenever the estimator needs template-based pose recovery, it uses:

- `SOLVEPNP_IPPE` for planar targets when applicable
- then `SOLVEPNP_ITERATIVE`
- then `SOLVEPNP_SQPNP`

The reprojection error is:

```text
e_reproj = sqrt( mean_i ||u_i_hat - u_i||^2 )
```


## Tier 1

### Conditions

Tier 1 activates when:

- `center_hole_outer` exists
- `center_hole_inner` exists
- both have ellipse fits
- outer passes the slot aspect-ratio gate

### Reliability

Tier 1 compares the detector center keypoints directly:

```text
d_kp = ||kp0_outer - kp0_inner||
reliability = conf_outer * conf_inner / (1 + d_kp / sigma)
```

If:

```text
reliability >= reliability_threshold
```

then the pair is accepted.

### Output

The final center and pose come from the inner circle conic branch:

- preferred: `conic_to_pose(inner ellipse)`
- fallback: `kp0_inner` + `solvePnP`


## Tier 2a

Tier 2a is the single-center fallback.

### Case 2a-i: inner center only

If the inner center exists with:

```text
conf_inner >= INNER_ALONE_CONF
```

then:

- use inner ellipse
- run circle conic recovery
- fallback to ellipse center + PnP if conic fails

Its returned confidence is:

```text
conf_2a_inner = 0.8 * conf_inner
```

### Case 2a-ii: outer center only

If only the outer slot is reliable enough:

- keep `kp0_outer` as the center
- solve PnP directly

Its returned confidence is:

```text
conf_2a_outer = 0.7 * conf_outer
```

Conic back-projection is not used here because the outer center target is a
slot, not a projected circle.


## Tier 2b

Tier 2b is the final fallback for bad conditions, especially when center
detections are unreliable.

The core idea is:

- recover a 3-D center for each observed hole
- merge duplicate detections that belong to the same physical hole
- exploit the row geometry of the board

### 1. Canonical Hole Center

The dataset geometry distinguishes:

- LV1: outer contour surface at `0.025 m`
- LV2: actual hole center depth at `0.055 m`

So the canonical offset is:

```text
Delta = 0.055 - 0.025 = 0.03 m
```

For outer detections, the observed surface center is shifted along the local
object `z` axis:

```text
t_hole = t_surface + Delta * z_axis
```

For inner detections:

```text
t_hole = t_surface
```

This makes inner and outer detections comparable in one common 3-D reference.

### 2. Per-Detection Hybrid Anchor

Each detection tries to produce one canonical 3-D hole center.

Two candidate anchors are attempted:

1. boundary-only anchor
2. kp0-assisted rescue anchor

#### Boundary-only anchor

- use boundary keypoints only
- exclude point 0 from PnP
- convert solved surface center to canonical hole center

#### kp0 rescue anchor

- use `kp0` as the center point
- run another PnP solve
- convert solved surface center to canonical hole center

### 3. Candidate Weights

Let:

- `c = detection confidence`
- `k = kp0 confidence`
- `b = boundary_support = min(num_valid_boundary / 8, 1)`
- `g = 1 / (1 + e_reproj / 2)`

Then the current weights are:

Boundary candidate:

```text
support_boundary = clip(0.25 + 0.75 * b, 0, 1)
w_boundary = c^2 * g * support_boundary
```

kp0 rescue candidate:

```text
support_kp0 = clip(0.20 + 0.30 * b + 0.50 * k, 0, 1)
w_kp0 = c^2 * g * support_kp0 * CASE2B_RESCUE_WEIGHT
```

where:

```text
CASE2B_RESCUE_WEIGHT = 0.65
```

### 4. Pixel Consistency Against kp0

Each candidate is projected back to the image and compared to `kp0`:

```text
d = ||u_pred - kp0||
consistency = exp( -0.5 * (d / sigma)^2 )
```

This does not directly replace geometry with `kp0`.
Instead, it measures whether the recovered 3-D center is consistent with the
detector center signal.

### 5. Hybrid Blending

If both candidates exist, the estimator blends them.

Scores:

```text
score_b = w_boundary * (0.55 + 0.45 * consistency_b)
score_k = w_kp0     * (0.35 + 0.65 * consistency_k)
alpha = score_b / (score_b + score_k)
```

Blended center:

```text
t_hybrid = alpha * t_boundary + (1 - alpha) * t_kp0
```

Final anchor weight:

```text
agreement = max(consistency_b, consistency_k)
w_hybrid = (alpha * w_boundary + (1 - alpha) * w_kp0) * (0.60 + 0.40 * agreement)
```

### 6. Duplicate-Hole Clustering

Multiple detections may refer to the same physical hole.

Tier 2b merges anchors within:

```text
||t_i - t_j|| < 0.04 m
```

Cluster centers are updated by weighted averaging:

```text
t_cluster = (sum_i w_i t_i) / (sum_i w_i)
```

### 7. Weighted Row Recovery

Let the surviving cluster centers be:

```text
t_i in R^3
```

with normalized weights:

```text
w_i >= 0, sum_i w_i = 1
```

Compute the weighted mean:

```text
mu = sum_i w_i t_i
```

and covariance:

```text
Sigma = sum_i w_i (t_i - mu)(t_i - mu)^T
```

The principal eigenvector of `Sigma` is the row direction:

```text
d = principal_eigenvector(Sigma)
```

Project each hole center onto the row:

```text
p_i = (t_i - mu)^T d
```

### 8. Board Template Match

The board template offsets are:

```text
o = [-0.22, -0.08, 0.0, 0.08, 0.22] m
```

If only `k` holes are available, Tier 2b enumerates all size-`k` subsets of
that template and solves:

```text
p_i ~= s * o_i + c
```

in weighted least squares form:

```text
[s, c]^T = (A^T W A)^(-1) A^T W p
```

where:

```text
A = [o_i, 1]
```

Residual:

```text
r = sqrt( sum_i w_i (p_i - (s o_i + c))^2 )
```

Score:

```text
score = r + 0.01 * |s - 1|
```

The best subset defines the board center:

```text
t_center = mu + c * d
```

### 9. Symmetry Breaking With EMA Prior

Yes. Tier 2b uses the EMA-managed previous world estimate as its prior.

More precisely:

1. `WorldTracker` stores the filtered center position in world coordinates.
2. `VisionNode._compute_prior_xy(...)` fetches that world estimate.
3. It converts that world estimate into the current camera frame:

```text
t_prior_cam = R_cw^T (x_world_prev - p_cam_world)
```

where:

- `R_cw = cam_to_world`
- `p_cam_world = cam_origin`

4. It projects that 3-D point into the current image:

```text
u_prior = fx * X / Z + cx
v_prior = fy * Y / Z + cy
prior_xy = [u_prior, v_prior]^T
```

5. `HolePoseEstimator.estimate(..., prior_xy=prior_xy)` receives it.
6. In Tier 2b, after one board-center candidate is found, the mirrored center:

```text
t_mirror = mu - c * d
```

is also projected to the image.
If the mirrored candidate is closer to `prior_xy`, the estimator flips to it.

The symmetry-breaking rule is:

```text
if ||pi(t_mirror) - prior_xy|| < ||pi(t_center) - prior_xy||:
    choose t_mirror
else:
    choose t_center
```

So the prior is not the raw previous image center.
It is the previous EMA-filtered world estimate, reprojected into the current
frame.


## Tier 2b Confidence

After row matching, Tier 2b returns:

```text
residual_gain = 1 / (1 + residual_m / 0.005)
confidence_2b = mean(cluster_confidences) * 0.4 * residual_gain
```

This intentionally keeps Tier 2b conservative relative to Tier 1 and Tier 2a.


## WorldTracker

`WorldTracker` exists because the camera moves with the robot.
Image-space EMA is therefore not physically meaningful.

Given camera-frame output:

```text
t_cam
```

the world position is:

```text
x_world = p_cam_world + R_cw * t_cam
```

EMA update:

```text
alpha = ema_alpha * confidence^(conf_power)
x_hat_t = alpha * x_world + (1 - alpha) * x_hat_(t-1)
c_hat_t = alpha * confidence + (1 - alpha) * c_hat_(t-1)
```

If a frame is missing or below `update_min_confidence`, the tracker decays its
stored confidence instead of blending in that measurement.


## Output Semantics

`HolePoseEstimator` returns:

- `center_xy`: image-space center estimate
- `pose["tvec_m"]`: camera-frame center position
- `confidence`
- `source`

`WorldTracker` returns:

- `world_xyz`: filtered world-space center position
- `confidence`
- `source`

`VisionNode` publishes `world_xyz` on `/cam0` or `/cam1`.


## Current Design Intent

The estimator currently follows this philosophy:

- use direct conic geometry whenever the circle observation is trustworthy
- use single-object PnP when only one reliable center target exists
- use multi-hole row constraints only as the final fallback
- keep Tier 2b conservative and stabilize it through world-space EMA


## Current Practical Limitations

Important current limitations are:

- conic depth uses a first-order approximation based on the major axis
- outer-hole canonical shift depends on the recovered local `z` axis
- Tier 2b assumes observed holes lie on one dominant row
- Tier 2b needs at least two usable hole anchors
- Tier 2b pose orientation is inherited from the best anchor, while only its
  translation is replaced by the fitted board center
