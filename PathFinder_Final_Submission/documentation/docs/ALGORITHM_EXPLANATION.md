# ALGORITHM_EXPLANATION.md — PathFinder

## 1. Problem framing
Given degraded IMU measurements (angular velocity $\omega_b(t)$ and specific force $f_b(t)$ in the body frame), estimate 6-DoF pose over time and, after complete IMU failure, predict the trajectory for 5 seconds using only prior motion history. No exteroceptive sensors or future ground truth are used in estimation.

## 2. Sensor model and conventions
- **Gyroscope:** $\omega_m = \omega + b_g + s_g \odot \omega + n_g$ (bias $b_g$, scale error $s_g$, noise $n_g$).
- **Accelerometer:** $a_m = f + b_a + s_a \odot f + n_a$, where specific force $f \approx R_{wb}^T (a_w - g_w)$ and $g_w = [0,0,-g]^T$ in a world frame with $+Z$ upward.
- **Quaternion:** $q = [w,x,y,z]$ (scalar-first internally) represents rotation from body to world, $v_w = R(q)\,v_b$.
- **Observability:** IMU-only integration cannot observe absolute position or yaw without external references; initial yaw is fixed by the chosen alignment.

## 3. Data validation and preprocessing
1. **Parsing/adaptation:** EuRoC/TUM-VI-compatible inputs are normalized to `(t_ns, gyro_xyz, accel_xyz)` with explicit column handling; malformed rows are handled safely and recorded.
2. **Timestamps:** compute $\Delta t_i = (t_i - t_{i-1})\cdot 10^{-9}$ s with robust handling for duplicates/non-positive intervals; cap extreme $\Delta t$ for numerical stability.
3. **Outliers/invalids:** non-finite values are flagged; robust per-axis clipping (median/MAD threshold) mitigates spikes. **Explicit all-zero rows are preserved** because they encode complete sensor failure in the degradation model.
4. **Stuck/gap evidence:** repeated identical samples and abnormal gaps are detected as corruption/dropout indicators for downstream failure logic.
5. **Initial bias and gravity (not assuming perfect stillness):** within an initial window, select low-motion samples using gyro norm and accelerometer-magnitude proximity to $g$ when available; otherwise fall back to robust medians over lower-motion samples. Gyro bias $\hat b_{g,0}$ is initialized from selected samples; accelerometer bias is not assumed observable and defaults to zero. Gravity magnitude $\hat g$ is estimated robustly from accelerometer norms with safe bounds.

## 4. Sensor correction
- Apply $\omega_c = S_g \omega_m - \hat b_g$, $a_c = S_a a_m - \hat b_a$ (scale factors default to identity; unknown time-varying bias/scale cannot be fully recovered from IMU alone).
- **Conservative online gyro-bias adaptation:** when measurements indicate sustained near-stationary dynamics (small $\|\omega_c\|$ and $|\|a_c\|-\hat g|/\hat g$ small), nudge $\hat b_g$ with a small learning rate to track slow drift without aggressive divergence.
- Optional low-pass filtering hooks are available for noise reduction.

## 5. Orientation estimation
- **Gyro propagation:** integrate $\dot q = \frac{1}{2} q \otimes [0,\omega_c]^T$ via rotation-vector increment $\Delta\theta = \omega_c \Delta t$ (trapezoidal gyro averaging when previous sample is available), followed by normalization.
- **Initial gravity alignment:** compute $q_0$ that aligns mean initial specific force with world $+Z$ (yaw unconstrained).
- **Accelerometer tilt correction (complementary, gated):** when $\|a_c\|$ is close to $\hat g$ (low linear acceleration), compute a small correction quaternion that nudges the rotated specific-force direction toward world gravity and blend with gain $\alpha$; correction is suppressed during high dynamics to avoid injecting linear-acceleration errors.

## 6. Trajectory propagation and drift mitigation
1. **Gravity compensation:** $f_w = R(q)a_c$, $a_{lin,w} = f_w + [0,0,-\hat g]^T$.
2. **Integration (trapezoidal):** $v_{k} = v_{k-1} + \frac{1}{2}(a_{lin,w,k-1}+a_{lin,w,k})\Delta t$, $p_k = p_{k-1} + \frac{1}{2}(v_{k-1}+v_k)\Delta t$.
3. **Conservative motion constraints:** if sustained stationarity is confidently detected (low gyro and accel magnitude near $\hat g$ over a minimum duration), apply gentle velocity damping (ZUPT-like evidence) rather than an arbitrary hard zeroing. A small global damping factor can limit numerical runaway; constraints are configurable and never force pose resets.

These measures reduce but do not eliminate IMU-only drift under strong bias/scale/noise.

## 7. Failure detection
The detector maintains state over time using: exact-zero/invalid samples, consecutive dead counts, stuck-run evidence, large timestamp gaps, and recent-window invalid ratios. States:
- **`temporary_dropout`:** short gaps or isolated unusable samples.
- **`severe_degradation`:** persistent stuck/invalid behavior or high recent corruption ratio.
- **`complete_failure`:** sustained dead evidence (e.g., consecutive zero/invalid samples) indicating loss of usable IMU signal.

Naturally small but valid readings (e.g., stationary IMU) are not treated as failure without dead/invalid/stuck evidence.

## 8. Dead reckoning (5 s prediction)
When complete failure is active, future pose is propagated without sensor updates using recent motion history $(p,v,a_{lin,w},q,\omega)$:
- Estimate recent velocity/acceleration/gyro trends (robust median over a short window) with magnitude clamping.
- Propagate with a damped model: $a_{pred}(t)=\hat a\,e^{-t/\tau}$, $v \leftarrow \mathrm{clip}(v + a_{pred}\Delta t)$, $p \leftarrow p + v\Delta t$; orientation is propagated by decayed gyro increment.
- Generate timestamped poses at a configurable interval for at least the required horizon (default 5 s). If history is insufficient, fall back to conservative constant-velocity/rest propagation.

Predictions are physically plausible but inherently uncertain; accuracy degrades with horizon length and pre-failure estimation error.

## 9. Evaluation metrics (ground truth only)
- **ATE (Absolute Trajectory Error):** position error after rigid alignment (Umeyama, no scale) between associated poses.
- **RPE (Relative Pose Error):** relative motion error over fixed pose deltas.
- **Position RMSE / orientation error:** global and windowed (pre-failure, post-failure/blind 5 s).
- **System metrics:** wall-clock processing time and measured peak memory; dropout-resilience and post-failure prediction quality reported per tier.

## 10. Limitations
- IMU-only double integration is drift-prone; long-duration accuracy is limited without aiding sensors.
- Initial yaw and absolute position are unobservable; reported ATE uses alignment where specified.
- Bias/scale are partially observable at best; online adaptation is conservative and cannot fully compensate severe time-varying errors.
- Dead-reckoning assumes motion continuity from recent history and cannot anticipate unobserved maneuvers during failure.
