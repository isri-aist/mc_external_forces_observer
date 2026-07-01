# mc_external_forces_observer

An `mc_rtc` observer that estimates external joint torques acting on a robot and
optionally feeds them back to the QP controller as `setExternalTorques`.

## Requirements

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)

## Installation

The recommended way is via the
[mc_rtc superbuild](https://github.com/mc-rtc/mc-rtc-superbuild).
For a standalone build, ensure `mc_rtc` is installed, then:

```bash
cmake -B build && cmake --build build && cmake --install build
```

## Estimation methods

Two strategies are available via the `estimation_method` key:

| Method | Description | Latency |
|---|---|---|
| `ForceSensorBased` | Projects FT sensor wrenches into joint space: `τ_ext = Σ Jᵀ w` | None |
| `MomentumObserver` | Integrates the generalised momentum residual; FT sensors are fed as known inputs so only unmeasured forces incur gain-related latency | Proportional to `1/K` |

## Configuration

All keys are optional and fall back to the defaults shown below.

```yaml
ObserverPipelines:
  - name: MainPipeline
    gui: true
    log: true
    observers:
      # Required before ExternalForcesObserver when using MomentumObserver
      - type: Encoder
        velocity: encoderVelocities      # encoderFiniteDifferences also works

      # Required for floating-base robots (tested: BodySensor in sim, Tilt on hardware)
      - type: BodySensor
        update: true
        bodySensor: FloatingBase

      - type: ExternalForcesObserver
        robot: <robot_name>              # defaults to the main robot
        updateRobot: <robot_name>        # robot whose torques are updated; defaults to robot
        estimation_method: MomentumObserver # or ForceSensorBased
        is_active: true                  # false = estimate and log only, no feedback to QP
        use_forces_from_ft_sensors: true # include FT sensor wrenches in the estimate
        use_active_joints_mask: false    # zero out gripper and mimic joints in the output
        # MomentumObserver-only keys:
        residual_gain: 10.0              # observer gain K; higher = faster response, more noise
        torque_source_type: CommandedTorque  # JointTorqueMeasurement preferred when available
```

> **Floating-base robots:** add an Encoder observer and a floating-base observer
> (e.g. [`BodySensor`](https://jrl.cnrs.fr/mc_rtc/tutorials/recipes/observers.html#bodysensor-observer) or [`Tilt`](https://github.com/jrl-umi3218/mc_state_observation/pull/52)) *before* `ExternalForcesObserver` in the pipeline.

## Torque source types

| Value | Notes |
|---|---|
| `CommandedTorque` | Uses QP output |
| `JointTorqueMeasurement` | Uses joint-side sensors directly; most accurate; rotor inertia is removed from the inertia matrix |
| `CurrentMeasurement` | Not yet implemented; falls back to `CommandedTorque` |
| `MotorTorqueMeasurement` | Uses motor-side sensors directly |

## Datastore interface

Other controllers or plugins can interact with the observer at runtime:

| Key | Type | Description |
|---|---|---|
| `EF_Estimator::isActive` | `bool` | Whether feedback is currently applied |
| `EF_Estimator::toggleActive` | `void()` | Enable/disable feedback |
| `EF_Estimator::setGain` | `void(double)` | Set gain K and reset observer state |
| `EF_Estimator::getGain` | `double` | Read current gain |
| `EF_Estimator::isUsingFTSensorMeasurements` | `bool` | FT fusion status |
| `EF_Estimator::toggleFTSensorMeasurements` | `void()` | Toggle FT fusion |
| `EF_Estimator::isUsingActiveJointsMask` | `bool` | Mask status |
| `EF_Estimator::toggleActiveJointsMask` | `void()` | Toggle joint mask |