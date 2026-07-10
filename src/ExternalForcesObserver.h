/*
 * Copyright 2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>
#include <Eigen/src/Core/Matrix.h>
#include <string>

/**
 * @brief Source of joint torque measurements used by the momentum observer.
 *
 * - CommandedTorque:       Use the torque commanded by the QP solver (requires
 *                          a friction model to be accurate).
 * - CurrentMeasurement:    Derive torque from motor current (requires motor
 *                          constants and a friction model — not yet implemented).
 * - JointTorqueMeasurement: Use joint-side torque directly from sensors
 *                          (most accurate when available; rotor inertia effects
 *                          are removed from the inertia matrix in this mode).
 * - MotorTorqueMeasurement: Expect motor-side torque scaled by the gear ratio 
 *                          accessible from the real robot. Compared to JointTorqueMeasurement, 
 *                          this mode does not remove rotor inertia effects.
 */
enum class TorqueSourceType
{
  CommandedTorque,
  CurrentMeasurement,
  MotorTorqueMeasurement,
  JointTorqueMeasurement,
};

/**
 * @brief Algorithm used to estimate external joint torques.
 *
 * - ForceSensorBased: Projects sensor wrenches into joint torque space:
 *                     @code
 *                       τ_ext = Σ J_s^T · R^T · w_s
 *                     @endcode
 *                     No integration, low latency, limited to sensor coverage.
 *
 * - MomentumObserver: Feeds force-sensor as known inputs into the observer, 
 *                     so the integral tracks only the unexplained residual:
 *                     @code
 *                       integral += (τ + τ_FT + C^T·qdot - g + r) · dt
 *                       r         = K · (M·qdot - integral + p0)
 *                       τ_ext_hat = τ_FT + r
 *                     @endcode
 *                     The gain-related latency applies only to unmeasured
 *                     forces; forces captured by sensors pass through at full
 *                     bandwidth.
 */
enum class EstimationMethod
{
  MomentumObserver,
  ForceSensorBased,
};

namespace mc_external_forces_observer
{

 /**
 * @brief mc_rtc global plugin that estimates external joint torques acting on a
 *        robot and optionally feeds them back to the QP controller.
 *
 * ## Overview
 *
 * The plugin runs at every control cycle (before the QP solve) and produces an
 * estimate of the external torques `τ_ext` acting on the robot joints. Two
 * estimation strategies are available (see EstimationMethod):
 *
 *   1. **Momentum observer** — integrates the generalised momentum residual:
 *      @code
 *        integral += (τ + τ_ext_FT + C^T·qdot - g + r) · dt
 *        r = K · (M·qdot - integral + p0)
 *      @endcode
 *      where `K` is the residual gain, `p0` is the initial momentum, and `r`
 *      is the momentum observer output. Force-sensor torques `τ_ext_FT` are
 *      optionally fused to reduce bias.
 *
 *   2. **Force-sensor based** — computes `τ_ext = J^T · w_sensor`
 *      directly from the measured wrenches.
 *
 *
 * ## Configuration keys (mc_rtc YAML)
 * | Key                                   | Type   | Description                                                        |
 * |---------------------------------------|--------|--------------------------------------------------------------------|
 * | `residual_gain`                       | double | Observer gain K (higher = faster, noisier)                         |
 * | `torque_source_type`                  | string | One of the TorqueSourceType names                                  |
 * | `estimation_method`                   | string | One of the EstimationMethod names                                  |
 * | `use_forces_from_ft_sensors`          | bool   | Whether to fuse force sensor measurements into the observer        |
 *
 * ## Datastore interface
 * | Key                                                  | Type   | Description                                                      |
 * |------------------------------------------------------|--------|------------------------------------------------------------------|
 * | `EF_Estimator::isActive`                             | bool   | Whether feedback is applied                                      |
 * | `EF_Estimator::toggleActive`                         | void   | Toggle feedback on/off                                           |
 * | `EF_Estimator::setGain`                              | void   | Set gain and reset observer state                                |
 * | `EF_Estimator::getGain`                              | double | Get current gain value                                           |
 * | `EF_Estimator::isUsingFTSensorMeasurements`          | bool   | Whether force sensor measurements are fused into the observer    |
 * | `EF_Estimator::toggleFTSensorMeasurements`           | void   | Toggle fusion of force sensor measurements into the observer     |
 */
struct ExternalForcesObserver : public mc_observers::Observer
{
  ExternalForcesObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;
  void reset(const mc_control::MCController & ctl) override;
  bool run(const mc_control::MCController & ctl) override;
  void update(mc_control::MCController & ctl) override;

protected:
  void addToLogger(const mc_control::MCController & ctl,
                   mc_rtc::Logger & logger,
                   const std::string & category) override;
  void addToGUI(const mc_control::MCController & ctl,
                mc_rtc::gui::StateBuilder & gui,
                const std::vector<std::string> & category) override;

private:
  std::string robot_ = ""; ///< Robot estimated by this observer
  std::string updateRobot_ = ""; ///< Robot to update (defaults to robot_)

  // ── Enum ↔ string conversion tables ────────────────────────────────────────
  static constexpr std::array<const char *, 4> torqueSourceNames =
  {
    "CommandedTorque",
    "CurrentMeasurement",
    "MotorTorqueMeasurement",
    "JointTorqueMeasurement"
  };

  static constexpr std::array<const char *, 2> estimationMethodNames =
  {
    "MomentumObserver",
    "ForceSensorBased"
  };

  static std::string toString(TorqueSourceType src);
  static TorqueSourceType toTorqueSource(const std::string & s);

  static std::string toString(EstimationMethod method);
  static EstimationMethod toEstimationMethod(const std::string & s);

  // ── Initialisation helpers ──────────────────────────────────────────────────
  void loadConfig(const mc_rtc::Configuration & config);
  void addDatastoreCall(mc_control::MCController & ctl);

  /** @brief Reset the momentum observer state. */
  void resetMomentumObserver(const mc_control::MCController & ctl);

  /**
   * @brief Generalised momentum observer for unmeasured external torques.
   *
   * Force-sensor torques are fed as known inputs so the integral term
   * accumulates only the unexplained residual. The gain-related latency
   * therefore applies only to unmeasured forces; sensor-captured forces
   * pass through at full bandwidth.
   *
   * Computed over the full DoF vector to preserve inertia coupling.
   *
   * @return Full-DoF external torque estimate (unmasked).
   */
  Eigen::VectorXd momentumObserver(const mc_control::MCController & ctl);

  /**
   * @brief Force-sensor based external torque estimation.
   *
   * For each force sensor, computes the joint torques induced by the measured
   * wrench via the world-frame Jacobian transpose:
   * @code
   *   τ_FT = Σ_sensors  J_s^T · R^T · w_s
   *   return τ_FT
   * @endcode
   *
   * @note Updates `tau_ext_ft_sensor_` as side effects.
   *
   * @return Full-DoF external torque estimate (unmasked).
   */
  Eigen::VectorXd forceSensorBasedEstimation(const mc_control::MCController & ctl);

  // ── Configuration ───────────────────────────────────────────────────────────

  TorqueSourceType tau_mes_src_;   ///< Torque source used by the momentum observer.
  EstimationMethod estimation_method_; ///< Active estimation algorithm.
  double residualGain_;            ///< Observer gain K.

  // ── Runtime state ───────────────────────────────────────────────────────────

  int nDof_;                       ///< Full robot DoF count from robot.mb().nrDof().
  bool isActive_ = false;          ///< Whether estimated torques are fed back to the QP.
  bool activeHasChanged_ = true;   ///< Tracks isActive_ transitions to log deactivation once.
  bool useFTSensorMeasurements_ = true;  ///< Whether to use force sensor data.

  // ── Observer state ──────────────────────────────────────────────────────────

  Eigen::VectorXd pZero_;          ///< Generalised momentum at initialisation time p(t0) = M(q0)·qdot0.
  Eigen::VectorXd integralTerm_;   ///< Running integral of the momentum observer (full DoF).

  // ── Torque signals ──────────────────────────────────────────────────────────

  /// Final external torque estimate sent to the controller.
  Eigen::VectorXd tau_ext_hat_;

  /// Momentum observer residual r = K·(M·qdot - integral + p0).
  Eigen::VectorXd tau_momentum_observer_;

  /// Force-sensor torque projection: Σ J_s^T · R^T · F_s.
  Eigen::VectorXd tau_ext_ft_sensor_;
  
  bool observerInitialized_ = false; ///< Whether the momentum observer has been initialised with a valid p0.

  // Utils
  inline void mbcToVector(const std::vector<std::vector<double>> & value,
                                Eigen::VectorXd & vec)
  {
      Eigen::DenseIndex idx = 0;

      for(const auto & block : value)
      {
          Eigen::DenseIndex size = static_cast<Eigen::DenseIndex>(block.size());
          if(size == 0)
              continue;

          Eigen::Map<const Eigen::VectorXd> qi(block.data(), size);

          vec.segment(idx, size) = qi;

          idx += size;
      }
  }

  bool resetObserver_;

  std::vector<std::string> dofNames_;
  std::vector<std::string> refDofOrder(const rbd::MultiBodyConfig & mbc, const rbd::MultiBody & mb);
};

} // namespace mc_external_forces_observer