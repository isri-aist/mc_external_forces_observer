#include "ExternalForcesObserver.h"
#include <mc_observers/ObserverMacros.h>
#include <mc_control/MCController.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/ArrayLabel.h>

#include <mc_tvm/Robot.h>

#include <RBDyn/Coriolis.h>

namespace mc_external_forces_observer
{

ExternalForcesObserver::ExternalForcesObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt) 
{
}

void ExternalForcesObserver::configure(const mc_control::MCController & ctl,
                                       const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobot_ = config("updateRobot", static_cast<std::string>(robot_));
  if(!ctl.robots().hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw("ExternalForcesObserver {} requires robot \"{}\" but this robot does not exit", name(), robot_);
  }
  if(!ctl.robots().hasRobot(updateRobot_))
  {
    mc_rtc::log::error_and_throw("ExternalForcesObserver {} requires robot \"{}\" (updateRobot) but this robot does not exit", name(),
                                 updateRobot_);
  }

  loadConfig(config);
  
  auto & robot = ctl.robots().robot(robot_);
  nDof_ = robot.mb().nrDof();
  dofNames_ = refDofOrder(robot.mbc(), robot.mb());
  mc_rtc::log::info("[ExternalForcesObserver][Init] Initializing observer for robot \"{}\" with {} DoFs", robot.name(), nDof_);

  pZero_ = Eigen::VectorXd::Zero(nDof_);
  tau_ext_hat_ = Eigen::VectorXd::Zero(nDof_);
  tau_momentum_observer_ = Eigen::VectorXd::Zero(nDof_);
  tau_ext_ft_sensor_ = Eigen::VectorXd::Zero(nDof_);
  integralTerm_ = Eigen::VectorXd::Zero(nDof_);
  resetObserver_ = true;

  addDatastoreCall(const_cast<mc_control::MCController &>(ctl));

  mc_rtc::log::info("[ExternalForcesObserver][Init] called with configuration:\n{}", config.dump(true, true));
}

void ExternalForcesObserver::reset(const mc_control::MCController & /* ctl */)
{
  mc_rtc::log::info("[ExternalForcesObserver][Reset] called");
  resetObserver_ = true;
}

bool ExternalForcesObserver::run(const mc_control::MCController & ctl)
{
  if(resetObserver_)
  {
    resetMomentumObserver(ctl);
    resetObserver_ = false;
  }

  auto & robot = ctl.robots().robot(robot_);
  if(estimation_method_ == EstimationMethod::MomentumObserver)
  {
    tau_ext_hat_ = momentumObserver(ctl);
  }
  else if(estimation_method_ == EstimationMethod::ForceSensorBased)
  {
    tau_ext_hat_ = forceSensorBasedEstimation(ctl);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[ExternalForcesObserver] Invalid estimation method.");
  }

  if(tau_ext_hat_.array().isNaN().any() || tau_ext_hat_.array().isInf().any())
  {
    mc_rtc::log::warning("[ExternalForcesObserver] NaN in tau_ext_hat_, resetting observer and sending zero.");
    resetObserver_ = true;
    tau_ext_hat_.setZero();
  }

  return true;
}

void ExternalForcesObserver::update(mc_control::MCController & ctl)
{
  if(activeHasChanged_ != isActive_)
  {
    activeHasChanged_ = isActive_;
    if(!isActive_)
    {
      mc_rtc::log::info("[ExternalForcesObserver] Estimation feedback deactivated, external torques set to zero.");
      ctl.robots().robot(updateRobot_).setExternalTorques(Eigen::VectorXd::Zero(nDof_));
      ctl.realRobot(updateRobot_).setExternalTorques(Eigen::VectorXd::Zero(nDof_));
    }
  }

  if(isActive_)
  {
    ctl.robots().robot(updateRobot_).setExternalTorques(tau_ext_hat_);
    ctl.realRobot(updateRobot_).setExternalTorques(tau_ext_hat_);
  }
}

void ExternalForcesObserver::resetMomentumObserver(const mc_control::MCController & ctl)
{
  integralTerm_.setZero();
  tau_momentum_observer_.setZero();
  observerInitialized_ = false;

  // Initialize pZero_ to the current momentum if the robot is moving
  auto & realRobot = ctl.realRobot(robot_);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nDof_);
  mbcToVector(realRobot.mbc().alpha, qdot);

  if(qdot.size() != nDof_)
  {
    mc_rtc::log::error("[ExternalForcesObserver] Size mismatch: qdot.size() = {}, expected nDof_ = {}", qdot.size(), nDof_);
    return;
  }

  rbd::ForwardDynamics fd = rbd::ForwardDynamics(realRobot.mb());
  fd.computeH(realRobot.mb(), realRobot.mbc());

  Eigen::MatrixXd M = fd.H();
  if (tau_mes_src_ == TorqueSourceType::JointTorqueMeasurement)
  {
    // Removing rotor inertia effects
    M -= fd.HIr();
  }

  pZero_ = M * qdot;
}

Eigen::VectorXd ExternalForcesObserver::momentumObserver(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robots().robot(robot_);
  auto & realRobot = ctl.realRobot(robot_);
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nDof_);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(nDof_);

  if(robot.encoderVelocities().empty())
  {
    mc_rtc::log::warning(
        "[ExternalForcesObserver] Encoder velocities observer is not available, external forces estimation skipped. Please add an EncoderObserver to the controller configuration with velocity update enabled to use the momentum observer estimation method.");
    return Eigen::VectorXd::Zero(nDof_);
  }

  const std::vector<std::vector<double>> * rawTorques = nullptr;
  switch(tau_mes_src_)
  {
    case TorqueSourceType::JointTorqueMeasurement:
      rawTorques = &realRobot.mbc().jointTorque;
      break;

    case TorqueSourceType::CurrentMeasurement:
      mc_rtc::log::warning("[ExternalForcesEstimator] CurrentMeasurement not implemented yet, "
                          "switching to CommandedTorque source.");
      // TODO: tau(i) = kt(i) * gear_ratio(i) * realRobot.jointJointSensor(mbIdx).motorCurrent();
      tau_mes_src_ = TorqueSourceType::CommandedTorque;
      [[fallthrough]];

    case TorqueSourceType::MotorTorqueMeasurement:
      if(tau_mes_src_ == TorqueSourceType::MotorTorqueMeasurement)
      {
        rawTorques = &realRobot.mbc().jointTorque; // Not including rotor inertia effects
      }
      break;

    case TorqueSourceType::CommandedTorque:
      rawTorques = &robot.mbc().jointTorque;
      break;
  }

  mbcToVector(*rawTorques, tau);
  mbcToVector(realRobot.mbc().alpha, qdot);

  // Print size of tau and qdot for debugging
  if(tau.size() != nDof_)
  {
    mc_rtc::log::error("[ExternalForcesObserver] Size mismatch: tau.size() = {}, expected nDof_ = {}", tau.size(), nDof_);
    return Eigen::VectorXd::Zero(nDof_);
  }

  if(qdot.size() != nDof_)
  {
    mc_rtc::log::error("[ExternalForcesObserver] Size mismatch: qdot.size() = {}, expected nDof_ = {}", qdot.size(), nDof_);
    return Eigen::VectorXd::Zero(nDof_);
  }

  rbd::ForwardDynamics fd = rbd::ForwardDynamics(realRobot.mb());
  fd.computeC(realRobot.mb(), realRobot.mbc());
  fd.computeH(realRobot.mb(), realRobot.mbc());

  rbd::Coriolis coriolis = rbd::Coriolis(realRobot.mb());
  Eigen::MatrixXd M = fd.H();
  if (tau_mes_src_ == TorqueSourceType::JointTorqueMeasurement)
  {
    // Removing rotor inertia effects
    M -= fd.HIr();
  }

  Eigen::VectorXd pt = M * qdot; // Momentum at current time

  Eigen::MatrixXd C = coriolis.coriolis(realRobot.mb(), realRobot.mbc());
  Eigen::VectorXd Cqdot_plus_g = fd.C();
  Eigen::VectorXd g = -(C*qdot - Cqdot_plus_g);

  forceSensorBasedEstimation(ctl);
  
  integralTerm_ += (tau + tau_ext_ft_sensor_ + C.transpose() * qdot - g + tau_momentum_observer_) * ctl.timeStep;
  tau_momentum_observer_ = residualGain_ * (pt - integralTerm_ + pZero_);

  if(tau_ext_ft_sensor_.size() != tau_momentum_observer_.size())
  {
    mc_rtc::log::error("[ExternalForcesObserver] SIZE MISMATCH: tau_ext_ft_sensor_={} tau_momentum_observer_={}",
      tau_ext_ft_sensor_.size(), tau_momentum_observer_.size());
    return Eigen::VectorXd::Zero(nDof_);
  }

  return  tau_ext_ft_sensor_ + tau_momentum_observer_;
}

Eigen::VectorXd ExternalForcesObserver::forceSensorBasedEstimation(const mc_control::MCController & ctl)
{
  auto & realRobot = ctl.realRobot(robot_);
  tau_ext_ft_sensor_ = Eigen::VectorXd::Zero(nDof_);
  const auto & forceSensors = realRobot.forceSensors();
  // Check if the list is empty, which can happen if the robot model doesn't include any force sensors or if there's an issue with loading them
  if(forceSensors.empty())
  {
    if(useFTSensorMeasurements_) mc_rtc::log::warning("[ExternalForcesEstimator] No force sensors found in the robot model, force sensor based estimation will return zero.");
    return tau_ext_ft_sensor_;
  }

  if(useFTSensorMeasurements_)
  {
    for(const auto & ft_sensor : realRobot.forceSensors())
    {
      // Transformation from parent body origin to sensor frame, used to place
      // the Jacobian at the exact sensor location rather than the body origin,
      // ensuring the moment arm is correct
      const sva::PTransformd & X_p_f = ft_sensor.X_p_f();
      auto jac = rbd::Jacobian(realRobot.mb(), ft_sensor.parentBody(), X_p_f.translation());

      // World-frame Jacobian (6 x path_dof), then expanded to full robot DoF
      // so J^T maps a world-frame wrench to all joint torques
      Eigen::MatrixXd shortJac = jac.jacobian(realRobot.mb(), realRobot.mbc());
      Eigen::MatrixXd fullJac = Eigen::MatrixXd::Zero(6, nDof_);
      jac.fullJacobian(realRobot.mb(), shortJac, fullJac);

      // wrenchWithoutGravity returns the wrench in the sensor (body) frame.
      // R.transpose() rotates it to the world frame to match the world-frame
      // Jacobian — virtual work requires both to be expressed in the same frame
      const Eigen::Matrix3d & R = realRobot.bodyPosW(ft_sensor.parentBody()).rotation();
      sva::ForceVecd w = ft_sensor.wrenchWithoutGravity(realRobot);
      w.force() = R.transpose() * w.force();
      w.couple() = R.transpose() * w.couple();

      // τ_ext += J^T * F: project the external wrench into joint torque space
      // and accumulate contributions from all sensors
      tau_ext_ft_sensor_ += fullJac.transpose() * w.vector();
    }
  }
  return tau_ext_ft_sensor_;
}

void ExternalForcesObserver::loadConfig(const mc_rtc::Configuration & config)
{
  // load config
  // residual_gain: 10 # Higher is better, but lead to more noise in the estimation (recommended values are between 10 and dt/2)
  // torque_source_type: CommandedTorque # Options: JointTorqueMeasurement, CommandedTorque, EstimatedTorque
  // estimation_method: MomentumObserver # Options: MomentumObserver, ForceSensorBased (MomentumObserver includes the use of the FT sensors)
  // use_forces_from_ft_sensors: true # If true, the forces from the FT sensors will be used in the estimation
  // is_active: true # If false, the observer will run but the estimated external torques will not be applied to the robot (useful for debugging or to just log the estimation without applying it as feedback)
  residualGain_ = config("residual_gain", 10.0);
  tau_mes_src_ = toTorqueSource(config("torque_source_type", std::string("CommandedTorque")));
  estimation_method_ = toEstimationMethod(config("estimation_method", std::string("MomentumObserver")));
  useFTSensorMeasurements_ = config("use_forces_from_ft_sensors", true);
  isActive_ = config("is_active", true);
}

void ExternalForcesObserver::addToGUI(const mc_control::MCController & ctl,
                                      mc_rtc::gui::StateBuilder & gui,
                                      const std::vector<std::string> & category)
{
  const auto & robot = ctl.robots().robot(robot_);
  gui.addElement(
      category, mc_rtc::gui::Checkbox("Is estimation feedback active", 
        [this]() { return isActive_; },
        [this]() 
        {
          resetObserver_ = true;
          isActive_ = !isActive_; 
        }),
      mc_rtc::gui::Checkbox("Use sensor measurements", useFTSensorMeasurements_),
      mc_rtc::gui::NumberInput(
          "Gain", [this]() { return residualGain_; },
          [this](double gain) {
            if(gain != residualGain_)
            {
              resetObserver_ = true;
            }
            residualGain_ = gain;
          }),
      mc_rtc::gui::ComboInput(
      "Estimation Mode",
      std::vector<std::string>(
          estimationMethodNames.begin(),
          estimationMethodNames.end()),
      [this]() -> std::string
      {
        return toString(estimation_method_);
      },
      [this](const std::string & v)
      {
        resetObserver_ = true;
        estimation_method_ = toEstimationMethod(v);
      }),
      mc_rtc::gui::ComboInput(
      "Torque measurement source",
      std::vector<std::string>(
          torqueSourceNames.begin(),
          torqueSourceNames.end()),
      [this]() -> std::string
      {
        return toString(tau_mes_src_);
      },
      [this](const std::string & v)
      {
        resetObserver_ = true;
        tau_mes_src_ = toTorqueSource(v);
      }),
      mc_rtc::gui::ArrayLabel("Torque Ext Estimated", dofNames_, [this]() { return tau_ext_hat_; }),
      mc_rtc::gui::ArrayLabel("Torque Ext from Momentum Observer", dofNames_,
                              [this]() { return tau_momentum_observer_; }),
      mc_rtc::gui::ArrayLabel("Torque Ext from Force Sensors", dofNames_,
                              [this]() { return tau_ext_ft_sensor_; })
      );
}

void ExternalForcesObserver::addToLogger(const mc_control::MCController & /* ctl */,
                                         mc_rtc::Logger & logger,
                                         const std::string & category)
{
  logger.addLogEntry(category + "_gain", this, [this]() { return residualGain_; });
  logger.addLogEntry(category + "_isActive", this, [this]() { return isActive_; });
  logger.addLogEntry(category + "_useFTSensorMeasurements", this, [this]() { return useFTSensorMeasurements_; });

  for(size_t i = 0; i < dofNames_.size(); ++i)
  {
    logger.addLogEntry(category + "_tauExtHat_" + dofNames_[i], this, [this, i]() { return tau_ext_hat_(i); });
    logger.addLogEntry(category + "_tauMomentumObserver_" + dofNames_[i], this, [this, i]() { return tau_momentum_observer_(i); });
    logger.addLogEntry(category + "_integralTerm_" + dofNames_[i], this, [this, i]() { return integralTerm_(i); });
    logger.addLogEntry(category + "_tauExtFtSensor_" + dofNames_[i], this, [this, i]() { return tau_ext_ft_sensor_(i); });
  }
}

void ExternalForcesObserver::addDatastoreCall( mc_control::MCController & ctl)
{
  ctl.datastore().make_call("EF_Estimator::isActive", [this]() { 
    return isActive_; 
  });
  ctl.datastore().make_call("EF_Estimator::toggleActive", [this]() { 
    isActive_ = !isActive_; 
    if(isActive_) resetObserver_ = true;
  });
  ctl.datastore().make_call("EF_Estimator::setGain", [this](double gain) {
    resetObserver_ = true;
    residualGain_ = gain;
  });
  ctl.datastore().make_call("EF_Estimator::getGain", [this]() { return residualGain_; });
  ctl.datastore().make_call("EF_Estimator::isUsingFTSensorMeasurements",
                                   [this]() { return useFTSensorMeasurements_; });
  ctl.datastore().make_call("EF_Estimator::toggleFTSensorMeasurements",
                                   [this]() { useFTSensorMeasurements_ = !useFTSensorMeasurements_; });
}

std::string ExternalForcesObserver::toString(TorqueSourceType src)
{
  return torqueSourceNames[static_cast<size_t>(src)];
}

TorqueSourceType ExternalForcesObserver::toTorqueSource(const std::string & s)
{
  for(size_t i = 0; i < torqueSourceNames.size(); ++i)
  {
    if(s == torqueSourceNames[i])
    {
      return static_cast<TorqueSourceType>(i);
    }
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("[ExternalForcesObserver] Invalid torque source type: {}", s);
}

std::string ExternalForcesObserver::toString(EstimationMethod method)
{
  return estimationMethodNames[static_cast<size_t>(method)];
}

EstimationMethod ExternalForcesObserver::toEstimationMethod(const std::string & s)
{
  for(size_t i = 0; i < estimationMethodNames.size(); ++i)
  {
    if(s == estimationMethodNames[i])
    {
      return static_cast<EstimationMethod>(i);
    }
  }
  mc_rtc::log::error_and_throw<std::runtime_error>("[ExternalForcesObserver] Invalid estimation method: {}", s);
}

std::vector<std::string> ExternalForcesObserver::refDofOrder(const rbd::MultiBodyConfig & mbc, const rbd::MultiBody & mb)
{
  std::vector<std::string> dofNames;
  const auto & alpha = mbc.alpha; // DoF Size variable
  bool robotIsFloatingBase =
      (mb.nrJoints() > 0 && mb.joint(0).type() == rbd::Joint::Free);

  for(size_t i = 0; i < alpha.size(); ++i)
  {
    const auto & block = alpha[i];
    Eigen::DenseIndex size = static_cast<Eigen::DenseIndex>(block.size());
    if(size == 0)
    {
      continue;
    }

    if(i == 0 && robotIsFloatingBase)
    {
      static const std::array<std::string, 6> freeFlyerNames = {"rx", "ry", "rz", "x", "y", "z"};
      dofNames.insert(dofNames.end(), freeFlyerNames.begin(), freeFlyerNames.end());
    }
    else
    {
      const std::string & jointName = mb.joint(static_cast<int>(i)).name();

      if(size == 1)
      {
        dofNames.push_back(jointName);
      }
      else
      {
        for(Eigen::DenseIndex k = 0; k < size; ++k)
        {
          dofNames.push_back(jointName + "_" + std::to_string(k));
        }
      }
    }
  }
  // Assert that the number of DoF names matches the size of the number of dofs in the MultiBodyConfig
  if(dofNames.size() != static_cast<size_t>(mb.nrDof()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[ExternalForcesObserver] Inconsistent DoF count");
  }
  return dofNames;
}

} // namespace mc_external_forces_observer

EXPORT_OBSERVER_MODULE("ExternalForcesObserver", mc_external_forces_observer::ExternalForcesObserver)