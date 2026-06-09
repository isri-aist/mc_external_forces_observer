#include "ExternalForcesObserver.h"
#include <mc_observers/ObserverMacros.h>
#include <mc_control/MCController.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/ArrayLabel.h>

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

  if(useActiveJointsMask_)
  {
    initializeActiveJoints(robot);
    activeJointsInitialized_ = true;
  }
  else
  {
    activeJoints_ = Eigen::VectorXd::Ones(nDof_);
  }
 
  // Initialize the momentum observer
  rbd::ForwardDynamics fd = rbd::ForwardDynamics(robot.mb());
  fd.computeC(robot.mb(), robot.mbc());
  fd.computeH(robot.mb(), robot.mbc());
  Eigen::MatrixXd M = fd.H();
  if (tau_mes_src_ == TorqueSourceType::JointTorqueMeasurement)
  {
    // Removing rotor inertia effects
    M -= fd.HIr();
  }
  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(nDof_);
  auto & realRobot = ctl.realRobot(robot_);
  bool robotIsFloatingBase = (realRobot.mb().nrJoints() > 0
                              && realRobot.mb().joint(0).type() == rbd::Joint::Free);
  int pos = 0;
  if(robotIsFloatingBase)
  {
    // Floating base velocity (6 DoF)
    const auto & alpha_fb = realRobot.mbc().alpha[0];
    for(int k = 0; k < 6; ++k) qdot(pos++) = alpha_fb[k];
  }
  for(int ji = robotIsFloatingBase ? 1 : 0; ji < realRobot.mb().nrJoints(); ++ji)
  {
    const auto & alpha_j = realRobot.mbc().alpha[ji];
    for(size_t k = 0; k < alpha_j.size(); ++k) qdot(pos++) = alpha_j[k];
  }
  pZero_ = M * qdot;

  tau_ext_hat_ = Eigen::VectorXd::Zero(nDof_);
  tau_momentum_observer_ = Eigen::VectorXd::Zero(nDof_);
  tau_ext_ft_sensor_ = Eigen::VectorXd::Zero(nDof_);
  integralTerm_ = Eigen::VectorXd::Zero(nDof_);

  addDatastoreCall(const_cast<mc_control::MCController &>(ctl));

  mc_rtc::log::info("[ExternalForcesObserver][Init] called with configuration:\n{}", config.dump(true, true));
}

void ExternalForcesObserver::reset(const mc_control::MCController & /* ctl */)
{
  mc_rtc::log::info("[ExternalForcesObserver][Reset] called");
}

bool ExternalForcesObserver::run(const mc_control::MCController & ctl)
{
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

  if(useActiveJointsMask_ && !activeJointsInitialized_)
  {
    initializeActiveJoints(robot);
    activeJointsInitialized_ = true;
  }
  else if(!useActiveJointsMask_ && activeJointsInitialized_)
  {
    activeJoints_ = Eigen::VectorXd::Ones(nDof_);
    activeJointsInitialized_ = false;
  }

  // Apply the active joint mask to the estimated external torque
  tau_ext_hat_ = tau_ext_hat_.cwiseProduct(activeJoints_);

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

void ExternalForcesObserver::resetMomentumObserver()
{
  integralTerm_.setZero();
  tau_momentum_observer_.setZero();
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

  const std::vector<double> * rawTorques = nullptr;
  switch(tau_mes_src_)
  {
    case TorqueSourceType::JointTorqueMeasurement:
      rawTorques = &realRobot.jointTorques();
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
        mc_rtc::log::warning("[ExternalForcesEstimator] MotorTorqueMeasurement not implemented yet, "
                            "switching to CommandedTorque source.");
        // TODO: tau(i) = realRobot.jointTorques(i) * mb.joint(mbIdx).gearRatio();
        tau_mes_src_ = TorqueSourceType::CommandedTorque;
      }
      [[fallthrough]];

    case TorqueSourceType::CommandedTorque:
      rawTorques = &robot.jointTorques();
      break;
  }

  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    int mbIdx  = robot.mb().jointIndexByName(rjo[i]);
    if(mbIdx < 0) continue;

    int dofIdx = robot.mb().jointPosInDof(mbIdx);
    if(robot.mb().joint(mbIdx).dof() != 1) continue;
    if(dofIdx < 0 || dofIdx >= nDof_) continue;

    tau(dofIdx) = (*rawTorques)[i];
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

  qdot = Eigen::VectorXd::Zero(nDof_);
  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);
  int pos = 0;
  if(robotIsFloatingBase)
  {
    // Floating base velocity (6 DoF)
    const auto & alpha_fb = realRobot.mbc().alpha[0];
    for(int k = 0; k < 6; ++k) qdot(pos++) = alpha_fb[k];
  }
  for(int ji = robotIsFloatingBase ? 1 : 0; ji < realRobot.mb().nrJoints(); ++ji)
  {
    const auto & alpha_j = realRobot.mbc().alpha[ji];
    for(size_t k = 0; k < alpha_j.size(); ++k) qdot(pos++) = alpha_j[k];
  }
  Eigen::VectorXd pt = M * qdot; // Momentum at current time

  Eigen::MatrixXd C = coriolis.coriolis(realRobot.mb(), realRobot.mbc());
  Eigen::VectorXd Cqdot_plus_g = fd.C();
  Eigen::VectorXd g = -(C*qdot - Cqdot_plus_g);

  forceSensorBasedEstimation(ctl);
  
  integralTerm_ += (tau + tau_ext_ft_sensor_ + C.transpose() * qdot - g + tau_momentum_observer_) * ctl.timeStep;
  tau_momentum_observer_ = residualGain_ * (pt - integralTerm_ + pZero_);

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

void ExternalForcesObserver::initializeActiveJoints(const mc_rbdyn::Robot & robot)
{
  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);

  // Collect gripper joints to exclude from estimation
  std::vector<std::string> activeGripperJoints;
  for(const auto & g : robot.grippers())
  {
    for(const auto & n : g.get().activeJoints())
    {
      activeGripperJoints.push_back(n);
    }
  }
  auto isActiveGripperJoint = [&](const std::string & jointName) {
    return std::find(activeGripperJoints.begin(), activeGripperJoints.end(), jointName)
           != activeGripperJoints.end();
  };

  // Initialize mask to zero over the full DoF vector
  activeJoints_ = Eigen::VectorXd::Zero(robot.mb().nrDof());

  // Floating base: first 6 DoFs are always estimated
  if(robotIsFloatingBase)
  {
    activeJoints_.head(6).setOnes();
  }

  // Walk the multibody joint list to stay in sync with the actual DoF vector
  // layout. pos tracks the current position in the DoF vector.
  // Joint 0 is either the floating base (Free, 6 DoF, already handled
  // above) or the fixed root (0 DoF), so we start at joint index 1 in both cases.
  int pos = robotIsFloatingBase ? 6 : 0;
  for(int ji = 1; ji < robot.mb().nrJoints(); ++ji)
  {
    const auto & j = robot.mb().joint(ji);
    if(j.dof() != 1)
    {
      // Multi-DoF or 0-DoF joints (fixed, mimic root) — advance pos and skip
      pos += j.dof();
      continue;
    }

    if(!j.isMimic() && !isActiveGripperJoint(j.name()))
    {
      activeJoints_(pos) = 1;
      mc_rtc::log::info("[ExternalForcesObserver][initializeActiveJoints] Estimated joint (pos {}) -> {}", pos,
                        j.name());
    }
    else
    {
      mc_rtc::log::info("[ExternalForcesObserver][initializeActiveJoints] Joint {} excluded from estimation.",
                        j.name());
    }

    pos++;
  }

  mc_rtc::log::info("[ExternalForcesObserver][initializeActiveJoints] Active DoFs: {}/{}",
                    static_cast<int>(activeJoints_.sum()), robot.mb().nrDof());
}

void ExternalForcesObserver::loadConfig(const mc_rtc::Configuration & config)
{
  // load config
  // residual_gain: 10 # Higher is better, but lead to more noise in the estimation (recommended values are between 10 and dt/2)
  // torque_source_type: CommandedTorque # Options: JointTorqueMeasurement, CommandedTorque, EstimatedTorque
  // estimation_method: MomentumObserver # Options: MomentumObserver, ForceSensorBased (MomentumObserver includes the use of the FT sensors)
  // use_active_joints_mask: false # If true, the mimic and grippers related joints will be masked out in the estimation
  // use_forces_from_ft_sensors: true # If true, the forces from the FT sensors will be used in the estimation
  // is_active: true # If false, the observer will run but the estimated external torques will not be applied to the robot (useful for debugging or to just log the estimation without applying it as feedback)
  residualGain_ = config("residual_gain", 10.0);
  tau_mes_src_ = toTorqueSource(config("torque_source_type", std::string("CommandedTorque")));
  estimation_method_ = toEstimationMethod(config("estimation_method", std::string("MomentumObserver")));
  useActiveJointsMask_ = config("use_active_joints_mask", false);
  useFTSensorMeasurements_ = config("use_forces_from_ft_sensors", true);
  isActive_ = config("is_active", true);
}

void ExternalForcesObserver::addToGUI(const mc_control::MCController & ctl,
                                      mc_rtc::gui::StateBuilder & gui,
                                      const std::vector<std::string> & category)
{
  const auto & robot = ctl.robots().robot(robot_);
  std::vector<std::string> jointNames;
  bool robotIsFloatingBase = (robot.mb().nrJoints() > 0 && robot.mb().joint(0).type() == rbd::Joint::Free);
  if(robotIsFloatingBase)
  {
    jointNames.reserve(6 + robot.refJointOrder().size());
    jointNames.insert(jointNames.end(), {"rx", "ry", "rz", "x", "y", "z"});
    jointNames.insert(jointNames.end(), robot.refJointOrder().begin(), robot.refJointOrder().end());
  }
  else
  {
    jointNames = robot.refJointOrder();
  }

  gui.addElement(
      category, mc_rtc::gui::Checkbox("Is estimation feedback active", isActive_),
      mc_rtc::gui::Checkbox("Use sensor measurements", useFTSensorMeasurements_),
      mc_rtc::gui::Checkbox("Active Gripper & Mimic joints mask", useActiveJointsMask_),
      mc_rtc::gui::NumberInput(
          "Gain", [this]() { return residualGain_; },
          [this](double gain) {
            if(gain != residualGain_)
            {
              resetMomentumObserver();
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
        resetMomentumObserver();
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
        resetMomentumObserver();
        tau_mes_src_ = toTorqueSource(v);
      }),
      mc_rtc::gui::ArrayLabel("Torque Ext Estimated", jointNames, [this]() { return tau_ext_hat_; }),
      mc_rtc::gui::ArrayLabel("Torque Ext from Momentum Observer", jointNames,
                              [this]() { return tau_momentum_observer_; }),
      mc_rtc::gui::ArrayLabel("Torque Ext from Force Sensors", jointNames,
                              [this]() { return tau_ext_ft_sensor_; })
      );
}

void ExternalForcesObserver::addToLogger(const mc_control::MCController & /* ctl */,
                                         mc_rtc::Logger & logger,
                                         const std::string & category)
{
  logger.addLogEntry(category + "_gain", this, [this]() { return residualGain_; });
  logger.addLogEntry(category + "_tauExtHat", this, [this]() { return tau_ext_hat_; });
  logger.addLogEntry(category + "_isActive", this, [this]() { return isActive_; });
  logger.addLogEntry(category + "_integralTerm", this, [this]() { return integralTerm_; });
  logger.addLogEntry(category + "_activeJoints", this, [this]() { return activeJoints_; });
  logger.addLogEntry(category + "_tauMomentumObserver", this, [this]() { return tau_momentum_observer_; });
  logger.addLogEntry(category + "_tauExtFtSensor", this, [this]() { return tau_ext_ft_sensor_; });
  logger.addLogEntry(category + "_useFTSensorMeasurements", this, [this]() { return useFTSensorMeasurements_; });
  logger.addLogEntry(category + "_useActiveJointsMask", this, [this]() { return useActiveJointsMask_; });
}

void ExternalForcesObserver::addDatastoreCall( mc_control::MCController & ctl)
{
  ctl.datastore().make_call("EF_Estimator::isActive", [this]() { return isActive_; });
  ctl.datastore().make_call("EF_Estimator::toggleActive", [this]() { isActive_ = !isActive_; });
  ctl.datastore().make_call("EF_Estimator::setGain", [this](double gain) {
    resetMomentumObserver();
    residualGain_ = gain;
  });
  ctl.datastore().make_call("EF_Estimator::getGain", [this]() { return residualGain_; });
  ctl.datastore().make_call("EF_Estimator::isUsingFTSensorMeasurements",
                                   [this]() { return useFTSensorMeasurements_; });
  ctl.datastore().make_call("EF_Estimator::toggleFTSensorMeasurements",
                                   [this]() { useFTSensorMeasurements_ = !useFTSensorMeasurements_; });
  ctl.datastore().make_call("EF_Estimator::isUsingActiveJointsMask",
                                   [this]() { return useActiveJointsMask_; });
  ctl.datastore().make_call("EF_Estimator::toggleActiveJointsMask",
                                   [this]() { useActiveJointsMask_ = !useActiveJointsMask_; });
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

} // namespace mc_external_forces_observer

EXPORT_OBSERVER_MODULE("ExternalForcesObserver", mc_external_forces_observer::ExternalForcesObserver)
