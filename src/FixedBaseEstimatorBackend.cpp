#include "EstimatorMathUtils.h"
#include "ExternalForcesObserver.h"
#include "ObserverGuiUtils.h"

#include <mc_control/MCController.h>


namespace mc_external_forces_observer
{

namespace
{

ExternalForcesObserver::ForceEffectsData computeFixedBaseForceFusion(
    const mc_rbdyn::Robot & robot,
    const mc_rbdyn::Robot & realRobot,
    const rbd::MultiBodyConfig & mbc,
    rbd::Jacobian & jac,
    const std::vector<int> & activeJointIndices,
    int dofNumber,
    const std::string & referenceFrame,
    double dt,
    double residualGain,
    bool useForceSensor,
    const std::string & ftSensorName,
    const Eigen::VectorXd & filteredSensorTorques,
    const Eigen::VectorXd & jointResidual)
{
  ExternalForcesObserver::ForceEffectsData fusion;
  fusion.sensorTorques = Eigen::VectorXd::Zero(dofNumber);
  fusion.filteredSensorTorques = filteredSensorTorques;
  fusion.fusedTorques = Eigen::VectorXd::Zero(dofNumber);
  fusion.filteredPublishedTorques = Eigen::VectorXd::Zero(dofNumber);
  const auto R = robot.bodyPosW(referenceFrame).rotation();

  auto jTranspose = jac.jacobian(robot.mb(), mbc);
  jTranspose.transposeInPlace();
  auto jTransposeActive = mc_external_forces_observer::detail::selectRows(jTranspose, activeJointIndices);
  Eigen::VectorXd residualWrench = jTransposeActive.completeOrthogonalDecomposition().solve(jointResidual);
  fusion.residualWrench = sva::ForceVecd(residualWrench);
  fusion.residualWrench.force() = R * fusion.residualWrench.force();
  fusion.residualWrench.couple() = R * fusion.residualWrench.couple();

  if(useForceSensor && ftSensorName != "none")
  {
    auto sva_EF_FT = realRobot.forceSensor(ftSensorName).wrenchWithoutGravity(realRobot);
    fusion.sensorWrench = sva_EF_FT.vector();
    fusion.fusedWrench.force() = R.transpose() * sva_EF_FT.force();
    fusion.fusedWrench.couple() = R.transpose() * sva_EF_FT.couple();
    fusion.sensorTorques = mc_external_forces_observer::detail::selectEntries(jac.jacobian(robot.mb(), mbc).transpose() * fusion.fusedWrench.vector(),
                                                            activeJointIndices);
    double alpha = 1 - exp(-dt * residualGain);
    fusion.filteredSensorTorques += alpha * (fusion.sensorTorques - fusion.filteredSensorTorques);
    fusion.fusedTorques = jointResidual + (fusion.sensorTorques - fusion.filteredSensorTorques);
    fusion.filteredPublishedTorques = fusion.fusedTorques;
    fusion.filteredSensorWrench =
        sva::ForceVecd(jTransposeActive.completeOrthogonalDecomposition().solve(fusion.filteredSensorTorques));
    fusion.filteredSensorWrench.force() = R * fusion.filteredSensorWrench.force();
    fusion.filteredSensorWrench.couple() = R * fusion.filteredSensorWrench.couple();

    fusion.unfilteredWrench =
        sva::ForceVecd(jTransposeActive.completeOrthogonalDecomposition().solve(fusion.fusedTorques));
    fusion.unfilteredWrench.force() = R * fusion.unfilteredWrench.force();
    fusion.unfilteredWrench.couple() = R * fusion.unfilteredWrench.couple();
    fusion.publishedTorques = mc_external_forces_observer::detail::scatterEntries(fusion.filteredPublishedTorques, activeJointIndices, dofNumber);
  }
  else
  {
    fusion.publishedTorques = mc_external_forces_observer::detail::scatterEntries(jointResidual, activeJointIndices, dofNumber);
  }

  auto activeExternalTorques = mc_external_forces_observer::detail::selectEntries(fusion.publishedTorques, activeJointIndices);
  fusion.fusedWrench = sva::ForceVecd(jTransposeActive.completeOrthogonalDecomposition().solve(activeExternalTorques));
  fusion.fusedWrench.force() = R * fusion.fusedWrench.force();
  fusion.fusedWrench.couple() = R * fusion.fusedWrench.couple();
  return fusion;
}

struct FixedBaseEstimatorBackend final : EstimatorBackend
{
  const char * name() const override { return "FixedBase"; }
  void addToGui(ExternalForcesObserver::EstimatorData & data,
                mc_control::MCController & controller,
                mc_rtc::gui::StateBuilder & gui,
                const std::vector<std::string> & category) override;
  void addToLogger(ExternalForcesObserver::EstimatorData & data,
                   mc_control::MCController & controller) override;

  ExternalForcesObserver::EstimatorResult run(ExternalForcesObserver::EstimatorData & data,
                                               mc_control::MCController & controller) override
  {
    return data.owner->computeForFixedBase(data, controller);
  }
};

} // namespace

std::unique_ptr<EstimatorBackend> makeFixedBaseEstimatorBackend()
{
  return std::make_unique<FixedBaseEstimatorBackend>();
}

void FixedBaseEstimatorBackend::addToGui(ExternalForcesObserver::EstimatorData & data,
                                         mc_control::MCController & controller,
                                         mc_rtc::gui::StateBuilder & gui,
                                         const std::vector<std::string> & category)
{
  addForceVisualizationToGui(data, controller, gui, category);
}

void FixedBaseEstimatorBackend::addToLogger(ExternalForcesObserver::EstimatorData & data,
                                            mc_control::MCController & controller)
{
  auto & logger = controller.logger();
  auto * forceEffects = data.forceEffects;
  auto * residuals = data.residuals;
  auto * speedResidual = data.speedResidual;
  logger.addLogEntry("ExternalForceEstimator_wrench", data.owner,
                     [forceEffects]() { return forceEffects->fusedWrench; });
  logger.addLogEntry("ExternalForceEstimator_non_filtered_wrench", data.owner,
                     [forceEffects]() { return forceEffects->unfilteredWrench; });
  logger.addLogEntry("ExternalForceEstimator_residual_joint_torque", data.owner,
                     [residuals]() { return residuals->jointResidual; });
  logger.addLogEntry("ExternalForceEstimator_external_residual_joint_torque", data.owner,
                     [residuals]() -> Eigen::Vector6d { return residuals->baseResidual; });
  logger.addLogEntry("ExternalForceEstimator_residual_wrench", data.owner,
                     [forceEffects]() { return forceEffects->residualWrench; });
  logger.addLogEntry("ExternalForceEstimator_integralTerm", data.owner,
                     [residuals]() { return residuals->integralJoint; });
  logger.addLogEntry("ExternalForceEstimator_FTSensor_filtered_torque", data.owner,
                     [forceEffects]() { return forceEffects->filteredSensorTorques; });
  logger.addLogEntry("ExternalForceEstimator_FTSensor_filtered_wrench", data.owner,
                     [forceEffects]() { return forceEffects->filteredSensorWrench; });
  logger.addLogEntry("ExternalForceEstimator_FTSensor_torque", data.owner,
                     [forceEffects]() { return forceEffects->sensorTorques; });
  logger.addLogEntry("ExternalForceEstimator_FTSensor_wrench", data.owner,
                     [forceEffects]() { return forceEffects->sensorWrench; });
  logger.addLogEntry("ExternalForceEstimator_non_filtered_torque_value", data.owner,
                     [forceEffects]() { return forceEffects->fusedTorques; });
  logger.addLogEntry("ExternalForceEstimator_torque_value", data.owner,
                     [forceEffects]() { return forceEffects->publishedTorques; });
  logger.addLogEntry("ExternalForceEstimator_residualWithRotorInertia", data.owner,
                     [residuals]() { return residuals->rotorInertiaResidual; });
  logger.addLogEntry("ExternalForceEstimator_residualSpeed", data.owner,
                     [speedResidual]() { return speedResidual->residual; });
}

void ExternalForcesObserver::updateFixedBaseResidualObserver(const Eigen::VectorXd & tauActive,
                                                              const Eigen::VectorXd & qdotActive,
                                                              const Eigen::VectorXd & coriolisGravityTerm,
                                                              const Eigen::MatrixXd & coriolisMatrixActive,
                                                              const Eigen::MatrixXd & inertiaMatrixActive,
                                                              double timestep)
{
  residuals_.integralJoint +=
      (tauActive + coriolisMatrixActive * qdotActive - coriolisGravityTerm + residuals_.jointResidual) * timestep;
  const auto momentum = inertiaMatrixActive * qdotActive;
  residuals_.jointResidual = residualGain * (momentum - residuals_.integralJoint + pzero);
}

void ExternalForcesObserver::updateRotorInertiaResidual(const Eigen::VectorXd & tauActive,
                                                         const Eigen::VectorXd & qdotActive,
                                                         const Eigen::VectorXd & coriolisGravityTerm,
                                                         const Eigen::MatrixXd & coriolisMatrixActive,
                                                         const Eigen::MatrixXd & inertiaMatrixWithRotorInertia,
                                                         double timestep)
{
  const auto momentumWithRotorInertia = inertiaMatrixWithRotorInertia * qdotActive;
  residuals_.rotorInertiaIntegral +=
      (tauActive + coriolisMatrixActive * qdotActive - coriolisGravityTerm + residuals_.rotorInertiaResidual)
      * timestep;
  residuals_.rotorInertiaResidual =
      residualGain * (momentumWithRotorInertia - residuals_.rotorInertiaIntegral + pzero);
}

void ExternalForcesObserver::updateSpeedResidualObserver(const Eigen::VectorXd & tauActive,
                                                          const Eigen::VectorXd & qdotActive,
                                                          const Eigen::VectorXd & coriolisGravityTerm,
                                                          const Eigen::MatrixXd & coriolisMatrixActive,
                                                          const Eigen::MatrixXd & inertiaMatrixActive,
                                                          double timestep)
{
  speedResidual_.integral +=
      (tauActive + coriolisMatrixActive * qdotActive - coriolisGravityTerm + speedResidual_.residual) * timestep;
  const auto momentum = inertiaMatrixActive * qdotActive;
  speedResidual_.residual = residualSpeedGain * (momentum - speedResidual_.integral + pzero);
}

ExternalForcesObserver::EstimatorResult
ExternalForcesObserver::computeForFixedBase(EstimatorData & data, mc_control::MCController & controller)
{
  static_cast<void>(data);
  auto & ctl = static_cast<mc_control::MCController &>(controller);
  auto inputs = buildEstimatorInputs(ctl, 0, true, true);
  updateDiagnostics(inputs);

  const auto inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  const auto inertiaMatrixActive = detail::selectSubmatrix(inertiaMatrix, activeJointIndices);
  const auto qdotActive = detail::selectEntries(inputs.qdot, activeJointIndices);
  const auto tauActive = detail::selectEntries(inputs.tau, activeJointIndices);
  const auto coriolisGravityTerm = detail::selectEntries(forwardDynamics.C(), activeJointIndices);
  const auto coriolisMatrixActive =
      detail::selectSubmatrix(inputs.coriolisMatrix + inputs.coriolisMatrix.transpose(), activeJointIndices);

  updateFixedBaseResidualObserver(tauActive, qdotActive, coriolisGravityTerm, coriolisMatrixActive, inertiaMatrixActive,
                                  ctl.timeStep);
  ctl.datastore().assign("EF_Estimator::getResidualOnly", residuals_.jointResidual);

  const auto inertiaMatrixWithRotorInertia = detail::selectSubmatrix(forwardDynamics.H(), activeJointIndices);
  updateRotorInertiaResidual(tauActive, qdotActive, coriolisGravityTerm, coriolisMatrixActive,
                             inertiaMatrixWithRotorInertia, ctl.timeStep);
  updateSpeedResidualObserver(tauActive, qdotActive, coriolisGravityTerm, coriolisMatrixActive, inertiaMatrixActive,
                              ctl.timeStep);
  updateSpeedResidualDatastore(ctl);

  EstimatorResult result;
  result.preservedPrefix = inputs.preservedPrefix;
  result.warnWhenInactive = inputs.warnWhenInactive;
  result.logPluginState = inputs.logPluginState;
  forceEffects_ = computeFixedBaseForceFusion(*inputs.robot, *inputs.realRobot, inputs.mbc, jac, activeJointIndices,
                                              dofNumber, referenceFrame, dt, residualGain, use_force_sensor_,
                                              ft_sensor_name_, forceEffects_.filteredSensorTorques,
                                              residuals_.jointResidual);
  forceEffects_.sensorForceEstimations.assign(static_cast<size_t>(inputs.robot->forceSensors().size()),
                                              sva::ForceVecd::Zero());
  result.torques = forceEffects_.publishedTorques;
  result.accelerations = forwardDynamics.H().ldlt().solve(result.torques);
  return result;
}

} // namespace mc_external_forces_observer
