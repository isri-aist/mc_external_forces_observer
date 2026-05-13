#include "EstimatorMathUtils.h"
#include "ExternalForcesObserver.h"
#include "ObserverGuiUtils.h"

#include <mc_control/MCController.h>


namespace mc_external_forces_observer
{

namespace
{

ExternalForcesObserver::ForceEffectsData computeFullGeneralizedForceFusion(
    const mc_rbdyn::Robot & robot,
    const rbd::MultiBodyConfig & mbc,
    rbd::Jacobian & jac,
    const std::vector<int> & activeJointIndices,
    int actuatedDofNumber,
    const std::string & referenceFrame,
    const Eigen::VectorXd & activeResidual)
{
  ExternalForcesObserver::ForceEffectsData fusion;
  fusion.sensorTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  fusion.filteredSensorTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  const auto R = robot.bodyPosW(robot.frame(referenceFrame).body()).rotation();

  auto jTranspose = jac.jacobian(robot.mb(), mbc);
  jTranspose.transposeInPlace();
  auto jTransposeActive = mc_external_forces_observer::detail::selectRows(jTranspose, activeJointIndices);
  fusion.residualWrench = sva::ForceVecd(jTransposeActive.completeOrthogonalDecomposition().solve(activeResidual));
  fusion.residualWrench.force() = R * fusion.residualWrench.force();
  fusion.residualWrench.couple() = R * fusion.residualWrench.couple();
  fusion.fusedWrench = fusion.residualWrench;
  fusion.filteredPublishedTorques = activeResidual;
  fusion.fusedTorques = activeResidual;
  fusion.unfilteredWrench = fusion.residualWrench;
  return fusion;
}

struct FloatingBaseFullGeneralizedBackend final : EstimatorBackend
{
  const char * name() const override { return "FloatingBaseFullGeneralized"; }
  void addToGui(ExternalForcesObserver::EstimatorData & data,
                mc_control::MCController & controller,
                mc_rtc::gui::StateBuilder & gui,
                const std::vector<std::string> & category) override;
  void addToLogger(ExternalForcesObserver::EstimatorData & data,
                   mc_control::MCController & controller) override;

  ExternalForcesObserver::EstimatorResult run(ExternalForcesObserver::EstimatorData & data,
                                               mc_control::MCController & controller) override
  {
    return data.owner->computeForFloatingBaseFullGeneralized(data, controller);
  }
};

} // namespace

std::unique_ptr<EstimatorBackend> makeFloatingBaseFullGeneralizedBackend()
{
  return std::make_unique<FloatingBaseFullGeneralizedBackend>();
}

void FloatingBaseFullGeneralizedBackend::addToGui(ExternalForcesObserver::EstimatorData & data,
                                                  mc_control::MCController & controller,
                                                  mc_rtc::gui::StateBuilder & gui,
                                                  const std::vector<std::string> & category)
{
  addForceVisualizationToGui(data, controller, gui, category);
}

void FloatingBaseFullGeneralizedBackend::addToLogger(ExternalForcesObserver::EstimatorData & data,
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
                     [residuals]() { return residuals->integralFull; });
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
  logger.addLogEntry("ExternalForceEstimator_residualSpeed", data.owner,
                     [speedResidual]() { return speedResidual->residual; });
}

void ExternalForcesObserver::updateFullGeneralizedResidualObserver(const Eigen::VectorXd & tau,
                                                                    const Eigen::VectorXd & qdot,
                                                                    const Eigen::VectorXd & coriolisGravityTerm,
                                                                    const Eigen::MatrixXd & coriolisMatrix,
                                                                    const Eigen::MatrixXd & inertiaMatrix,
                                                                    double timestep)
{
  residuals_.integralFull +=
      (tau + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm + residuals_.residualFull)
      * timestep;
  residuals_.residualFull = residualGain * (inertiaMatrix * qdot - residuals_.integralFull);
}

ExternalForcesObserver::EstimatorResult
ExternalForcesObserver::computeForFloatingBaseFullGeneralized(EstimatorData & data,
                                                                mc_control::MCController & controller)
{
  static_cast<void>(data);
  auto & ctl = static_cast<mc_control::MCController &>(controller);
  auto inputs = buildEstimatorInputs(ctl, 6, false, false);
  updateDiagnostics(inputs);

  const auto inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  const auto coriolisGravityTerm = forwardDynamics.C();

  updateFullGeneralizedResidualObserver(inputs.tau, inputs.qdot, coriolisGravityTerm, inputs.coriolisMatrix,
                                        inertiaMatrix, ctl.timeStep);
  const auto activeResidual = detail::selectEntries(residuals_.residualFull, activeJointIndices);

  EstimatorResult result;
  result.preservedPrefix = inputs.preservedPrefix;
  result.warnWhenInactive = inputs.warnWhenInactive;
  result.logPluginState = inputs.logPluginState;
  forceEffects_ = computeFullGeneralizedForceFusion(*inputs.robot, inputs.mbc, jac, activeJointIndices,
                                                    actuatedDofNumber, referenceFrame, activeResidual);
  forceEffects_.sensorForceEstimations.assign(static_cast<size_t>(inputs.robot->forceSensors().size()),
                                              sva::ForceVecd::Zero());
  result.torques = residuals_.residualFull;
  result.accelerations = inertiaMatrix.ldlt().solve(result.torques);
  forceEffects_.publishedTorques = result.torques;
  return result;
}

} // namespace mc_external_forces_observer
