#include "EstimatorMathUtils.h"
#include "ExternalForcesObserver.h"
#include "ObserverGuiUtils.h"

#include <mc_control/MCController.h>


namespace mc_external_forces_observer
{

namespace detail
{
inline void computeForwardDynamicFlacco(const mc_rbdyn::Robot & robot,
                                        const rbd::MultiBodyConfig & mbc,
                                        Eigen::MatrixXd & H,
                                        Eigen::MatrixXd & Hd)
{
  const auto & mb = robot.mb();
  std::vector<sva::RBInertiad> I_st_(static_cast<size_t>(mb.nrBodies()));
  std::vector<Eigen::Matrix6d> Id_st_(static_cast<size_t>(mb.nrBodies()));
  std::vector<Eigen::Matrix6d> Xd_p_vec(static_cast<size_t>(mb.nrBodies()));
  std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> F_(static_cast<size_t>(mb.nrJoints()));
  std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> Fd_(static_cast<size_t>(mb.nrJoints()));
  std::vector<int> dofPos_(static_cast<size_t>(mb.nrJoints()));

  int dofP = 0;
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    F_[ui].resize(6, mb.joint(i).dof());
    Fd_[ui].resize(6, mb.joint(i).dof());
    dofPos_[ui] = dofP;
    dofP += mb.joint(i).dof();
  }

  const std::vector<rbd::Body> & bodies = mb.bodies();
  const std::vector<rbd::Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  H.setZero(mb.nrDof(), mb.nrDof());
  Hd.setZero(mb.nrDof(), mb.nrDof());
  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
    Xd_p_vec[ui] = -sva::vector6ToCrossMatrix(mbc.jointVelocity[ui].vector()) * X_p_i.matrix();
    I_st_[i] = bodies[i].inertia();
    Id_st_[i].setZero();
  }

  for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    if(pred[ui] != -1)
    {
      const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
      Eigen::Matrix6d Xd_p_i = Xd_p_vec[ui];
      I_st_[static_cast<size_t>(pred[ui])] += X_p_i.transMul(I_st_[ui]);
      Id_st_[static_cast<size_t>(pred[ui])] += X_p_i.matrix().transpose() * Id_st_[ui] * X_p_i.matrix()
                                               + Xd_p_i.transpose() * I_st_[ui].matrix() * X_p_i.matrix()
                                               + X_p_i.matrix().transpose() * I_st_[ui].matrix() * Xd_p_i;
    }

    for(int dof = 0; dof < joints[ui].dof(); ++dof)
    {
      F_[ui].col(dof).noalias() = (I_st_[ui] * sva::MotionVecd(mbc.motionSubspace[ui].col(dof))).vector();
      Fd_[ui].col(dof).noalias() = Id_st_[ui] * mbc.motionSubspace[ui].col(dof);
    }

    H.block(dofPos_[ui], dofPos_[ui], joints[ui].dof(), joints[ui].dof()).noalias() =
        mbc.motionSubspace[ui].transpose() * F_[ui];
    Hd.block(dofPos_[ui], dofPos_[ui], joints[ui].dof(), joints[ui].dof()).noalias() =
        mbc.motionSubspace[ui].transpose() * Fd_[ui];

    size_t j = ui;
    while(pred[j] != -1)
    {
      const sva::PTransformd & X_p_j = mbc.parentToSon[j];
      const Eigen::Matrix6d & Xd_p_j = Xd_p_vec[j];
      for(int dof = 0; dof < joints[ui].dof(); ++dof)
      {
        F_[ui].col(dof) = X_p_j.transMul(sva::ForceVecd(F_[ui].col(dof))).vector();
        Fd_[ui].col(dof) =
            X_p_j.transMul(sva::ForceVecd(Fd_[ui].col(dof))).vector() + Xd_p_j.transpose() * F_[ui].col(dof);
      }
      j = static_cast<size_t>(pred[j]);

      if(joints[j].dof() != 0)
      {
        H.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).noalias() =
            F_[ui].transpose() * mbc.motionSubspace[j];
        Hd.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).noalias() =
            Fd_[ui].transpose() * mbc.motionSubspace[j];

        H.block(dofPos_[j], dofPos_[ui], joints[j].dof(), joints[ui].dof()).noalias() =
            H.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).transpose();
        Hd.block(dofPos_[j], dofPos_[ui], joints[j].dof(), joints[ui].dof()).noalias() =
            Hd.block(dofPos_[ui], dofPos_[j], joints[ui].dof(), joints[j].dof()).transpose();
      }
    }
  }

  H.noalias() = H;
}

inline Eigen::VectorXd computeCHatPc0HatFlacco(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc)
{
  const auto & mb = robot.mb();
  Eigen::VectorXd c_hat = Eigen::VectorXd::Zero(mb.nrDof());
  std::vector<sva::MotionVecd> acc_(static_cast<size_t>(mb.nrBodies()));
  std::vector<sva::ForceVecd> f_(static_cast<size_t>(mb.nrBodies()));
  std::vector<int> dofPos_(static_cast<size_t>(mb.nrJoints()));

  int dofP = 0;
  for(int i = 0; i < mb.nrJoints(); ++i)
  {
    const auto ui = static_cast<size_t>(i);
    dofPos_[ui] = dofP;
    dofP += mb.joint(i).dof();
  }

  const std::vector<rbd::Body> & bodies = mb.bodies();
  const std::vector<rbd::Joint> & joints = mb.joints();
  const std::vector<int> & pred = mb.predecessors();

  sva::MotionVecd a_0(Eigen::Vector3d::Zero(), mbc.gravity);

  for(std::size_t i = 0; i < bodies.size(); ++i)
  {
    const sva::PTransformd & X_p_i = mbc.parentToSon[i];

    const sva::MotionVecd & vj_i = mbc.jointVelocity[i];

    const sva::MotionVecd & vb_i = mbc.bodyVelB[i];

    if(pred[i] != -1)
      acc_[i] = X_p_i * acc_[static_cast<size_t>(pred[i])] + vb_i.cross(vj_i);
    else
      acc_[i] = X_p_i * a_0 + vb_i.cross(vj_i);

    f_[i] = bodies[i].inertia() * acc_[i] + vb_i.crossDual(bodies[i].inertia() * vb_i);
  }

  for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i)
  {
    const auto ui = static_cast<size_t>(i);
    c_hat.segment(dofPos_[ui], joints[ui].dof()).noalias() = mbc.motionSubspace[ui].transpose() * f_[ui].vector();

    if(pred[ui] != -1)
    {
      const sva::PTransformd & X_p_i = mbc.parentToSon[ui];
      f_[static_cast<size_t>(pred[ui])] += X_p_i.transMul(f_[ui]);
    }
  }

  return c_hat;
}

inline std::vector<sva::ForceVecd> estimateFloatingBaseSensorWrenches(
    const mc_rbdyn::Robot & robot,
    const mc_rbdyn::Robot & realRobot,
    const rbd::MultiBodyConfig & mbc,
    int actuatedDofNumber,
    int dofNumber,
    const std::vector<int> & activeJointIndices,
    const Eigen::MatrixXd & FT,
    const Eigen::MatrixXd & I_c_0_inv,
    const Eigen::VectorXd & residualFB)
{
  std::vector<sva::ForceVecd> estimations(static_cast<size_t>(realRobot.forceSensors().size()), sva::ForceVecd::Zero());
  Eigen::MatrixXd augmented_Jfb_forces(6 + actuatedDofNumber, 6);

  size_t fsi = 0;
  for(auto & sensor : realRobot.forceSensors())
  {
    rbd::Jacobian jacobian_forces = rbd::Jacobian(robot.mb(), sensor.parentBody());
    Eigen::MatrixXd Jac_forces = jacobian_forces.jacobian(robot.mb(), mbc, realRobot.posW());
    Eigen::MatrixXd fullJac_forces(6, dofNumber);
    jacobian_forces.fullJacobian(robot.mb(), Jac_forces, fullJac_forces);
    Eigen::MatrixXd Jfb_forces_T = selectCols(fullJac_forces, activeJointIndices).transpose() - FT * I_c_0_inv;
    augmented_Jfb_forces.block(0, 0, 6, 6).setIdentity();
    augmented_Jfb_forces.block(6, 0, actuatedDofNumber, 6) = Jfb_forces_T;
    Eigen::VectorXd estimated_wrench_fb = augmented_Jfb_forces.completeOrthogonalDecomposition().solve(residualFB);
    Eigen::VectorXd estimated_wrench = realRobot.posW().matrix().transpose() * estimated_wrench_fb;
    estimations[fsi] = sva::ForceVecd(estimated_wrench);
    fsi++;
  }

  return estimations;
}

} // namespace detail

namespace
{

struct FloatingBaseDecoupledBackend final : EstimatorBackend
{
  const char * name() const override { return "FloatingBaseDecoupled"; }
  void addToGui(ExternalForcesObserver::EstimatorData & data,
                mc_control::MCController & controller,
                mc_rtc::gui::StateBuilder & gui,
                const std::vector<std::string> & category) override;
  void addToLogger(ExternalForcesObserver::EstimatorData & data,
                   mc_control::MCController & controller) override;

  ExternalForcesObserver::EstimatorResult run(ExternalForcesObserver::EstimatorData & data,
                                               mc_control::MCController & controller) override
  {
    return data.owner->computeForFloatingBaseDecoupled(data, controller);
  }
};

} // namespace

std::unique_ptr<EstimatorBackend> makeFloatingBaseDecoupledBackend()
{
  return std::make_unique<FloatingBaseDecoupledBackend>();
}

void FloatingBaseDecoupledBackend::addToGui(ExternalForcesObserver::EstimatorData & data,
                                            mc_control::MCController & controller,
                                            mc_rtc::gui::StateBuilder & gui,
                                            const std::vector<std::string> & category)
{
  addForceVisualizationToGui(data, controller, gui, category);
}

void FloatingBaseDecoupledBackend::addToLogger(ExternalForcesObserver::EstimatorData & data,
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

ExternalForcesObserver::FloatingBaseCouplingTerms
ExternalForcesObserver::computeFloatingBaseCouplingTerms(const Eigen::VectorXd & coriolisGravityTerm,
                                                          const Eigen::MatrixXd & inertiaMatrix,
                                                          const Eigen::MatrixXd & inertiaRateMatrix) const
{
  FloatingBaseCouplingTerms terms;
  terms.F = detail::selectCols(inertiaMatrix.topRows(6), activeJointIndices);
  terms.FT = terms.F.transpose();
  terms.Ic0 = inertiaMatrix.topLeftCorner(6, 6);
  terms.I_c_0_inv = terms.Ic0.inverse();

  const auto Hsub = detail::selectSubmatrix(inertiaMatrix, activeJointIndices);
  const auto Fd = detail::selectCols(inertiaRateMatrix.topRows(6), activeJointIndices);
  const auto FdT = Fd.transpose();
  const auto I_c_0d = inertiaRateMatrix.topLeftCorner(6, 6);
  const auto Hdsub = detail::selectSubmatrix(inertiaRateMatrix, activeJointIndices);

  terms.Hfb = Hsub - terms.FT * terms.I_c_0_inv * terms.F;
  terms.Cfb =
      detail::selectEntries(coriolisGravityTerm, activeJointIndices) - terms.FT * terms.I_c_0_inv * coriolisGravityTerm.head(6);
  terms.Hfbd = Hdsub - FdT * terms.I_c_0_inv * terms.F - terms.FT * terms.I_c_0_inv * Fd
               - terms.FT * (-terms.I_c_0_inv * I_c_0d * terms.I_c_0_inv) * terms.F;
  return terms;
}

void ExternalForcesObserver::updateDecoupledResidualObservers(const FloatingBaseCouplingTerms & couplingTerms,
                                                               const Eigen::VectorXd & tau,
                                                               const Eigen::VectorXd & tauJoint,
                                                               const Eigen::VectorXd & qdot,
                                                               const Eigen::VectorXd & qdotBase,
                                                               const Eigen::VectorXd & qdotJoint,
                                                               const Eigen::VectorXd & coriolisGravityTerm,
                                                               double timestep)
{
  residuals_.integralFull += (tau + Hd * qdot - coriolisGravityTerm + residuals_.residualFull) * timestep;
  residuals_.residualFull = residualGain * (H * qdot - residuals_.integralFull);

  residuals_.integralJoint +=
      (tauJoint + couplingTerms.Hfbd * qdotJoint - couplingTerms.Cfb + residuals_.jointResidual) * timestep;
  residuals_.jointResidual = residualGain * (couplingTerms.Hfb * qdotJoint - residuals_.integralJoint);

  const auto inertiaRateMatrix = Hd.topLeftCorner(6, 6);
  const auto forceRateMatrix = detail::selectCols(Hd.topRows(6), activeJointIndices);
  residuals_.integralBase +=
      (inertiaRateMatrix * qdotBase + forceRateMatrix * qdotJoint - coriolisGravityTerm.head(6) + residuals_.baseResidual)
      * timestep;
  residuals_.baseResidual =
      residualGain * (couplingTerms.Ic0 * qdotBase + couplingTerms.F * qdotJoint - residuals_.integralBase);
}

ExternalForcesObserver::EstimatorResult
ExternalForcesObserver::computeForFloatingBaseDecoupled(EstimatorData & data,
                                                         mc_control::MCController & controller)
{
  static_cast<void>(data);
  auto & ctl = static_cast<mc_control::MCController &>(controller);
  auto inputs = buildEstimatorInputs(ctl, 6, false, false);
  updateDiagnostics(inputs);

  const auto tauJoint = detail::selectEntries(inputs.tau, activeJointIndices);
  const auto qdotBase = inputs.qdot.head(6);
  const auto qdotJoint = detail::selectEntries(inputs.qdot, activeJointIndices);

  Eigen::VectorXd coriolisGravityTerm;

  if(forward_dynamics_mode_ == ForwardDynamicsMode::Flacco)
  {
    detail::computeForwardDynamicFlacco(*inputs.robot, inputs.mbc, H, Hd);
  }
  else
  {
    H = forwardDynamics.H() - forwardDynamics.HIr();
    Hd = inputs.coriolisMatrix + inputs.coriolisMatrix.transpose();
  }

  if(bias_term_mode_ == BiasTermMode::Flacco)
  {
    coriolisGravityTerm = detail::computeCHatPc0HatFlacco(*inputs.robot, inputs.mbc);
  }
  else
  {
    coriolisGravityTerm = forwardDynamics.C();
  }

  const auto couplingTerms = computeFloatingBaseCouplingTerms(coriolisGravityTerm, H, Hd);

  updateDecoupledResidualObservers(couplingTerms, inputs.tau, tauJoint, inputs.qdot, qdotBase, qdotJoint,
                                   coriolisGravityTerm, ctl.timeStep);

  Eigen::VectorXd residualFB(6 + actuatedDofNumber);
  residualFB.head(6) = residuals_.baseResidual;
  residualFB.tail(actuatedDofNumber) = residuals_.jointResidual;

  Eigen::VectorXd residual = Eigen::VectorXd::Zero(dofNumber);
  residual.head(6) = residuals_.baseResidual;
  residual += detail::scatterEntries(residuals_.jointResidual
                                         + couplingTerms.FT * couplingTerms.I_c_0_inv * residuals_.baseResidual,
                                     activeJointIndices, dofNumber);

  EstimatorResult result;
  result.preservedPrefix = inputs.preservedPrefix;
  result.warnWhenInactive = inputs.warnWhenInactive;
  result.logPluginState = inputs.logPluginState;
  forceEffects_.sensorForceEstimations =
      detail::estimateFloatingBaseSensorWrenches(*inputs.robot, *inputs.realRobot, inputs.mbc, actuatedDofNumber,
                                                 dofNumber, activeJointIndices, couplingTerms.FT,
                                                 couplingTerms.I_c_0_inv, residualFB);
  result.torques = residual;
  result.accelerations = H.ldlt().solve(result.torques);
  forceEffects_.sensorTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  forceEffects_.filteredSensorTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  forceEffects_.fusedTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  forceEffects_.filteredPublishedTorques = Eigen::VectorXd::Zero(actuatedDofNumber);
  forceEffects_.publishedTorques = result.torques;
  return result;
}

} // namespace mc_external_forces_observer
