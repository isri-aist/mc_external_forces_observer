/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_rtc/log/FlatLog.h>

#include <RBDyn/Coriolis.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <Eigen/src/Core/Matrix.h>

#include <memory>
#include <string>
#include <vector>

#include <mc_tvm/Robot.h>

enum class TorqueSourceType
{
  CommandedTorque,
  CurrentMeasurement,
  MotorTorqueMeasurement,
  JointTorqueMeasurement,
};

enum class FloatingBaseMode
{
  FullGeneralized,
  Decoupled,
};

enum class ForwardDynamicsMode
{
  Classical,
  Flacco,
};

enum class BiasTermMode
{
  Classical,
  Flacco,
};

namespace mc_external_forces_observer
{

struct EstimatorBackend;

struct ObserverData
{
  std::unique_ptr<EstimatorBackend> backend;
  std::vector<int> activeJointIndices;
  int actuatedDofNumber = 0;

  bool robotIsFloatingBase = false;
  int dofNumber = 0;
  int counter = 0;
  double dt = 0.0;
  bool verbose = false;
  bool isActive = true;

  double residualGain = 0.0;
  std::string referenceFrame;

  // Used for collision avoidance observer, not for the control
  double residualSpeedGain = 0.0;

  // Force sensor
  bool use_force_sensor_ = false;
  TorqueSourceType tau_mes_src_ = TorqueSourceType::JointTorqueMeasurement;
  FloatingBaseMode floating_base_mode_ = FloatingBaseMode::Decoupled;
  ForwardDynamicsMode forward_dynamics_mode_ = ForwardDynamicsMode::Classical;
  BiasTermMode bias_term_mode_ = BiasTermMode::Classical;

  std::string ft_sensor_name_;
};

struct ExternalForcesObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ExternalForcesObserver(const std::string & type, double dt);
  ~ExternalForcesObserver() override;

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;
  void reset(const mc_control::MCController & ctl) override;
  bool run(const mc_control::MCController & ctl) override;
  void update(mc_control::MCController & ctl) override;

protected:
  void addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger, const std::string & category) override;
  void removeFromLogger(mc_rtc::Logger & logger, const std::string & category) override;
  void addToGUI(const mc_control::MCController & ctl,
                mc_rtc::gui::StateBuilder & gui,
                const std::vector<std::string> & category) override;

public:
  struct ForceEffectsData
  {
    Eigen::VectorXd sensorTorques;
    Eigen::VectorXd filteredSensorTorques;
    Eigen::VectorXd fusedTorques;
    Eigen::VectorXd publishedTorques;
    Eigen::VectorXd filteredPublishedTorques;
    sva::ForceVecd fusedWrench = sva::ForceVecd::Zero();
    sva::ForceVecd residualWrench = sva::ForceVecd::Zero();
    sva::ForceVecd unfilteredWrench = sva::ForceVecd::Zero();
    sva::ForceVecd filteredSensorWrench = sva::ForceVecd::Zero();
    Eigen::Vector6d sensorWrench = Eigen::Vector6d::Zero();
    std::vector<sva::ForceVecd> sensorForceEstimations;
  };

  struct SpeedResidualData
  {
    Eigen::VectorXd residual;
    Eigen::VectorXd integral;
  };

  struct ResidualData
  {
    Eigen::VectorXd integralFull;
    Eigen::VectorXd residualFull;
    Eigen::VectorXd integralJoint;
    Eigen::VectorXd jointResidual;
    Eigen::VectorXd integralBase;
    Eigen::VectorXd baseResidual;
    Eigen::VectorXd rotorInertiaResidual;
    Eigen::VectorXd rotorInertiaIntegral;
  };

  struct RuntimeDiagnostics
  {
    Eigen::VectorXd alphas;
    Eigen::VectorXd gravity;
    Eigen::VectorXd inputTorque;
    Eigen::VectorXd commandedAcceleration;
  };

  struct EstimatorInputs
  {
    mc_control::MCController * controller = nullptr;
    const mc_rbdyn::Robot * robot = nullptr;
    const mc_rbdyn::Robot * realRobot = nullptr;
    rbd::MultiBodyConfig mbc;
    Eigen::VectorXd qdot;
    Eigen::VectorXd tau;
    Eigen::VectorXd commandedAcceleration;
    Eigen::VectorXd gravity;
    Eigen::MatrixXd coriolisMatrix;
    int preservedPrefix = 0;
    bool warnWhenInactive = false;
    bool logPluginState = false;
  };

  struct EstimatorResult
  {
    Eigen::VectorXd torques;
    Eigen::VectorXd accelerations;
    int preservedPrefix = 0;
    bool warnWhenInactive = false;
    bool logPluginState = false;
  };

  struct EstimatorData
  {
    ExternalForcesObserver * owner = nullptr;
    ResidualData * residuals = nullptr;
    ForceEffectsData * forceEffects = nullptr;
    SpeedResidualData * speedResidual = nullptr;
    RuntimeDiagnostics * diagnostics = nullptr;
    rbd::Jacobian * jac = nullptr;
    rbd::ForwardDynamics * forwardDynamics = nullptr;
    Eigen::VectorXd * pzero = nullptr;
    std::vector<int> * activeJointIndices = nullptr;
    int * actuatedDofNumber = nullptr;
    int * dofNumber = nullptr;
    int * counter = nullptr;
    double * dt = nullptr;
    bool * robotIsFloatingBase = nullptr;
    bool * verbose = nullptr;
    bool * isActive = nullptr;
    double * residualGain = nullptr;
    std::string * referenceFrame = nullptr;
    double * residualSpeedGain = nullptr;
    bool * use_force_sensor_ = nullptr;
    TorqueSourceType * tau_mes_src_ = nullptr;
    FloatingBaseMode * floating_base_mode_ = nullptr;
    ForwardDynamicsMode * forward_dynamics_mode_ = nullptr;
    BiasTermMode * bias_term_mode_ = nullptr;
    std::string * ft_sensor_name_ = nullptr;
  };

  EstimatorData estimatorData()
  {
    return EstimatorData{this, &residuals_,        &forceEffects_,          &speedResidual_,    &diagnostics_,
                         &jac,  &forwardDynamics,   &pzero,                  &activeJointIndices, &actuatedDofNumber,
                         &dofNumber, &counter,       &dt,                     &robotIsFloatingBase,
                         &verbose, &isActive,        &residualGain,           &referenceFrame,    &residualSpeedGain,
                         &use_force_sensor_,         &tau_mes_src_,           &floating_base_mode_,
                         &forward_dynamics_mode_,    &bias_term_mode_,        &ft_sensor_name_};
  }

  EstimatorResult computeForFixedBase(EstimatorData & data, mc_control::MCController & controller);
  EstimatorResult computeForFloatingBaseFullGeneralized(EstimatorData & data, mc_control::MCController & controller);
  EstimatorResult computeForFloatingBaseDecoupled(EstimatorData & data, mc_control::MCController & controller);

  const ResidualData & residualData() const { return residuals_; }
  const ForceEffectsData & forceEffectsData() const { return forceEffects_; }
  const SpeedResidualData & speedResidualData() const { return speedResidual_; }
  const ResidualData & residualObserverState() const { return residuals_; }
  const ForceEffectsData & forceFusionState() const { return forceEffects_; }
  const SpeedResidualData & speedObserverState() const { return speedResidual_; }
  const RuntimeDiagnostics & diagnostics() const { return diagnostics_; }
  const std::vector<sva::ForceVecd> & forceSensorEstimations() const { return forceEffects_.sensorForceEstimations; }
  const std::string & referenceFrameName() const { return referenceFrame; }
  double gain() const { return residualGain; }
  double residualSpeedGainValue() const { return residualSpeedGain; }
  bool isEstimatorActive() const { return isActive; }
  bool useForceSensor() const { return use_force_sensor_; }
  int numberOfDofs() const { return dofNumber; }
  bool floatingBaseRobot() const { return robotIsFloatingBase; }
  ForwardDynamicsMode forwardDynamicsMode() const { return forward_dynamics_mode_; }
  BiasTermMode biasTermMode() const { return bias_term_mode_; }

private:
  void initializeActiveJoints(const mc_rbdyn::Robot & robot);
  void loadConfiguration(const mc_rtc::Configuration & config);
  void initializeEstimatorState(const mc_rbdyn::Robot & robot, const Eigen::VectorXd & qdot);
  EstimatorInputs buildEstimatorInputs(mc_control::MCController & controller,
                                       int preservedPrefix,
                                       bool warnWhenInactive,
                                       bool logPluginState);
  void updateFixedBaseResidualObserver(const Eigen::VectorXd & tauActive,
                                       const Eigen::VectorXd & qdotActive,
                                       const Eigen::VectorXd & coriolisGravityTerm,
                                       const Eigen::MatrixXd & coriolisMatrixActive,
                                       const Eigen::MatrixXd & inertiaMatrixActive,
                                       double timestep);
  void updateRotorInertiaResidual(const Eigen::VectorXd & tauActive,
                                  const Eigen::VectorXd & qdotActive,
                                  const Eigen::VectorXd & coriolisGravityTerm,
                                  const Eigen::MatrixXd & coriolisMatrixActive,
                                  const Eigen::MatrixXd & inertiaMatrixWithRotorInertia,
                                  double timestep);
  void updateSpeedResidualObserver(const Eigen::VectorXd & tauActive,
                                   const Eigen::VectorXd & qdotActive,
                                   const Eigen::VectorXd & coriolisGravityTerm,
                                   const Eigen::MatrixXd & coriolisMatrixActive,
                                   const Eigen::MatrixXd & inertiaMatrixActive,
                                   double timestep);
  void updateFullGeneralizedResidualObserver(const Eigen::VectorXd & tau,
                                             const Eigen::VectorXd & qdot,
                                             const Eigen::VectorXd & coriolisGravityTerm,
                                             const Eigen::MatrixXd & coriolisMatrix,
                                             const Eigen::MatrixXd & inertiaMatrix,
                                             double timestep);
  struct FloatingBaseCouplingTerms
  {
    Eigen::MatrixXd F;
    Eigen::MatrixXd FT;
    Eigen::MatrixXd Ic0;
    Eigen::MatrixXd I_c_0_inv;
    Eigen::MatrixXd Hfb;
    Eigen::MatrixXd Hfbd;
    Eigen::VectorXd Cfb;
  };
  FloatingBaseCouplingTerms computeFloatingBaseCouplingTerms(const Eigen::VectorXd & coriolisGravityTerm,
                                                             const Eigen::MatrixXd & inertiaMatrix,
                                                             const Eigen::MatrixXd & inertiaRateMatrix) const;
  void updateDecoupledResidualObservers(const FloatingBaseCouplingTerms & couplingTerms,
                                        const Eigen::VectorXd & tau,
                                        const Eigen::VectorXd & tauJoint,
                                        const Eigen::VectorXd & qdot,
                                        const Eigen::VectorXd & qdotBase,
                                        const Eigen::VectorXd & qdotJoint,
                                        const Eigen::VectorXd & coriolisGravityTerm,
                                        double timestep);
  rbd::MultiBodyConfig prepareRuntimeInputs(const mc_rbdyn::Robot & robot,
                                            const mc_rbdyn::Robot & realRobot,
                                            int preservedPrefix,
                                            Eigen::VectorXd & qdot,
                                            Eigen::VectorXd & tau);
  void updateDiagnostics(const EstimatorInputs & inputs);
  Eigen::VectorXd readMeasuredTorque(const mc_rbdyn::Robot & robot,
                                     const mc_rbdyn::Robot & realRobot,
                                     int preservedPrefix) const;
  bool updatePluginActivation(mc_control::MCController & controller) const;
  void updateSpeedResidualDatastore(mc_control::MCController & controller);
  void updateRobotExternalForces(mc_control::MCController & controller,
                                 const mc_rbdyn::Robot & robot,
                                 const mc_rbdyn::Robot & realRobot,
                                 const Eigen::VectorXd & torques,
                                 const Eigen::VectorXd & accelerations);
  void clearRobotExternalForces(mc_control::MCController & controller,
                                const mc_rbdyn::Robot & realRobot) const;
  void resolveAndUpdateRobot(mc_control::MCController & controller,
                             const mc_rbdyn::Robot & robot,
                             const mc_rbdyn::Robot & realRobot,
                             Eigen::VectorXd torques,
                             Eigen::VectorXd accelerations,
                             int preservedPrefix,
                             bool warnWhenInactive,
                             bool logPluginState);
  void resetResidualGain(double gain);

  struct PendingOutput
  {
    bool valid = false;
    Eigen::VectorXd torques;
    Eigen::VectorXd accelerations;
    int preservedPrefix = 0;
    bool warnWhenInactive = false;
    bool logPluginState = false;
  };

  ObserverData observerData_;
  std::unique_ptr<EstimatorBackend> & backend_ = observerData_.backend;
  std::vector<int> & activeJointIndices = observerData_.activeJointIndices;
  int & actuatedDofNumber = observerData_.actuatedDofNumber;

  bool & robotIsFloatingBase = observerData_.robotIsFloatingBase;
  int & dofNumber = observerData_.dofNumber;
  int & counter = observerData_.counter;
  double & dt = observerData_.dt;
  bool & verbose = observerData_.verbose;
  bool & isActive = observerData_.isActive;

  double & residualGain = observerData_.residualGain;
  std::string & referenceFrame = observerData_.referenceFrame;

  rbd::Jacobian jac;
  std::unique_ptr<rbd::Coriolis> coriolis;
  rbd::ForwardDynamics forwardDynamics;

  Eigen::VectorXd pzero;
  ResidualData residuals_;
  ForceEffectsData forceEffects_;

  SpeedResidualData speedResidual_;
  double & residualSpeedGain = observerData_.residualSpeedGain;

  bool & use_force_sensor_ = observerData_.use_force_sensor_;
  TorqueSourceType & tau_mes_src_ = observerData_.tau_mes_src_;
  FloatingBaseMode & floating_base_mode_ = observerData_.floating_base_mode_;
  ForwardDynamicsMode & forward_dynamics_mode_ = observerData_.forward_dynamics_mode_;
  BiasTermMode & bias_term_mode_ = observerData_.bias_term_mode_;

  std::string & ft_sensor_name_ = observerData_.ft_sensor_name_;

  Eigen::MatrixXd H;
  Eigen::MatrixXd F;
  Eigen::MatrixXd Ic0;
  Eigen::MatrixXd Hd;
  Eigen::MatrixXd Fd;
  Eigen::MatrixXd Ic0d;
  RuntimeDiagnostics diagnostics_;

  PendingOutput pending_;
};

struct EstimatorBackend
{
  virtual ~EstimatorBackend() = default;
  virtual const char * name() const = 0;
  virtual ExternalForcesObserver::EstimatorResult run(ExternalForcesObserver::EstimatorData & data,
                                                      mc_control::MCController & controller) = 0;
  virtual void addToGui(ExternalForcesObserver::EstimatorData & data,
                        mc_control::MCController & controller,
                        mc_rtc::gui::StateBuilder & gui,
                        const std::vector<std::string> & category) = 0;
  virtual void addToLogger(ExternalForcesObserver::EstimatorData & data,
                           mc_control::MCController & controller) = 0;
};

std::unique_ptr<EstimatorBackend> makeFixedBaseEstimatorBackend();
std::unique_ptr<EstimatorBackend> makeFloatingBaseFullGeneralizedBackend();
std::unique_ptr<EstimatorBackend> makeFloatingBaseDecoupledBackend();

} // namespace mc_external_forces_observer
