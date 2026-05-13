/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ExternalForcesObserver.h"

#include <mc_rtc/logging.h>

#include <algorithm>

namespace mc_external_forces_observer::detail
{

inline Eigen::VectorXd selectEntries(const Eigen::VectorXd & vector, const std::vector<int> & indices)
{
  Eigen::VectorXd out(indices.size());
  for(size_t i = 0; i < indices.size(); ++i)
  {
    out(static_cast<Eigen::Index>(i)) = vector(indices[i]);
  }
  return out;
}

inline Eigen::VectorXd scatterEntries(const Eigen::VectorXd & vector, const std::vector<int> & indices, int fullSize)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(fullSize);
  for(size_t i = 0; i < indices.size(); ++i)
  {
    out(indices[i]) = vector(static_cast<Eigen::Index>(i));
  }
  return out;
}

inline void zeroInactiveEntries(Eigen::VectorXd & vector, const std::vector<int> & activeIndices, int preservedPrefix)
{
  std::vector<bool> active(static_cast<size_t>(vector.size()), false);
  for(int i = 0; i < preservedPrefix && i < vector.size(); ++i)
  {
    active[static_cast<size_t>(i)] = true;
  }
  for(int idx : activeIndices)
  {
    if(0 <= idx && idx < vector.size())
    {
      active[static_cast<size_t>(idx)] = true;
    }
  }
  for(int i = preservedPrefix; i < vector.size(); ++i)
  {
    if(!active[static_cast<size_t>(i)])
    {
      vector(i) = 0.0;
    }
  }
}

inline Eigen::MatrixXd selectRows(const Eigen::MatrixXd & matrix, const std::vector<int> & indices)
{
  Eigen::MatrixXd out(indices.size(), matrix.cols());
  for(size_t i = 0; i < indices.size(); ++i)
  {
    out.row(static_cast<Eigen::Index>(i)) = matrix.row(indices[i]);
  }
  return out;
}

inline Eigen::MatrixXd selectCols(const Eigen::MatrixXd & matrix, const std::vector<int> & indices)
{
  Eigen::MatrixXd out(matrix.rows(), indices.size());
  for(size_t i = 0; i < indices.size(); ++i)
  {
    out.col(static_cast<Eigen::Index>(i)) = matrix.col(indices[i]);
  }
  return out;
}

inline Eigen::MatrixXd selectSubmatrix(const Eigen::MatrixXd & matrix, const std::vector<int> & indices)
{
  Eigen::MatrixXd out(indices.size(), indices.size());
  for(size_t row = 0; row < indices.size(); ++row)
  {
    for(size_t col = 0; col < indices.size(); ++col)
    {
      out(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) = matrix(indices[row], indices[col]);
    }
  }
  return out;
}

inline int jointDofOffset(const rbd::MultiBody & mb, int jointIndex)
{
  int offset = 0;
  for(int i = 0; i < jointIndex; ++i)
  {
    offset += mb.joint(i).dof();
  }
  return offset;
}

inline Eigen::VectorXd mapFullDofByJointName(const mc_rbdyn::Robot & sourceRobot,
                                             const Eigen::VectorXd & raw,
                                             const mc_rbdyn::Robot & targetRobot,
                                             int fullSize)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(fullSize);
  const bool sourceFloating = sourceRobot.mb().nrJoints() > 0 && sourceRobot.mb().joint(0).type() == rbd::Joint::Free;
  const bool targetFloating = targetRobot.mb().nrJoints() > 0 && targetRobot.mb().joint(0).type() == rbd::Joint::Free;
  if(sourceFloating && targetFloating)
  {
    out.head(std::min<int>(6, std::min<int>(raw.size(), fullSize))) =
        raw.head(std::min<int>(6, std::min<int>(raw.size(), fullSize)));
  }
  for(int jIndex = sourceFloating ? 1 : 0; jIndex < sourceRobot.mb().nrJoints(); ++jIndex)
  {
    const auto & sourceJoint = sourceRobot.mb().joint(jIndex);
    if(sourceJoint.dof() != 1 || !targetRobot.hasJoint(sourceJoint.name()))
    {
      continue;
    }
    const auto targetIndex = targetRobot.mb().jointIndexByName(sourceJoint.name());
    const auto & targetJoint = targetRobot.mb().joint(targetIndex);
    if(targetJoint.dof() != 1)
    {
      continue;
    }
    const auto sourceOffset = jointDofOffset(sourceRobot.mb(), jIndex);
    const auto targetOffset = jointDofOffset(targetRobot.mb(), targetIndex);
    if(sourceOffset < raw.size() && targetOffset < fullSize)
    {
      out(targetOffset) = raw(sourceOffset);
    }
  }
  return out;
}

inline Eigen::VectorXd refJointOrderToFullDof(const mc_rbdyn::Robot & sourceRobot,
                                              const Eigen::VectorXd & raw,
                                              const mc_rbdyn::Robot & targetRobot,
                                              int fullSize)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(fullSize);
  for(Eigen::Index i = 0; i < raw.size(); ++i)
  {
    if(i >= static_cast<Eigen::Index>(sourceRobot.refJointOrder().size()))
    {
      continue;
    }
    const auto & jointName = sourceRobot.refJointOrder()[static_cast<size_t>(i)];
    if(!targetRobot.hasJoint(jointName))
    {
      continue;
    }
    const auto jointIndex = targetRobot.mb().jointIndexByName(jointName);
    const auto & joint = targetRobot.mb().joint(jointIndex);
    if(joint.dof() != 1)
    {
      continue;
    }
    out(jointDofOffset(targetRobot.mb(), jointIndex)) = raw(i);
  }
  return out;
}

inline Eigen::VectorXd sanitizeTorqueInput(const mc_rbdyn::Robot & sourceRobot,
                                           const mc_rbdyn::Robot & targetRobot,
                                           const Eigen::VectorXd & raw,
                                           const std::vector<int> & activeIndices,
                                           int fullSize,
                                           int preservedPrefix)
{
  if(raw.size() == fullSize)
  {
    Eigen::VectorXd out = raw;
    zeroInactiveEntries(out, activeIndices, preservedPrefix);
    return out;
  }
  if(raw.size() == static_cast<Eigen::Index>(activeIndices.size()))
  {
    return scatterEntries(raw, activeIndices, fullSize);
  }
  if(raw.size() == static_cast<Eigen::Index>(sourceRobot.mb().nrDof()))
  {
    auto out = mapFullDofByJointName(sourceRobot, raw, targetRobot, fullSize);
    zeroInactiveEntries(out, activeIndices, preservedPrefix);
    return out;
  }
  if(raw.size() == static_cast<Eigen::Index>(sourceRobot.refJointOrder().size()))
  {
    auto out = refJointOrderToFullDof(sourceRobot, raw, targetRobot, fullSize);
    zeroInactiveEntries(out, activeIndices, preservedPrefix);
    return out;
  }
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "[ExternalForcesObserver] Unexpected torque vector size {}, expected {}, {}, {} or {}", raw.size(), fullSize,
      activeIndices.size(), sourceRobot.mb().nrDof(), sourceRobot.refJointOrder().size());
}

} // namespace mc_external_forces_observer::detail
