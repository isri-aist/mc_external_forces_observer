#pragma once

#include "ExternalForcesObserver.h"

#include <mc_control/MCController.h>

namespace mc_external_forces_observer
{

inline void addForceVisualizationToGui(ExternalForcesObserver::EstimatorData & data,
                                       mc_control::MCController & controller,
                                       mc_rtc::gui::StateBuilder & gui,
                                       const std::vector<std::string> & category)
{
  auto * forceEffects = data.forceEffects;
  const auto * referenceFrame = data.referenceFrame;
  auto fConf = mc_rtc::gui::ForceConfig();
  fConf.force_scale = 0.01;

  fConf.color = mc_rtc::gui::Color::Blue;
  gui.addElement(category,
                 mc_rtc::gui::Force(
                     "EndEffector", fConf,
                     [forceEffects]() { return forceEffects->fusedWrench; },
                     [&controller, referenceFrame]()
                     { return controller.robot().bodyPosW(controller.robot().frame(*referenceFrame).body()); }));

  fConf.color = mc_rtc::gui::Color::Yellow;
  gui.addElement(category,
                 mc_rtc::gui::Force(
                     "EndEffector Residual", fConf,
                     [forceEffects]() { return forceEffects->residualWrench; },
                     [&controller, referenceFrame]()
                     { return controller.robot().bodyPosW(controller.robot().frame(*referenceFrame).body()); }));

  fConf.color = mc_rtc::gui::Color::Red;
  gui.addElement(category,
                 mc_rtc::gui::Force(
                     "EndEffector F/T sensor", fConf,
                     [forceEffects]()
                     {
                       const auto & sensor = forceEffects->sensorWrench;
                       return sva::ForceVecd(sensor.segment(0, 3), sensor.segment(3, 3));
                     },
                     [&controller, referenceFrame]()
                     { return controller.robot().bodyPosW(controller.robot().frame(*referenceFrame).body()); }));

  fConf.color = mc_rtc::gui::Color::Blue;
  size_t fsi = 0;
  const auto * sensorEstimations = &forceEffects->sensorForceEstimations;
  for(const auto & sensor : controller.robot().forceSensors())
  {
    gui.addElement(category,
                   mc_rtc::gui::Force(
                       fmt::format("Estimation at {}", sensor.name()), fConf,
                       [sensorEstimations, fsi]() { return (*sensorEstimations)[fsi]; },
                       [&controller, sensor]() { return controller.realRobot().bodyPosW(sensor.parent()); }));
    fsi++;
  }
}

} // namespace mc_external_forces_observer