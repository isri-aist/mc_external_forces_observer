cmake_minimum_required(VERSION 3.5)

project(
  mc_external_forces_observer
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
find_package(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_control)
  message(FATAL_ERROR "mc_rtc::mc_control is required to build this project")
endif() 

# Configure and install the observer configuration
configure_file(etc/ExternalForcesObserver.in.yaml
         "${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalForcesObserver.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalForcesObserver.yaml"
    DESTINATION "${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC
  src/ExternalForcesObserver.cpp
    src/FixedBaseEstimatorBackend.cpp
    src/FloatingBaseFullGeneralizedBackend.cpp
    src/FloatingBaseDecoupledBackend.cpp)
set(plugin_HDR
  src/ExternalForcesObserver.h
    src/EstimatorMathUtils.h)

# The add_observer macro is provided by mc_rtc
add_observer(ExternalForcesObserver ${plugin_SRC} ${plugin_HDR})
target_link_libraries(ExternalForcesObserver PUBLIC mc_rtc::mc_control)
