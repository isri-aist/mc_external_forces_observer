cmake_minimum_required(VERSION 3.22)

project(
  mc_external_forces_observer
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
find_package(mc_rtc REQUIRED)

set(observer_SRC
    src/ExternalForcesObserver.cpp)
set(observer_HDR
    src/ExternalForcesObserver.h)

# The add_observer macro is provided by mc_rtc
add_observer(ExternalForcesObserver ${observer_SRC} ${observer_HDR})