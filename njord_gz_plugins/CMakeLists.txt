# Njord Gazebo system plugins, built as an ament package inside the Docker
# image (Gazebo Harmonic: gz-sim8, gz-transport13). Each plugin is a shared
# library loaded from SDF by file name, e.g. libNjordPhysics.so.
cmake_minimum_required(VERSION 3.16)
project(njord_gz_plugins)
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
# Command timeout and thrust clamp for the VRX WAM-V thrusters.
add_library(NjordActuatorWatchdog SHARED src/ActuatorWatchdog.cc)
target_compile_features(NjordActuatorWatchdog PUBLIC cxx_std_17)
target_link_libraries(NjordActuatorWatchdog gz-sim8::gz-sim8 gz-transport13::gz-transport13)
install(TARGETS NjordActuatorWatchdog LIBRARY DESTINATION lib)
# Contact heartbeat on /njord/contacts for the evaluator.
add_library(NjordContactMonitor SHARED src/ContactMonitor.cc)
target_compile_features(NjordContactMonitor PUBLIC cxx_std_17)
target_link_libraries(NjordContactMonitor gz-sim8::gz-sim8 gz-transport13::gz-transport13)
install(TARGETS NjordContactMonitor LIBRARY DESTINATION lib)
# Buoyancy, thrusters and wind for the Njord vessel profile.
add_library(NjordPhysics SHARED src/NjordPhysics.cc)
target_compile_features(NjordPhysics PUBLIC cxx_std_17)
target_include_directories(NjordPhysics PRIVATE include)
target_link_libraries(NjordPhysics gz-sim8::gz-sim8 gz-transport13::gz-transport13)
install(TARGETS NjordPhysics LIBRARY DESTINATION lib)
# Gazebo-free analytic test of the header-only hydrostatics.
if(BUILD_TESTING)
  enable_testing()
  add_executable(njord_hydrostatics_test test/hydrostatics_test.cc)
  target_compile_features(njord_hydrostatics_test PUBLIC cxx_std_17)
  target_include_directories(njord_hydrostatics_test PRIVATE include)
  add_test(NAME njord_hydrostatics COMMAND njord_hydrostatics_test)
endif()
ament_package()
