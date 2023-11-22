cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME ObjectTrackingToss)
set(PROJECT_DESCRIPTION "ObjectTrackingToss")
set(PROJECT_URL "")

project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)
find_package(mc_iam REQUIRED)

set(SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/toss_scene")
set(MESHES_DESTINATION "${SHARE_DESTINATION}/meshes")
set(MODULES_DESTINATION "${SHARE_DESTINATION}/modules")
set(URDF_DESTINATION "${SHARE_DESTINATION}/urdf")

macro(install_aliase ROBOT)
  set(ALIASE_OUT "${CMAKE_CURRENT_BINARY_DIR}/aliases/${ROBOT}.yaml")
  configure_file(aliases/${ROBOT}.in.yaml "${ALIASE_OUT}")
  install(FILES "${ALIASE_OUT}" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")
endmacro()

# VANDERLANDE TEST BENCH INSTALLATION
# Setup blender test model
#install(FILES urdf/vi_testbench.urdf DESTINATION "${URDF_DESTINATION}")
set(URDF_TESTBENCH "${CMAKE_CURRENT_BINARY_DIR}/urdf/vi_testbench.urdf")
configure_file(urdf/vi_testbench.urdf "${URDF_TESTBENCH}" @ONLY)
install(FILES "${URDF_TESTBENCH}" DESTINATION "${URDF_DESTINATION}")
install_aliase(vi_testbench)

# Install surfaces description
install(DIRECTORY rsdf DESTINATION "${SHARE_DESTINATION}" FILES_MATCHING PATTERN "*.rsdf")

# Install convex
install(DIRECTORY convex DESTINATION "${SHARE_DESTINATION}" FILES_MATCHING PATTERN "*-ch.txt")

# Install meshes
install(DIRECTORY meshes DESTINATION "${SHARE_DESTINATION}")
#install(DIRECTORY meshes DESTINATION "${SHARE_DESTINATION}" FILES_MATCHING PATTERN "*.dae")

# BOXES INSTALLATION
install(FILES urdf/box1.urdf DESTINATION "${URDF_DESTINATION}")
install(FILES urdf/box2.urdf DESTINATION "${URDF_DESTINATION}")
install(FILES urdf/box3.urdf DESTINATION "${URDF_DESTINATION}")
install(FILES urdf/box4.urdf DESTINATION "${URDF_DESTINATION}")
install(FILES urdf/box5.urdf DESTINATION "${URDF_DESTINATION}")
install(FILES urdf/conveyor_box.urdf DESTINATION "${URDF_DESTINATION}")

install_aliase(toss_box1)
install_aliase(toss_box2)
install_aliase(toss_box3)
install_aliase(toss_box4)
install_aliase(toss_box5)
install_aliase(conveyor_box)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ObjectTrackingToss.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ObjectTrackingToss.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ObjectTrackingToss.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

