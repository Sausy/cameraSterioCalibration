# Install script for directory: /home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sausy/Projects/lighthouse/cameraSterioCalibration/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs/msg" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/Abortion.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/BodyPart.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/COM.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/ControllerParameters.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/ControllerType.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/ExternalForce.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/ForceTorque.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/IMU.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/Input.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/Joint.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/JointState.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/LegState.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/Model.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/MotorControl.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/RecordingControl.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/Tendon.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/TendonInitialization.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/TendonUpdate.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/msg/VisualizationControl.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs/srv" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/srv/UpdateControllerParameters.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/srv/Energies.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/srv/GymStep.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/srv/GymReset.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/srv/GymGoal.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs/cmake" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_simulation_msgs/catkin_generated/installspace/roboy_simulation_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/include/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roseus/ros/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/common-lisp/ros/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/gennodejs/ros/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/lib/python2.7/dist-packages/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/lib/python2.7/dist-packages/roboy_simulation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_simulation_msgs/catkin_generated/installspace/roboy_simulation_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs/cmake" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_simulation_msgs/catkin_generated/installspace/roboy_simulation_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs/cmake" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_simulation_msgs/catkin_generated/installspace/roboy_simulation_msgsConfig.cmake"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_simulation_msgs/catkin_generated/installspace/roboy_simulation_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_simulation_msgs" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_simulation_msgs/package.xml")
endif()

