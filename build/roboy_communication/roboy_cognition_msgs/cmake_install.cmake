# Install script for directory: /home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/msg" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/AudioData.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/AudioInfo.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/DirectionVector.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/FaceCoordinates.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/FacialFeatures.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/SpeechSynthesis.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/AudioLocation.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/RecognizedFaces.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/Faces.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/RecognizedSpeech.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/msg/IceCream.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/srv" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/ApplyFilter.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/AssertProperty.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/CallQuery.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/CheckProperty.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/CheckQuery.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/CreateInstance.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DataQuery.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DescribeScene.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DetectFace.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DetectIntent.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DriveToLocation.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/EvalSeq2Seq.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/FindInstances.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/FindObject.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/GenerateAnswer.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/GetObject.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/LookAtSpeaker.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/Payment.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/PlaySound.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/QualityOfTone.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/QualityOfTones.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/RecognizeFaces.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/RecognizeObject.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/RecognizeSpeech.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/SaveObject.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/ShowInstances.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/ShowProperty.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/ShowPropertyValue.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/Talk.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/TalkToFile.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/TrainSeq2Seq.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/WhichTones.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/DetectIceCream.srv"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/srv/LocalizeObject.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/action" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/action/OrderIceCream.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/msg" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamAction.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamActionGoal.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamActionResult.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamActionFeedback.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamGoal.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamResult.msg"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roboy_cognition_msgs/msg/OrderIceCreamFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/cmake" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_cognition_msgs/catkin_generated/installspace/roboy_cognition_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/include/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/roseus/ros/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/common-lisp/ros/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/share/gennodejs/ros/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/lib/python2.7/dist-packages/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/devel/lib/python2.7/dist-packages/roboy_cognition_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_cognition_msgs/catkin_generated/installspace/roboy_cognition_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/cmake" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_cognition_msgs/catkin_generated/installspace/roboy_cognition_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs/cmake" TYPE FILE FILES
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_cognition_msgs/catkin_generated/installspace/roboy_cognition_msgsConfig.cmake"
    "/home/sausy/Projects/lighthouse/cameraSterioCalibration/build/roboy_communication/roboy_cognition_msgs/catkin_generated/installspace/roboy_cognition_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roboy_cognition_msgs" TYPE FILE FILES "/home/sausy/Projects/lighthouse/cameraSterioCalibration/src/roboy_communication/roboy_cognition_msgs/package.xml")
endif()

