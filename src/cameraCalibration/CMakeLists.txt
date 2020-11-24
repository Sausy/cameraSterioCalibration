cmake_minimum_required(VERSION 3.15)
project(camerasterio)

#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
#set(CMAKE_CXX_STANDARD 14)
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -std=gnu99 -rdynamic -Werror=incompatible-pointer-types -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=implicit-function-declaration -Wpedantic -Werror=return-type -fvisibility=hidden -Werror=vla")

#SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# ROS Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
				roscpp
				#roslint
				#rqt_gui
				#rqt_gui_cpp
				std_msgs
				message_generation
        roboy_middleware_msgs
        roboy_control_msgs
				darkroom
				common_utilities
				visualization_msgs
				#rviz
				tf
				tf_conversions
				)

catkin_package(CATKIN_DEPENDS
        message_runtime
				roboy_middleware_msgs
				roboy_control_msgs
				darkroom
				common_utilities
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        )




add_subdirectory (include/mathUtils)

#========OPENCV=========
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


#====UsbDriver Stuff=========
FILE(GLOB SRC_FILES_USB include/usbDriver/*.cpp)
FILE(GLOB INCLUDE_FILES_USB include/usbDriver/*.h)

#========3dModel=========
FILE(GLOB SRC_FILES_MODEL include/load3dModel/*.cpp)
FILE(GLOB INCLUDE_FILES_MODEL include/load3dModel/*.h)

#========utils=========
#this includes utils like azimuth,elevation to 2D plane calculation
FILE(GLOB SRC_FILES_UTIL include/utils/*.cpp)
FILE(GLOB INCLUDE_FILES_UTIL include/utils/*.hpp)

#========filter=========
FILE(GLOB SRC_FILES_FILTER include/filter/*.cpp)
FILE(GLOB INCLUDE_FILES_FILTER include/filter/*.hpp)

#========custom ROS interface=========
FILE(GLOB SRC_FILES_ROS include/rosInterface/*.cpp)
FILE(GLOB INCLUDE_FILES_ROS include/rosInterface/*.hpp)

#==============================================================================
# Main Project files
#==============================================================================
FILE(GLOB SRC_FILES src/*.cpp)
FILE(GLOB INCLUDE_FILES src/*.h)
FILE(GLOB INCLUDE_FILES_B src/*.hpp)


#==============================================================================
# Project specific defines
#==============================================================================
#preprocessor definition for absolut path to project
add_definitions(-DSEARCH_PATH="${CMAKE_SOURCE_DIR}")

#==============================================================================
# Dependencies
#==============================================================================
include_directories(
        src/
        include/
        include/usbDriver/
        include/load3dModel/
        include/utils/
        include/filter/
        include/rosInterface/
      )

#==============================================================================
# Application
#==============================================================================
add_executable( ${PROJECT_NAME}
        ${SRC_FILES}
        #${INCLUDE_FILES}
        ${INCLUDE_FILES_B}
        ${SRC_FILES_USB}
        ${INCLUDE_FILES_USB}
        ${SRC_FILES_MODEL}
        ${INCLUDE_FILES_MODEL}
        ${SRC_FILES_UTIL}
        ${INCLUDE_FILES_UTIL}
        ${SRC_FILES_FILTER}
        ${INCLUDE_FILES_FILTER}
        ${SRC_FILES_ROS}
        ${INCLUDE_FILES_ROS}
        main.cpp
        #main_registration.cpp
        )


add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries( ${PROJECT_NAME} -lpthread -lusb-1.0 -lrt ${catkin_LIBRARIES} ${OpenCV_LIBS} mathUtils )
