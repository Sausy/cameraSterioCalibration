cmake_minimum_required(VERSION 2.8.3)
project(vive_ros)

# TODO: make find script, to use find_package(openvr REQUIRED)
set(OPENVR "$ENV{HOME}/Downloads/openvr")
find_library(OPENVR_LIB openvr_api "${OPENVR}/lib/linux64" NO_DEFAULT_PATH)
if (OPENVR_LIB)
    ## Find catkin macros and libraries
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            tf
            std_srvs
            geometry_msgs
            darkroom
            )

    set(CMAKE_BUILD_TYPE "Release")


    message("Found libopenvr_api: ${OPENVR_LIB}")
    ###################################
    ## catkin specific configuration ##
    ###################################
    catkin_package(LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS darkroom)

    ###########
    ## Build ##
    ###########
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    # TODO: find openvr
    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${OPENVR}/headers
    )

    # VIVE node:
    add_executable(vive_node
            src/vive_node.cpp
            src/vr_interface.cpp
            )
    target_link_libraries(vive_node ${catkin_LIBRARIES} ${OPENVR_LIB})

else ()
    message(STATUS "could not find STEAMVR, won't compile vive_ros!!!")
endif ()