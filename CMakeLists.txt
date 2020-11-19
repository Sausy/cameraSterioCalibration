cmake_minimum_required(VERSION 3.1)
project(camerasterio)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

FILE(GLOB SRC_FILES src/*.cpp)
FILE(GLOB INCLUDE_FILES src/*.h)

#====UsbDriver Stuff=====
FILE(GLOB SRC_FILES_USB usbDriver/*.cpp)
FILE(GLOB INCLUDE_FILES_USB usbDriver/*.h)

#====3dModel=====
FILE(GLOB SRC_FILES_MODEL load3dModel/*.cpp)
FILE(GLOB INCLUDE_FILES_MODEL load3dModel/*.h)


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
        usbDriver/
        load3dModel/
      )

#==============================================================================
# Application
#==============================================================================
add_executable( ${PROJECT_NAME}
        ${SRC_FILES}
        ${INCLUDE_FILES}
        ${SRC_FILES_USB}
        ${INCLUDE_FILES_USB}
        ${SRC_FILES_MODEL}
        ${INCLUDE_FILES_MODEL}
        main.cpp
        #main_registration.cpp
        )

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
