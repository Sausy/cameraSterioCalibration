cmake_minimum_required(VERSION 3.1)
project(camerasterio)


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

#==============================================================================
# Main Project files
#==============================================================================
FILE(GLOB SRC_FILES src/*.cpp)
FILE(GLOB INCLUDE_FILES src/*.h)


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
        ${SRC_FILES_UTIL}
        ${INCLUDE_FILES_UTIL}
        ${SRC_FILES_FILTER}
        ${INCLUDE_FILES_FILTER}
        main.cpp
        #main_registration.cpp
        )

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} mathUtils)
