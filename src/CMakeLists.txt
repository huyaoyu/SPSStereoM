#Project
cmake_minimum_required(VERSION 3.8)
project(spsstereo)

set(CMAKE_BUILD_TYPE Debug)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-msse4.2")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-msse4.2")

# Flags
#set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -msse4.2")
#if (APPLE)
#set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -msse4.2 -stdlib=libstdc++")
#endif (APPLE)
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /opt/ros/kinetic/share/OpenCV-3.3.1-dev)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

include_directories(AFTER ${OpenCV_INCLUDE_DIRS})

include_directories(BEFORE ../includes)

# Defines.
#add_compile_definitions(TINYPLY_IMPLEMENTATION)

# SPS-Stereo
file(GLOB_RECURSE SPSSTEREO_SRC_FILES "*.cpp")

IF (SPSSTEREO_SRC_FILES)
    FOREACH (SFILE ${SPSSTEREO_SRC_FILES})
        MESSAGE(STATUS "CPP: ${SFILE}")
    ENDFOREACH (SFILE)
ENDIF (SPSSTEREO_SRC_FILES)

add_executable(spsstereo ${SPSSTEREO_SRC_FILES})
#add_executable(spsstereo spsstereo_main.cpp SGMStereo.cpp SPSStereo.cpp StereoUtils/PLY.cpp)
target_include_directories(spsstereo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(spsstereo ${OpenCV_LIBRARIES})
