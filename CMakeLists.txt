cmake_minimum_required(VERSION 3.26)
project(mono_slam)

set(CMAKE_CXX_STANDARD 17)


# OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 not found")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# Sophus
find_package(Sophus REQUIRED)
if(NOT Sophus_FOUND)
    message(FATAL_ERROR "Sophus not found")
endif()
include_directories(${Sophus_INCLUDE_DIRS})

# Glog
find_package(Glog REQUIRED)
if(NOT Glog_FOUND)
    message(FATAL_ERROR "Glog not found")
endif()
include_directories(${GLOG_INCLUDE_DIRS})

#find_package(GOOGLE REQUIRED)

add_executable(mono_slam main.cpp
        include/frame.h
        include/common_include.h
        include/camera.h
        src/camera.cpp
        src/frame.cpp
        src/frame.cpp
        include/feature.h
        src/feature.cpp
        include/mappoint.h
        src/mappoint.cpp
        include/map.h
        src/map.cpp
)

target_link_libraries(mono_slam
        ${OpenCV_LIBS}
        ${EIGEN3_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${GLOG_LIBRARIES})
