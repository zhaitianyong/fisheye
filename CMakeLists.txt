cmake_minimum_required(VERSION 2.6)
project(fisheye)


# Check C++11 or C++0x support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE "Debug")


# opencv
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

# Eigen3
find_package(Eigen3 3.1.0 REQUIRED)

# Ceres
find_package(Ceres REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${CERES_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(test_opencv_fisheye
        sample/test_opencv_fisheye.cpp
        src/opencv_fisheye.cpp
        )
target_link_libraries(test_opencv_fisheye
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        )


#install(TARGETS calibration RUNTIME DESTINATION bin)
