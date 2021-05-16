cmake_minimum_required(VERSION 3.17)
project(master_cpp)

set(CMAKE_CXX_STANDARD 17)


#find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenCV REQUIRED )
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()


find_package(realsense2 REQUIRED)
if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()

add_executable(master_cpp main.cpp src/TrackingObject.cpp src/TrackingObject.h src/Sample.h src/SampleGrid.cpp src/SampleGrid.h src/constants.h src/SpeedEstimation.cpp src/SpeedEstimation.h src/RealSenseCamera.cpp src/RealSenseCamera.h src/Grid.cpp src/Grid.h src/ObjectAssignment.h src/Transform3D.h src/Grid.h src/Grid.cpp src/DataGrid.h src/SpeedEstimationUtil.cpp src/Benchmark.cpp src/Benchmark.h src/helper.h)
include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(${Open3D_LIBRARY_DIRS})

target_link_libraries(master_cpp  glfw ${OpenCV_LIBS} ${realsense2_LIBRARY} ${Open3D_LIBRARIES})
target_include_directories(master_cpp PUBLIC ${Open3D_INCLUDE_DIRS})


