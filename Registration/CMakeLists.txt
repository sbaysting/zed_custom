SET(execName registration)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(PCL)
find_package(Eigen3)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(EIGEN3_INCLUDE_DIR)

link_directories(${PCL_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
 
#ADD_EXECUTABLE(${execName} ${SRC_FILES})
ADD_EXECUTABLE(${execName} src/main.cpp src/filters.cpp src/io.cpp src/stitching_algorithms.cpp src/cloud_viewer.cpp)
set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "registration")

TARGET_LINK_LIBRARIES(${execName}
                        ${PCL_LIBRARIES}
                    )
