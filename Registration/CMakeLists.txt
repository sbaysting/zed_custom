SET(execName registration)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
 
#ADD_EXECUTABLE(${execName} ${SRC_FILES})
ADD_EXECUTABLE(${execName} src/main.cpp src/voxelgrid_filter.cpp src/outlier_filter.cpp src/io.cpp)
set_property(TARGET ${execName} PROPERTY OUTPUT_NAME "registration")

TARGET_LINK_LIBRARIES(${execName}
                        ${PCL_LIBRARIES}
                    )
