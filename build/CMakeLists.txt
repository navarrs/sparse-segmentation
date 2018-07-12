cmake_minimum_required(VERSION 2.8)
project(GroundExtraction)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories( "../include" )

add_library(gealgorithm ../src/groundExtractor.cpp)

add_library(pointcloud ../src/pointCloud.cpp)

add_executable(../extractGround ../src/mainGround.cpp)
add_executable(../squeezeGround ../src/mainGroundSqueeze.cpp)

target_include_directories(gealgorithm PRIVATE ${../include})

target_include_directories(../extractGround PRIVATE ${../include})
target_include_directories(../squeezeGround PRIVATE ${../include})

target_link_libraries(../extractGround gealgorithm pointcloud ${Boost_LIBRARIES})
target_link_libraries(../squeezeGround gealgorithm pointcloud ${Boost_LIBRARIES})
