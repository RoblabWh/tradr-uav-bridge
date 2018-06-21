cmake_minimum_required(VERSION 2.8)

project(tradr_uav_bridge)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wno-pmf-conversions")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    bridge_path_interface_msgs
    uav_videostream_msgs
)
find_package(Protobuf REQUIRED)
find_package(Boost 1.54 COMPONENTS thread date_time system chrono regex REQUIRED)

catkin_package()

file(GLOB_RECURSE PROTO_FILES "proto/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${PROTO_FILES})

file(GLOB_RECURSE SRC_DIR "*.cpp" "*.cc" "*.h")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/protofiles
)

add_library(messages SHARED ${ProtoSources} ${ProtoHeaders})

target_link_libraries(messages
    ${Boost_LIBRARIES}
    ${Protobuf_LIBRARIES}
)

link_directories(.)

add_executable(TradrUavBridge ${SRC_DIR})

add_dependencies(TradrUavBridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(TradrUavBridge LINK_PUBLIC
    messages
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARY}
)

file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

