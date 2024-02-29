cmake_minimum_required(VERSION 3.16.0)
project(experimental_pcl_transport)

set( COMPILE_TMC3_AS_LIBRARY ON )
add_subdirectory("mpeg-pcc-tmc13")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  open3d_conversions
  message_generation
)

find_package(Open3D REQUIRED)

add_message_files(
    FILES
    MinimalCompressedPointCloud2.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

# MESSAGE(">>>> Visible Source Directory = ${CMAKE_SOURCE_DIR}")
# MESSAGE(">>>> Visible Build Directory = ${CMAKE_BINARY_DIR}")
MESSAGE(">>>> Visible CURRENT Source Directory = ${CMAKE_CURRENT_SOURCE_DIR}")
MESSAGE(">>>> Visible CURRENT Build Directory = ${CMAKE_CURRENT_BINARY_DIR}")

set( EXTRA_LIBS_DIR "${CMAKE_BINARY_DIR}/../devel/lib/" )
MESSAGE(">>>>> ${Open3D_INCLUDE_DIRS}")


include_directories(
  include
    ${catkin_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/mpeg-pcc-tmc13/"
    "${CMAKE_CURRENT_SOURCE_DIR}/mpeg-pcc-tmc13/dependencies/program-options-lite/"
    "${CMAKE_CURRENT_SOURCE_DIR}/mpeg-pcc-tmc13/tmc3/"
    "${CMAKE_CURRENT_BINARY_DIR}/mpeg-pcc-tmc13/tmc3"
)

# MESSAGE( ">>>> ${catkin_INCLUDE_DIRS}" )
# MESSAGE( ">>>> ${catkin_LIBRARIES}" )

add_executable(compressed_pcl_publisher
    src/compressed_pcl_publisher.cpp
)

add_executable(compressed_pcl_subscriber
    src/compressed_pcl_subscriber.cpp
)

target_link_libraries(compressed_pcl_publisher
    ${catkin_LIBRARIES}
    ${Open3D_LIBRARIES}
    tmc3lib
)

target_link_libraries(compressed_pcl_subscriber
    ${catkin_LIBRARIES}
    ${Open3D_LIBRARIES}
    tmc3lib
)
# add_dependencies(talker beginner_tutorials_generate_messages_cpp)

