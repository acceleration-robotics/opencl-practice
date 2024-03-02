cmake_minimum_required(VERSION 3.8)
project(bag_reading_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-g)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(OpenCL REQUIRED)
find_package(a1_perception_2nodes REQUIRED)
find_package(ros2_benchmark REQUIRED)
find_package(tracetools_benchmark REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# resize library
ament_auto_add_library(resize_cl SHARED
  src/resize_cl.cpp
)

ament_auto_add_library(rectify_cl SHARED
  src/rectify_cl.cpp
)

ament_auto_add_library(rectify_resize_cl SHARED
  src/rectify_resize_cl.cpp
)

# add_executable(resize_cl src/resize_cl.cpp)
ament_target_dependencies(
    resize_cl 
    rclcpp 
    tracetools_image_pipeline
    cv_bridge
    image_transport
    sensor_msgs
    image_geometry
    a1_perception_2nodes
    ros2_benchmark
    tracetools_benchmark
    OpenCL)

ament_target_dependencies(
  rectify_cl 
  rclcpp 
  tracetools_image_pipeline
  cv_bridge
  image_transport
  sensor_msgs
  image_geometry
  a1_perception_2nodes
  ros2_benchmark
  tracetools_benchmark
  OpenCL)

ament_target_dependencies(
  rectify_resize_cl
  rclcpp 
  tracetools_image_pipeline
  cv_bridge
  image_transport
  sensor_msgs
  image_geometry
  a1_perception_2nodes
  ros2_benchmark
  tracetools_benchmark
  OpenCL)

add_executable(sync_node src/sync_node.cpp)
ament_target_dependencies(
    sync_node 
    rclcpp 
    rosbag2_cpp 
    rosbag2_storage
    sensor_msgs
    message_filters)

# set(node_plugins "")

ament_auto_add_library(image_input_component SHARED src/image_input_component.cpp)
ament_target_dependencies(image_input_component tracetools_benchmark image_transport)
target_compile_definitions(image_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(image_input_component "robotperf::perception::ImageInputComponent")
set(node_plugins "${node_plugins}robotperf::perception::ImageInputComponent;$<TARGET_FILE:image_input_component>\n")


target_compile_definitions(resize_cl
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(resize_cl "image_proc::CLImageProc")
set(node_plugins "${node_plugins}image_proc::CLImageProc;$<TARGET_FILE:resize_cl>\n")

target_compile_definitions(rectify_cl
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(rectify_cl "image_proc::RectifyCL")
set(node_plugins "${node_plugins}image_proc::RectifyCL;$<TARGET_FILE:rectify_cl>\n")


target_compile_definitions(rectify_resize_cl
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(rectify_resize_cl "image_proc::RectifyResizeCL")
set(node_plugins "${node_plugins}image_proc::RectifyResizeCL;$<TARGET_FILE:rectify_resize_cl>\n")

install(TARGETS
  sync_node
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(node_plugins "")

ament_auto_package(INSTALL_TO_SHARE launch)
