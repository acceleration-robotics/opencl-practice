cmake_minimum_required(VERSION 3.8)
project(bag_reading_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(OpenCL 2.2 EXACT REQUIRED)

include_directories(include)

# add_executable(image_bag_reader src/image_bag_reader.cpp)
# ament_target_dependencies(
#   image_bag_reader 
#   rclcpp 
#   rosbag2_cpp 
#   rosbag2_storage
#   cv_bridge
#   image_transport
#   sensor_msgs
#   image_geometry
#   OpenCL)

add_executable(image_rectify src/image_rectify.cpp)
ament_target_dependencies(
    image_rectify 
    rclcpp 
    rosbag2_cpp 
    rosbag2_storage
    cv_bridge
    image_transport
    sensor_msgs
    image_geometry
    OpenCL)

# add_executable(image_rectify_cpu src/image_rectify_cpu.cpp)
# ament_target_dependencies(
#     image_rectify_cpu
#     rclcpp 
#     rosbag2_cpp 
#     rosbag2_storage
#     cv_bridge
#     image_transport
#     sensor_msgs
#     image_geometry
#     OpenCL)

add_executable(image_resize src/image_resize.cpp)
ament_target_dependencies(
    image_resize 
    rclcpp 
    rosbag2_cpp 
    rosbag2_storage
    cv_bridge
    image_transport
    sensor_msgs
    image_geometry
    OpenCL)

add_executable(rectify_resize_cl src/rectify_resize_cl.cpp)
ament_target_dependencies(
    rectify_resize_cl 
    rclcpp 
    rosbag2_cpp 
    rosbag2_storage
    cv_bridge
    image_transport
    sensor_msgs
    image_geometry
    OpenCL)

add_executable(rectify_resize_cpu src/rectify_resize_cpu.cpp)
ament_target_dependencies(
    rectify_resize_cpu 
    rclcpp 
    rosbag2_cpp 
    rosbag2_storage
    cv_bridge
    image_transport
    sensor_msgs
    image_geometry
    OpenCL)

add_executable(test_zerocopy src/zerocopy_test_onlyhost_endmap.cpp) # --works
# add_executable(test_zerocopy src/zerocopy_test_onlyhost_endmap_resize.cpp) #-- works, resize
# add_executable(test_zerocopy src/zerocopy_test.cpp)
ament_target_dependencies(
  test_zerocopy
  OpenCL)

# Link your executable with OpenCV
# target_include_directories(image_bag_reader PRIVATE ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(image_bag_reader ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

target_include_directories(image_rectify PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_rectify ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

target_include_directories(image_resize PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_resize ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

target_include_directories(test_zerocopy PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_zerocopy ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

# target_include_directories(image_rectify_cpu PRIVATE ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(image_rectify_cpu ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

target_include_directories(rectify_resize_cl PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rectify_resize_cl ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

target_include_directories(rectify_resize_cpu PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rectify_resize_cpu ${OpenCV_LIBRARIES} ${OpenCL_LIBRARY})

# install(TARGETS
#   image_bag_reader
#   DESTINATION lib/${PROJECT_NAME}
#   )

install(TARGETS
  image_rectify
  image_resize
  test_zerocopy
  rectify_resize_cl
  rectify_resize_cpu
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

ament_package()
