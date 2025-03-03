cmake_minimum_required(VERSION 3.8)
project(inrof2025_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


function(custom_examples target)
  add_executable(${target} examples/${target}.cpp)
  ament_target_dependencies(${target} 
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "gazebo_msgs"
    "visualization_msgs"
  )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()


function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} 
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "gazebo_msgs"
    "visualization_msgs"
  )
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(custom_component target class_name)
  rclcpp_components_register_node(${target}
    PLUGIN ${class_name}
    EXECUTABLE ${target}
  )
  ament_target_dependencies(${target}
    "rclcpp"
    "geometry_msgs"
    "std_msgs"
    "tf2_ros"
    "sensor_msgs"
    "nav_msgs"
    "rclcpp_components"
    "gazebo_msgs"
    "visualization_msgs"
  )
endfunction()

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

install(
  DIRECTORY launch worlds models urdf config map
  DESTINATION share/${PROJECT_NAME}
)

custom_executable(omuni_drive_node)
custom_executable(omuni_odometry)
custom_executable(example_omuni_drive_node)
custom_executable(diff_drive)
custom_executable(mcl_node)

# custom_examples(diff_drive)

ament_package()
