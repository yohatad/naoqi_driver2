cmake_minimum_required(VERSION 3.5)
project(naoqi_driver)

# C++17 is required for std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Limit parallel compilation to prevent OOM
if(CMAKE_GENERATOR MATCHES "Ninja")
  set_property(GLOBAL PROPERTY JOB_POOLS single_job=1)
  set(CMAKE_JOB_POOL_COMPILE single_job)
  set(CMAKE_JOB_POOL_LINK single_job)
endif()

# For any generator: limit build parallelism via build tool mode
set(CMAKE_BUILD_PARALLEL_LEVEL 1 CACHE STRING "Limit parallel build jobs")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(naoqi_bridge_msgs REQUIRED)
find_package(naoqi_libqi REQUIRED)
find_package(naoqi_libqicore REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost QUIET COMPONENTS chrono program_options regex system thread random)

set(
  CONVERTERS_SRC
  src/converters/audio.cpp
  src/converters/touch.cpp
  src/converters/camera.cpp
  src/converters/diagnostics.cpp
  src/converters/imu.cpp
  src/converters/info.cpp
  src/converters/joint_state.cpp
  src/converters/laser.cpp
  src/converters/memory_list.cpp
  src/converters/memory/bool.cpp
  src/converters/memory/int.cpp
  src/converters/memory/float.cpp
  src/converters/memory/string.cpp
  src/converters/sonar.cpp
  src/converters/log.cpp
  src/converters/odom.cpp
  )
set(
  TOOLS_SRC
  src/tools/robot_description.cpp
  src/tools/from_any_value.cpp
  )

set(
  PUBLISHER_SRC
  src/publishers/camera.cpp
  src/publishers/joint_state.cpp
  src/publishers/log.cpp
  src/publishers/sonar.cpp
  )

set(
  SUBSCRIBER_SRC
  src/subscribers/teleop.cpp
  src/subscribers/moveto.cpp
  src/subscribers/speech.cpp
  )

set(
  SERVICES_SRC
  src/services/robot_config.cpp
  src/services/set_language.cpp
  src/services/get_language.cpp
  src/services/odom_reset.cpp
  src/services/trigger.cpp
  )

set(
  ACTIONS_SRC
  src/actions/listen.cpp
  src/actions/led.cpp
  src/actions/speech.cpp
  src/actions/audio_player.cpp
)

set(
  RECORDER_SRC
  src/recorder/camera.cpp
  src/recorder/diagnostics.cpp
  src/recorder/joint_state.cpp
  src/recorder/log.cpp
  src/recorder/sonar.cpp
  )

set(
  DRIVER_SRC
  src/naoqi_driver.cpp
  src/boot_config.cpp
  src/heartbeat.cpp
  src/helpers/driver_helpers.cpp
  src/helpers/ros_helpers.cpp
  src/recorder/globalrecorder.cpp
  src/event/basic.hxx
  src/event/basic.hpp
  src/event/audio.cpp
  src/event/touch.cpp
  src/ros_env.cpp
)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

# Add pre-processor compile flag indicating ament
add_definitions( -DAMENT_BUILD )

include_directories(
  "include"
  "src"
  ${naoqi_libqi_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  naoqi_driver
  SHARED
  ${DRIVER_SRC}
  ${CONVERTERS_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${SERVICES_SRC}
  ${ACTIONS_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
)

ament_target_dependencies(naoqi_driver
  rclcpp
  rclcpp_action
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  image_transport
  kdl_parser
  naoqi_bridge_msgs
  naoqi_libqi
  naoqi_libqicore
  robot_state_publisher
  rosbag2_cpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  std_srvs
  OpenCV
  Boost

)

install(TARGETS naoqi_driver DESTINATION lib/)

# create the binary of the bridge
add_executable(naoqi_driver_node src/external_registration.cpp)
target_link_libraries(naoqi_driver_node
    naoqi_driver
    ${rclcpp_LIBRARIES}
    ${naoqi_libqi_LIBRARIES}
    ${Boost_LIBRARIES}
)

install(TARGETS naoqi_driver_node DESTINATION lib/${PROJECT_NAME})

# install the urdf for runtime loading
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install the rviz2 configurations
install(DIRECTORY rviz2 DESTINATION share/${PROJECT_NAME})

# boot_config.json (converter defaults), pepper.yaml and safe_teleop
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# scripts that run on the robot, to copy there (see README)
install(PROGRAMS scripts/robot_watchdog.py DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  # Code style: 'ament_uncrustify --reformat src include test' fixes any failure.
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(src include test)

  find_package(ament_cmake_gtest REQUIRED)

  # ThreadSanitizer makes the threaded tests several times slower.
  if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=thread")
    set(NAOQI_DRIVER_TEST_TIMEOUT 600)
  else()
    set(NAOQI_DRIVER_TEST_TIMEOUT 120)
  endif()

  function(naoqi_driver_add_test name)
    ament_add_gtest(${name} test/${name}.cpp TIMEOUT ${NAOQI_DRIVER_TEST_TIMEOUT})
    target_include_directories(${name} PRIVATE test)
    # The gtest headers matching the linked gtest_vendor library come first: some packages
    # (rtabmap 0.23.13) install other gtest headers into /opt/ros/<distro>/include.
    target_include_directories(${name} BEFORE PRIVATE ${GTEST_INCLUDE_DIRS})
    target_compile_definitions(${name} PRIVATE NAOQI_DRIVER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${name} naoqi_driver ${naoqi_libqi_LIBRARIES})
    ament_target_dependencies(${name}
      rclcpp
      geometry_msgs
      naoqi_bridge_msgs
      naoqi_libqi
      sensor_msgs
      tf2_msgs
    )
  endfunction()

  naoqi_driver_add_test(test_string_helpers)
  naoqi_driver_add_test(test_filesystem_helpers)
  naoqi_driver_add_test(test_camera_info)
  naoqi_driver_add_test(test_boost_ownership)
  naoqi_driver_add_test(test_recorder)
  naoqi_driver_add_test(test_driver)
  naoqi_driver_add_test(test_teleop)
  naoqi_driver_add_test(test_moveto)
  naoqi_driver_add_test(test_proximity_guard)
  naoqi_driver_add_test(test_async_helpers)
  naoqi_driver_add_test(test_goal_worker)
  naoqi_driver_add_test(test_listen)
  naoqi_driver_add_test(test_route_actions)
  naoqi_driver_add_test(test_boot_config)
  naoqi_driver_add_test(test_recorder_state)
  naoqi_driver_add_test(test_log_source)
  naoqi_driver_add_test(test_naoqi_image)
  naoqi_driver_add_test(test_motion_safety)
  naoqi_driver_add_test(test_driver_shutdown)
  naoqi_driver_add_test(test_heartbeat)
  naoqi_driver_add_test(test_speech)
  naoqi_driver_add_test(test_clock_offset)
  naoqi_driver_add_test(test_recording)
  naoqi_driver_add_test(test_imu)
  ament_target_dependencies(test_recording rosbag2_cpp std_srvs)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_watchdog test/test_robot_watchdog.py TIMEOUT 60)

  # naoqi_driver_node as a process, against a fake NAOqi in another process.
  add_executable(fake_naoqi_server test/fake_naoqi_server.cpp)
  target_include_directories(fake_naoqi_server PRIVATE test)
  target_link_libraries(fake_naoqi_server ${naoqi_libqi_LIBRARIES})
  ament_target_dependencies(fake_naoqi_server naoqi_libqi Boost)
  ament_add_pytest_test(test_driver_process test/test_driver_process.py TIMEOUT 180
    ENV
    FAKE_NAOQI_SERVER=$<TARGET_FILE:fake_naoqi_server>
    DRIVER_NODE=$<TARGET_FILE:naoqi_driver_node>)

  # launch/safe_teleop.launch.py; skipped without its optional packages.
  ament_add_pytest_test(test_safe_teleop test/test_safe_teleop.py TIMEOUT 120)
endif()

ament_export_libraries(naoqi_driver)
ament_package()