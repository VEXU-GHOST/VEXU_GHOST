cmake_minimum_required(VERSION 3.8)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  geometry_msgs
  ghost_msgs
  Eigen3
  eigen3_cmake_module
  )

# find dependencies
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

add_executable(diff_drive_ss
  src/diff_drive_ss.cpp
)

ament_target_dependencies(diff_drive_ss
  ${DEPENDENCIES}
)

target_include_directories(diff_drive_ss
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  diff_drive_ss
  DESTINATION lib/${PROJECT_NAME})


add_executable(mtr_vltg_pub
  test/mtr_vltg_pub.cpp
)

ament_target_dependencies(mtr_vltg_pub
  ${DEPENDENCIES}
)

target_include_directories(mtr_vltg_pub
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  mtr_vltg_pub
  DESTINATION lib/${PROJECT_NAME})

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
