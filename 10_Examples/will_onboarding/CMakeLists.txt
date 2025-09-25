cmake_minimum_required(VERSION 3.8)
project(will_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dependencies and libraries

# find dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  std_msgs
)

foreach(dep IN LISTS DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

# include stuff
set(INCLUDE
  include
)

# Tells the package where all our header files live
include_directories(${INCLUDE})

# Tells any packages that depend on us what include files we use.
ament_export_include_directories(${INCLUDE})

# Libraries
add_library(pub_sub src/pub_sub.cpp)
ament_target_dependencies(pub_sub
  ${DEPENDENCIES}
)
install(
  TARGETS pub_sub
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()



# Executables
add_executable(main src/main.cpp)
target_link_libraries(main pub_sub)

ament_target_dependencies(main
  ${DEPENDENCIES}
)

# Tests
ament_add_gtest(test_example test/test.cpp)
target_include_directories(test_example PRIVATE ${INCLUDE})
target_link_libraries(test_example
  pub_sub
  gtest
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

  install(
  DIRECTORY include/
  DESTINATION include
)

# # This copies the launch/urdf/config folders to ~/VEXU_GHOST/install/ghost_examples/share/ghost_examples.
# install(DIRECTORY
#   launch
#   urdf
#   config
#   DESTINATION share/${PROJECT_NAME})

# This generates the ROS2 package and should always be the last line.
ament_package()

