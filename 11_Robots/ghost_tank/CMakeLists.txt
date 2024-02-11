cmake_minimum_required(VERSION 3.8)
project(ghost_tank)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()


# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_gtest REQUIRED)
# find_package(rclcpp REQUIRED)



# # Tell any packages that depend on us what dependencies we need.
# ament_export_dependencies(${DEPENDENCIES})

# Add any include directories to a list
# Usually this is just "include", but for some depedencies there could be extras.
set(INCLUDE
  include
)

# Tells the package where all our header files live
include_directories(${INCLUDE})

# Tells any packages that depend on us what include files we use.
ament_export_include_directories(${INCLUDE})

add_library(tank_odom src/tank_wheel_odom.cpp)
ament_target_dependencies(tank_odom ${DEPENDENCIES})
ament_export_targets(tank_odom HAS_LIBRARY_TARGET)
install(
  TARGETS tank_odom
  EXPORT tank_odom
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(tank_main src/tank_wheel_main.cpp)
ament_target_dependencies(tank_main
 ${DEPENDENCIES}
)
target_link_libraries(tank_main
  tank_odom
)
install(TARGETS
  tank_main
  DESTINATION lib/${PROJECT_NAME})

ament_add_gtest(tank_test test/tank_wheel_tests.cpp)
target_link_libraries(tank_test
  gtest
  tank_odom
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
