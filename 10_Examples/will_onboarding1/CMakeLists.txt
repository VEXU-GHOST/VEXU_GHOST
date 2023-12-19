cmake_minimum_required(VERSION 3.8)
project(will_onboarding1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  std_msgs
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(std_msgs REQUIRED)


# Tell any packages that depend on us what dependencies we need.
ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
)

# Tells the package where all our header files live
include_directories(${INCLUDE})

# Tells any packages that depend on us what include files we use.
ament_export_include_directories(${INCLUDE})


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

#Creating a library for the class that I made
add_library(Math_Node SHARED src/Mather.cpp)
ament_target_dependencies(Math_Node
${DEPENDENCIES})

#exporting the library
ament_export_targets(Math_Node HAS_LIBRARY_TARGET)
install(
  TARGETS Math_Node
  EXPORT Math_Node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#Main Method Stuff
add_executable(main_executable src/main.cpp)
ament_target_dependencies(main_executable
  ${DEPENDENCIES}
)
target_link_libraries(main_executable
Math_Node)

install(TARGETS
main_executable
DESTINATION lib/${PROJECT}
)

#Gtest stuff
ament_add_gtest(test_example src/wills_Test_File.cpp)
target_link_libraries(test_example
  Math_Node
  gtest
)

install(
  DIRECTORY include/
  DESTINATION include
)



ament_package()
