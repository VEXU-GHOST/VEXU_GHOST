cmake_minimum_required(VERSION 3.8)
project(TiannaResendez_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(my_class_library src/my_class.cpp)

ament_target_dependencies(my_class_library
  "rclcpp"
)

# ######################################################################
# ## NEW AND FINAL SECTION: Tell the compiler where to find header files
# ## This tells any target that uses 'my_class_library' to look in the 'include' folder
# ######################################################################
target_include_directories(my_class_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ######################################################################

add_executable(hello_world src/hello_world.cpp)

target_link_libraries(hello_world my_class_library)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
