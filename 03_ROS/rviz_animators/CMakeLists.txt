
cmake_minimum_required(VERSION 3.8)
project(rviz_animators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)


set(DEPENDENCIES
  ament_cmake
  rclcpp
  visualization_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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


ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
)
include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})


add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher ${DEPENDENCIES})
install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})

  add_library(subscriber SHARED src/subscriber.cpp)
  ament_target_dependencies(subscriber
    ${DEPENDENCIES}
  )
  ament_export_targets(subscriber HAS_LIBRARY_TARGET)
  install(
    TARGETS subscriber
    EXPORT subscriber
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  add_executable(subscriber_main src/subscriber_main.cpp)
  ament_target_dependencies(subscriber_main
  ${DEPENDENCIES}
)
target_link_libraries(subscriber_main
  subscriber
)

install(TARGETS
  subscriber_main
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()