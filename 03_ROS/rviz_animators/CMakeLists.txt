cmake_minimum_required(VERSION 3.8)
project(rviz_animators)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# foreach(pkg ${DEPENDENCIES})
#   find_package(${pkg} REQUIRED)
# endforeach()

find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPENDENCIES
  ament_cmake
  rclcpp
  visualization_msgs
  std_msgs
  )

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

add_executable(main src/subscriber_main.cpp)
ament_target_dependencies(main
  ${DEPENDENCIES}
)

target_link_libraries(main
  subscriber
)
install(TARGETS
main
  DESTINATION lib/${PROJECT_NAME})

#   ament_add_gtest(animation_test test/test.cpp)
#   target_link_libraries(animation_test
#     subscriber
#     gtest
#   )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

  ament_package()