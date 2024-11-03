cmake_minimum_required(VERSION 3.8)
project(corey-onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

set(DEPENDENCIES
  ament_cmake
  ament_cmake_python
  ament_cmake_gtest
  rclcpp
  rclpy
  sensor_msgs
  geometry_msgs
  yaml-cpp
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_export_dependencies(${DEPENDENCIES})

set(INCLUDE
  include
)

include_directories(${INCLUDE})

ament_export_include_directories(${INCLUDE})

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node ${DEPENDENCIES})

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node ${DEPENDENCIES})

add_executable(hello_world src/helloworld.cpp)
ament_target_dependencies(hello_world ${DEPENDENCIES})


add_library(bankmain_lib SHARED src/bankmain.cpp)
ament_target_dependencies(bankmain_lib ${DEPENDENCIES})
ament_export_targets(bankmain_lib HAS_LIBRARY_TARGET)
install(
  TARGETS bankmain_lib
  EXPORT bankmain_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
add_executable(bankmain_exe src/bankmain.cpp)
ament_target_dependencies(bankmain_exe
  ${DEPENDENCIES}
)
target_link_libraries(bankmain_exe
  bankmain_lib  
)
install(TARGETS
  bankmain_exe
  DESTINATION lib/${PROJECT_NAME})

ament_add_gtest(bankmain_test test/banktest.cpp)
target_link_libraries(bankmain_test
  bankmain_lib
  gtest
)
target_include_directories(bankmain_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  hello_world
  publisher_node
  subscriber_node
  bankmain_exe
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
