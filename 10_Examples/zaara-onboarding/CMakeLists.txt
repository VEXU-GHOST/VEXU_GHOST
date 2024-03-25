cmake_minimum_required(VERSION 3.8)
project(zaara-onboarding)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  std_msgs
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


add_executable(triv_method_publisher src/trivialmethodspublisher.cpp)
ament_target_dependencies(triv_method_publisher ${DEPENDENCIES})
install(TARGETS
  triv_method_publisher
  DESTINATION lib/${PROJECT_NAME})

add_library(triv_method_subscriber SHARED src/trivialmethodssubscriber.cpp)
ament_target_dependencies(triv_method_subscriber
  ${DEPENDENCIES}
)
ament_export_targets(triv_method_subscriber HAS_LIBRARY_TARGET)
install(
  TARGETS triv_method_subscriber
  EXPORT triv_method_subscriber
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(triv_method_main src/trivialmethodsmain.cpp)
ament_target_dependencies(triv_method_main
  ${DEPENDENCIES}
)
target_link_libraries(triv_method_main
  triv_method_subscriber
)
install(TARGETS
  triv_method_main
  DESTINATION lib/${PROJECT_NAME})

  ament_add_gtest(onboarding_test test/tests.cpp)
  target_link_libraries(onboarding_test
    triv_method_subscriber
    gtest
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )
 
  


ament_package()