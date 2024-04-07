cmake_minimum_required(VERSION 3.8)
project(annie-onboarding)



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



# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(ament_cmake_gtest REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Tell any packages that depend on us what dependencies we need.
ament_export_dependencies(${DEPENDENCIES})

# Add any include directories to a list
# Usually this is just "include", but for some depedencies there could be extras.
set(INCLUDE
  include
)

# Tells the package where all our header files live
include_directories(${INCLUDE})

# Tells any packages that depend on us what include files we use.
ament_export_include_directories(${INCLUDE})

# add_library(hello_world SHARED src/hello_world.cpp)
# ament_target_dependencies(hello_world
#   ${DEPENDENCIES}
# )
# ament_export_targets(hello_world HAS_LIBRARY_TARGET)
# install(
#   TARGETS hello_world
#   EXPORT hello_world
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# add_executable(hello_world_main src/hello_world_main.cpp)
# target_link_libraries(hello_world_main
# hello_world
# )
# install(TARGETS
# hello_world_main
#   DESTINATION lib/${PROJECT_NAME})

#   ament_add_gtest(test_example src/test.cpp)
#   target_link_libraries(test_example
#   gtest
#   hello_world
#   )



  add_executable(publisher_member_function src/publisher_member_function.cpp)
  ament_target_dependencies(publisher_member_function ${DEPENDENCIES})
  install(TARGETS
  publisher_member_function
    DESTINATION lib/${PROJECT_NAME})

  add_library(subscriber_member_function SHARED src/subscriber_member_function.cpp)
  ament_target_dependencies(subscriber_member_function 
  ${DEPENDENCIES}
  )

  ament_export_targets(subscriber_member_function HAS_LIBRARY_TARGET)
  install(
  TARGETS subscriber_member_function
  EXPORT subscriber_member_function
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(main src/main.cpp)
ament_target_dependencies(main
  ${DEPENDENCIES}
)

target_link_libraries(main
  subscriber_member_function 
)
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

  ament_add_gtest(onboarding_test test/test.cpp)
  target_link_libraries(onboarding_test
    subscriber_member_function
    gtest
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  ament_package()