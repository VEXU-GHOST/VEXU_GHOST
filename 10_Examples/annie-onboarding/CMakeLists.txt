cmake_minimum_required(VERSION 3.8)
project(annie-onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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

add_library(hello_world SHARED src/hello_world.cpp)
ament_target_dependencies(hello_world
  ${DEPENDENCIES}
)
ament_export_targets(hello_world HAS_LIBRARY_TARGET)
install(
  TARGETS hello_world
  EXPORT hello_world
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(hello_world_main src/hello_world_main.cpp)
target_link_libraries(hello_world_main
hello_world
)
install(TARGETS
hello_world_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()


# ament_add_gtest(test_example test.cpp)
# target_link_libraries(test_example
#   gtest
# )


#hii