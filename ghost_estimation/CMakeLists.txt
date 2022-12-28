cmake_minimum_required(VERSION 3.5)
project(ghost_estimation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)

###############
### Filters ###
###############
add_library(first_order_low_pass_filter SHARED
src/filters/first_order_low_pass_filter.cpp
)
target_include_directories(first_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(first_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS first_order_low_pass_filter
  EXPORT first_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(second_order_low_pass_filter SHARED
src/filters/second_order_low_pass_filter.cpp
)
target_include_directories(second_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(second_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS second_order_low_pass_filter
  EXPORT second_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
