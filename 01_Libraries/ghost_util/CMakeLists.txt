cmake_minimum_required(VERSION 3.8)
project(ghost_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
  include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})

######################
##### Ghost Util #####
######################
add_library(angle_util SHARED
  src/angle_util.cpp
)
target_include_directories(angle_util
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(angle_util HAS_LIBRARY_TARGET)
install(
  TARGETS angle_util
  EXPORT angle_util
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(byte_utils SHARED
  src/byte_utils.cpp
)
target_include_directories(byte_utils
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(byte_utils HAS_LIBRARY_TARGET)
install(
  TARGETS byte_utils
  EXPORT byte_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

#################
#### Testing ####
#################
ament_add_gtest(test_angle_util test/test_angle_util.cpp)
target_link_libraries(test_angle_util
  angle_util
  gtest
)

ament_add_gtest(test_vector_list test/test_vector_util.cpp)
target_link_libraries(test_vector_list
  gtest
)

ament_add_gtest(test_parsing_util test/test_parsing_util.cpp)
target_link_libraries(test_parsing_util
  gtest
)

ament_package()
