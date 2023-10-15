cmake_minimum_required(VERSION 3.8)
project(ghost_examples)

# Adds options for GCC and Clang compilers to show all warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Adds all our package dependencies to one list
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  rclcpp
  sensor_msgs
  geometry_msgs
  yaml-cpp
  )

# Iterates through the list and finds the packages!
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

# This is another way to find packages, you can use either.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

####################
##### Targets ######
####################
# CONTEXT
# In compiled languages, any code we make is either going to be an Executable, or a Library.
# an Executable has a "main" function (and only one!), so it can run on its own.
# A Library does not have a main, and so it has to be used by an executable.
# Usually, your Executables should be small and most of your code should live in a Library.
# This is because it makes it easier to test (because we can have a separate executable for testing).

# EXAMPLE 1
# This block adds a new executable called "ros_publisher_example".
# add_executable:
#   defines the name of the executable, and then follows it with ALL related cpp files.
#   Among these cpp files, there can only be one "main" function.
# ament_target_dependencies:
#   Again, we start with the executable name, then we add the names of all the ROS Package dependencies
# install:
#   This function is what actually causes the executable file to show up in our build folder (where our binaries live).
#   If you navigate to ~/VEXU_GHOST/install/ghost_examples/lib/ghost_examples, you will see an executable
#   called ros_publisher_example! Notice how that matches up with the DESTINATION of lib/${PROJECT_NAME}.
add_executable(ros_publisher_example src/ros_publisher_example.cpp)
ament_target_dependencies(ros_publisher_example ${DEPENDENCIES})
install(TARGETS
  ros_publisher_example
  DESTINATION lib/${PROJECT_NAME})

# EXAMPLE 2
# This adds an example for using YAML files. This block has one important difference.
# Instead of ament_target_dependencies, we use target_link_libraries.
# That is because yaml-cpp is not a ROS package, it is just raw C++.
# This block would be valid even if we weren't using ROS, although we will usually use ament_target_dependencies.
add_executable(yaml_load_example src/yaml_load.cpp)
target_link_libraries(yaml_load_example
  yaml-cpp 
)
install(TARGETS
  yaml_load_example
  DESTINATION lib/${PROJECT_NAME}
)

# EXAMPLE 3
# The following two blocks will add an executable and a library for the ROS subscriber.
# In this case, this could have been done exactly like the publisher, but this provides a simple example.
# The most important use of this is that we can now reuse the subscriber code in other executables.

# add_library
#   similar to add_executable, we name the target and provide source files. There is one difference with the additional second
#   argument. This tells the compiler whether to make this a Shared or Static Library. We usually use Shared, and the difference
#   is not super important right now.
# ament_target_dependencies
#   same as executables
# ament_export_targets
#   This function makes the library available to other targets to use (by "linking" against them). This may only be needed
#   if using the library outside the package, but doesn't hurt to add.
# install
#   This serves the same purpose as installing an executable, but has additional arguments for a library target.
add_library(ros_subscriber_example SHARED src/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  ${DEPENDENCIES}
)
ament_export_targets(ros_subscriber_example HAS_LIBRARY_TARGET)
install(
  TARGETS ros_subscriber_example
  EXPORT ros_subscriber_example
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Here, we add the executable which runs the subscriber node.
# Notice that we used target_link_libraries to link the ros_subscriber_example.
add_executable(ros_subscriber_example_main src/ros_subscriber_example_main.cpp)
ament_target_dependencies(ros_subscriber_example_main
  ${DEPENDENCIES}
)
target_link_libraries(ros_subscriber_example_main
  ros_subscriber_example
)
install(TARGETS
  ros_subscriber_example_main
  DESTINATION lib/${PROJECT_NAME})

#################
##### Test ######
#################
# This is basically wrapping an "add_executable" call with some extra configuration.
# That way ROS knows its a test (and so it runs when we use colcon-test).
# Note:
#   Most GTests have the same main function, so you might see the test link to "gtest_main" instead of "gtest"
# and then not have a main function in the test file. They are equivalent.

ament_add_gtest(test_example test/test_example.cpp)
target_link_libraries(test_example
  ros_subscriber_example
  gtest
)

####################
##### Install ######
####################
# Just like we installed the targets, we will do the same for our header files and anything else we want to include.
# Typically if you download code from somewhere else, you won't get the source files, just the headers, binaries, and
# any extras (like the config files).

# This copies our install folder to ~/VEXU_GHOST/install/ghost_examples/include/
# Note:
#   The slash after include causes it to copy the contents of the folder and not the folder itself.
#   Without it, we would have "include/include" because we already have include as the DESTINATION.
install(
  DIRECTORY include/
  DESTINATION include
)

# This copies the launch/urdf/config folders to ~/VEXU_GHOST/install/ghost_examples/share/ghost_examples.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# This generates the ROS2 package and should always be the last line.
ament_package()
