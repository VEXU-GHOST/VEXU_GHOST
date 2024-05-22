########################
### Compiler Options ###
########################
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_BUILD_TYPE "RelWDebInfo")

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O0)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#################################
### Ghost Compilation Options ###
#################################
add_compile_options(-DGHOST_DEBUG_VERBOSE)

# Device specific flag for code portability between Coprocessor and V5 Brain
add_definitions(-DGHOST_JETSON=1)
add_definitions(-DGHOST_V5_BRAIN=2)

add_definitions(-DGHOST_DEVICE=1)

##########################
### Autolinter Testing ###
##########################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()