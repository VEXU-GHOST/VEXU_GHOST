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
