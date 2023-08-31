cmake_minimum_required(VERSION 2.8.11)

PROJECT(amrl-shared-lib)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(LIBS glog pthread)

SET(AMRL_LIBRARY_NAME "amrl_shared_lib"
    CACHE STRING "Name of compiled library")
SET(AMRL_LIB_UNITTEST_NAME "amrl_shared_lib_tests"
    CACHE STRING "Name of compiled unit test executable")
OPTION(GENERATE_SHARED_LIB_UNITTESTS,
       "Option to enable generation of shared library unit tests"
       OFF)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")

ADD_LIBRARY(${AMRL_LIBRARY_NAME}
            util/helpers.cc
            util/pthread_utils.cc
            util/timer.cc
            util/random.cc
            util/serialization.cc
            util/terminal_colors.cc)
TARGET_LINK_LIBRARIES(${AMRL_LIBRARY_NAME} ${LIBS})

IF(GENERATE_SHARED_LIB_UNITTESTS)
  ADD_EXECUTABLE(${AMRL_LIB_UNITTEST_NAME}
                tests/math/line2d_tests.cc
                tests/math/math_tests.cc
                tests/util/kdtree_tests.cc)
  TARGET_LINK_LIBRARIES(${AMRL_LIB_UNITTEST_NAME}
                        ${AMRL_LIBRARY_NAME}
                        gtest
                        gtest_main
                        ${LIBS})
ENDIF()
