/*
 * Filename: test_gazebo_swerve_plugin
 * Created Date: Monday July 18th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Monday July 18th 2022 11:41:53 am
 * Modified By: Maxx Wilson
 */

#include "gazebo_swerve_plugin.hpp"

#include "gtest/gtest.h"


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}