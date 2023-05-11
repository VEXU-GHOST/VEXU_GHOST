/*
 * Filename: test_mechanical_differential
 * Created Date: Thursday May 11th 2023
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 *
 * Last Modified: Thursday May 11th 2023 12:53:41 am
 * Modified By: Maxx Wilson
 */

#include "ghost_control/models/mechanical_differential.hpp"

#include "gtest/gtest.h"

using ghost_control::MechanicalDifferential;

class TestDifferential: public ::testing::Test {
    protected:

    void SetUp() override {
        differential = MechanicalDifferential(1/10, 1/2);
    }

    MechanicalDifferential differential;
};

TEST_F(TestDifferential, testTest){
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}