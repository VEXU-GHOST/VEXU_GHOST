/*
 * Filename: test_dc_motor_model
 * Created Date: Sunday July 17th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Maxx Wilson
 */

#include "COBS/COBS.hpp"

#include "gtest/gtest.h"

class TestCOBS: public ::testing::Test {
    protected:

    void SetUp() override {}
};

TEST_F(TestCOBS, testCOBSEncode){
    unsigned char input_buffer[] = {'t', 'e', '\x00', 's', 't'};
    unsigned char output_buffer[6] = {0,};

    COBS::cobsEncode(input_buffer, sizeof(input_buffer), output_buffer);

    unsigned char expected[] = {3, 't', 'e', 3, 's', 't'};


    for(int i = 0; i < sizeof(expected); i++){
        ASSERT_EQ(expected[i], output_buffer[i]);
    }
}

TEST_F(TestCOBS, testCOBSDecode){
    unsigned char input_buffer[] = {3, 't', 'e', 3, 's', 't'};
    unsigned char output_buffer[6] = {0,};

    COBS::cobsDecode(input_buffer, sizeof(input_buffer), output_buffer);

    unsigned char expected[] = {'t', 'e', '\x00', 's', 't'};


    for(int i = 0; i < sizeof(expected); i++){
        ASSERT_EQ(expected[i], output_buffer[i]);
    }
}

TEST_F(TestCOBS, testCOBSDecodeSerialData){
    unsigned char input_buffer[] = {0x05, 0x73, 0x6f, 0x75, 0x74, 2, 2, 1, 1, 0};
    unsigned char output_buffer[10] = {0,};

    int num = COBS::cobsDecode(input_buffer, sizeof(input_buffer), output_buffer);
    ASSERT_EQ(num, 9);
    unsigned char expected[] = {'s', 'o', 'u', 't', 0x00, 0x02, 0x00, 0x00};


    for(int i = 0; i < sizeof(expected); i++){
        ASSERT_EQ(expected[i], output_buffer[i]);
    }
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}