#include "bankheader.h"
#include "gtest/gtest.h"

class BankAccountTest : public ::testing::Test

{
protected:
    BankAccountTest()
    {
        bankmaincode = std::make_shared<BankAccount>("kelly", 50000);
    }

    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
    std::shared_ptr<BankAccount> bankmaincode;

};

TEST_F(BankAccountTest, testWithdrawMoney)
{
    bankmaincode->withdraw(20000);
    EXPECT_EQ(bankmaincode->balance, 30000);
    EXPECT_TRUE(bankmaincode->balance == 30000);
    EXPECT_FALSE(bankmaincode->balance == 35000);
}

TEST_F(BankAccountTest, testPrint)
{
    testing::internal::CaptureStdout();
    bankmaincode->print();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_STREQ(output.c_str(), "kelly has a balance of 50000\n");
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}