#include "gtest/gtest.h"
#include <thread>

#include "stamped_queue/stamped_queue.hpp"

using stamped_queue::StampedQueue;
using std::chrono::system_clock;

using namespace std::chrono_literals;

class TestStampedQueue: public ::testing::Test {
    protected:

    void SetUp() override {
    }
};

TEST_F(TestStampedQueue, testDefaultConstructor){
    bool success = true;
    try{
        auto queue = StampedQueue<double>();
    }
    catch(...){
        success = false;
    }
    ASSERT_EQ(success, true);
}

TEST_F(TestStampedQueue, testConstructor){
    bool success = true;
    try{
        auto queue = StampedQueue<double>(1000ms);
    }
    catch(...){
        success = false;
    }
    ASSERT_EQ(success, true);
}

TEST_F(TestStampedQueue, testAddValue){
    bool success = true;
    try{
        auto queue = StampedQueue<double>();
        queue.AddValue(system_clock::now(), 10);
    }
    catch(...){
        success = false;
    }
    ASSERT_EQ(success, true);
}

TEST_F(TestStampedQueue, testGetHistory){
    auto queue = StampedQueue<double>();
    auto t1 = system_clock::now();
    std::this_thread::sleep_for(250ms);
    auto t2 = system_clock::now();

    queue.AddValue(t1, 10);
    queue.AddValue(t2, 11);

    // Get history structure and test values
    auto history = queue.GetHistory();
    ASSERT_EQ(history[0].first, t1);
    ASSERT_EQ(history[0].second, 10);

    ASSERT_EQ(history[1].first, t2);
    ASSERT_EQ(history[1].second, 11);
}

TEST_F(TestStampedQueue, testHistoryIsChronological){
    auto queue = StampedQueue<double>(1000ms);
    auto t1 = system_clock::now();
    std::this_thread::sleep_for(250ms);
    auto t2 = system_clock::now();

    queue.AddValue(t2, 11);
    queue.AddValue(t1, 10);

    // Get history structure and test values
    auto history = queue.GetHistory();
    ASSERT_EQ(history[0].first, t1);
    ASSERT_EQ(history[0].second, 10);

    ASSERT_EQ(history[1].first, t2);
    ASSERT_EQ(history[1].second, 11);
}

TEST_F(TestStampedQueue, testOldMessagesExpire){
    auto queue = StampedQueue<double>(250ms);
    auto t1 = system_clock::now();
    queue.AddValue(t1, 10);

    std::this_thread::sleep_for(500ms);
    
    auto t2 = system_clock::now();
    queue.AddValue(t2, 11);

    // Get history structure and test values
    auto history = queue.GetHistory();
    ASSERT_EQ(history[0].first, t2);
    ASSERT_EQ(history[0].second, 11);
}

TEST_F(TestStampedQueue, testGetHistoryThreadSafe){
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}