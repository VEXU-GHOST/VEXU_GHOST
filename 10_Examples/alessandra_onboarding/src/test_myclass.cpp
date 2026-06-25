#include "MyClass.h"
#include "gtest/gtest.h"

TEST(myclasstest, values) {
    
    MyClass obj;

    ASSERT_EQ(0, obj.getValue());
    ASSERT_EQ("empty", obj.getName());
}

TEST(myclasstest, addingby2) {
    MyClass obj(5, "initial");

    obj.addValue();
    ASSERT_EQ(7, obj.getValue());
}

TEST(myclasstest, namechange) {
    MyClass obj(0, "initialname");

    obj.changeName("difname");
    ASSERT_EQ("difname", obj.getName());
}