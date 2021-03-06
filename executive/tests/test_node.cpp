// Bring in my package's API, which is what I'm testing
#include "executive/chief_executive.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(ChiefExecutiveClass, settingDuty)
{
    
}

// Declare another test
TEST(TestSuite, testCase2)
{
<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}