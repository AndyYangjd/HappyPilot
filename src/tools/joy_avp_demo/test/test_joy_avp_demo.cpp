#include "gtest/gtest.h"
#include "joy_avp_demo/joy_avp_demo.hpp"

TEST(test_joy_avp_demo, test_hello) {
  EXPECT_EQ(joypilot::joy_avp_demo::print_hello(), 0);
}
