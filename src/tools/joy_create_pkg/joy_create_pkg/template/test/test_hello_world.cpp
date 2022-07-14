#include "gtest/gtest.h"
#include "hello_world/hello_world.hpp"

TEST(test_hello_world, test_hello) {
  EXPECT_EQ(joypilot::hello_world::print_hello(), 0);
}
