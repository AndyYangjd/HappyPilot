#include "gtest/gtest.h"
#include "carla_interface/carla_interface.hpp"

TEST(test_carla_interface, test_hello) {
  EXPECT_EQ(joypilot::carla_interface::print_hello(), 0);
}
