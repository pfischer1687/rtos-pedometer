/**
 * @file test_main.cpp
 * @brief Main function for running the unit tests.
 */

#include <gtest/gtest.h>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
