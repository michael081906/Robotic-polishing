// "Copyright [2017] <Michael Kam>"
/** @file main.cpp
 *  @brief main test file of this project.
 **
 *  The cpp-test start entering this file.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <gtest/gtest.h>
#include <ros/console.h>
#include "ros/ros.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
