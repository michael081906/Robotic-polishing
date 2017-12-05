// "Copyright [2017] <Michael Kam>"
/** @file pclIoTest.cpp
 *  @brief pclIoTest.cpp consists of 2 unit test cases that test the
 *  pclIo class.
 **
 *  TEST(pclIoTest, loadPCDfileMustFail) will test the readPCDfile() method.
 *  TEST(pclIoTest, loadPCDfileShowResult) will test the getPointCloud() method.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */

#include "pclIo.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <gtest/gtest.h>
#include <iostream>

/** @brief TEST(pclIoTest, loadPCDfileMustFail) will
 * test the readPCDfile() method*/
TEST(pclIoTest, loadPCDfileMustFail) {
  pclIo pclLoad;
  int load = pclLoad.readPCDfile("no_such_file.pcd");
  EXPECT_NEAR(-1, load, 0.5);
}
/** @brief TEST(pclIoTest, loadPCDfileShowResult) will
 * test the getPointCloud() method*/
TEST(pclIoTest, loadPCDfileShowResult) {
  pclIo pclLoad;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pclLoad.readPCDfile("../pclIo_test.pcd");
  pclLoad.getPointCloud(*cloud);
  EXPECT_NEAR(10, cloud->points[0].x, 0.5);
  EXPECT_NEAR(20, cloud->points[0].y, 0.5);
  EXPECT_NEAR(30, cloud->points[0].z, 0.5);
}

