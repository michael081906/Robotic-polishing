// "Copyright [2017] <Michael Kam>"
/** @file pclPassThroughTest.cpp
 *  @brief pclPassThroughTest.cpp consists of 2 unit test cases that test the
 *  pclPassThrough class.
 **
 *  TEST(pclPassThroughTest, setLimitValue) will test the setFilterXlimit() ,setFilterYlimit,
 *  setFilterZlimit(), and getFilterLimit() method.
 *  TEST(pclPassThroughTest, PassThroughFiltered) will test the filterProcess() method.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include "pclIo.h"
#include "pclPassThrough.h"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <iostream>

using std::vector;
// using namespace pcl;
/**@brief TEST(pclPassThroughTest, setLimitValue) will
 * test the setFilterXlimit() ,setFilterYlimit, setFilterZlimit(),
 * and getFilterLimit() method.*/
TEST(pclPassThroughTest, setLimitValue) {
  pclPassThrough pclpt;
  vector<float> passLimit(6);
  pclpt.setFilterXlimit(0, 1.0);
  pclpt.setFilterYlimit(5.0, 10.3);
  pclpt.setFilterZlimit(15.1, 20.3);
  passLimit = pclpt.getFilterLimit();
  EXPECT_NEAR(0, passLimit[0], 0.1);
  EXPECT_NEAR(1.0, passLimit[1], 0.1);
  EXPECT_NEAR(5.0, passLimit[2], 0.1);
  EXPECT_NEAR(10.3, passLimit[3], 0.1);
  EXPECT_NEAR(15.1, passLimit[4], 0.1);
  EXPECT_NEAR(20.3, passLimit[5], 0.1);
}
/**@brief TEST(pclPassThroughTest, PassThroughFiltered) will
 * test the filterProcess() method.*/
TEST(pclPassThroughTest, PassThroughFiltered) {
  pclPassThrough pclpt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // define five point cloud
  cloud->points[0].x = 3;
  cloud->points[0].y = 3;
  cloud->points[0].z = -1.2;
  cloud->points[1].x = 3;
  cloud->points[1].y = 3;
  cloud->points[1].z = 3;
  cloud->points[2].x = 3;
  cloud->points[2].y = 3;
  cloud->points[2].z = 0.1;
  cloud->points[3].x = 3;
  cloud->points[3].y = 3;
  cloud->points[3].z = 0.2;
  cloud->points[4].x = 3;
  cloud->points[4].y = 3;
  cloud->points[4].z = -4.0;
  pclpt.setInputCloud(*cloud);
  pclpt.setFilterZlimit(0.0, 1.0);
  pclpt.filterProcess(*cloudFiltered);
  int sizeFiltered = cloudFiltered->height * cloudFiltered->width;
  EXPECT_EQ(2, sizeFiltered);
}


