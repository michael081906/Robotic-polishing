// "Copyright [2017] <Michael Kam>"
/** @file pclStatisticalOutlierRemovalTest.cpp
 *  @brief pclStatisticalOutlierRemovalTest.cpp consists of 3 unit test cases that test the
 *  pclStatisticalOutlierRemoval class.
 **
 *  TEST(pclStatisticalOutlierRemovalTest, SetMeankAndThresh) will test the getMeanK(),
 *  setStddevMulThresh(), setMeanK(), and getStddevMulThresh() method.
 *  TEST(pclStatisticalOutlierRemovalTest, SetPointCloud) will test the getInputCloud() and setInputCloud() method.
 *  TEST(pclStatisticalOutlierRemovalTest, SORFilterTest) will test the filterProcess() method.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include "pclStatisticalOutlierRemoval.h"
#include <iostream>
#include <vector>
using std::vector;
// using namespace pcl;
/**@ brief TEST(pclStatisticalOutlierRemovalTest,
 * SetMeankAndThresh) will test the getMeanK(),
 * setStddevMulThresh(), setMeanK(), and getStddevMulThresh() method.*/
TEST(pclStatisticalOutlierRemovalTest, SetMeankAndThresh) {
  pclStatistOutRev sor;
  sor.setMeanK(4);
  int meanK = sor.getMeanK();
  sor.setStddevMulThresh(1.2);
  double thresh = sor.getStddevMulThresh();
  EXPECT_EQ(4, meanK);
  EXPECT_EQ(1.2, thresh);
}

/**@brief TEST(pclStatisticalOutlierRemovalTest,
 * SetPointCloud) will test the getInputCloud() and
 * setInputCloud() method. */
TEST(pclStatisticalOutlierRemovalTest, SetPointCloud) {
  pclStatistOutRev sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 2;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // define point cloud
  cloud->points[0].x = 3;
  cloud->points[0].y = 0.2;
  cloud->points[0].z = -1.2;
  cloud->points[1].x = 1.2;
  cloud->points[1].y = 2.3;
  cloud->points[1].z = 1.4;
  sor.setInputCloud(*cloud);
  sor.getInputCloud(*cloudFiltered);
  int originSize = cloud->width * cloud->height;
  int getInputSize = cloudFiltered->width * cloudFiltered->height;
  EXPECT_EQ(originSize, getInputSize);
  EXPECT_NEAR(-1.2, cloudFiltered->points[0].z, 0.1);
}
/**@brief TEST(pclStatisticalOutlierRemovalTest, SORFilterTest)
 * will test the filterProcess() method. */
TEST(pclStatisticalOutlierRemovalTest, SORFilterTest) {
  pclStatistOutRev sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZ>);
  // generate point cloud
  cloud->width = 20;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }
  cloud->points[19].x = 40;
  cloud->points[19].y = 21;
  cloud->points[19].z = 12;
  sor.setInputCloud(*cloud);
  sor.setMeanK(30);
  sor.setStddevMulThresh(1.0);
  sor.filterProcess(*cloudFiltered);
  int size = cloudFiltered->points.size();
  EXPECT_EQ(19, size);
}
