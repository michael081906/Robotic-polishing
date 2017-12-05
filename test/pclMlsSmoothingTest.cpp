// "Copyright [2017] <Michael Kam>"
/** @file pclMlsSmoothingTest.cpp
 *  @brief pclMlsSmoothingTest.cpp consists of 3 unit test cases that test the
 *  pclMlsSmoothing class.
 **
 *  TEST(pclMlsSmoothingTest, setRadius) will test the setSearchRadius() and getSearchRadius() method.
 *  TEST(pclMlsSmoothingTest, setpclCloud) will test the getInputCloud() and setInputCloud() method.
 *  TEST(pclMlsSmoothingTest, MlsFiltering) will test the mlsProcess().
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <gtest/gtest.h>
#include <pclMlsSmoothing.h>
using std::cout;
using std::endl;
/** @brief TEST(pclMlsSmoothingTest, setRadius) will
 * test the setSearchRadius() and getSearchRadius() method */
TEST(pclMlsSmoothingTest, setRadius) {
  pclMlsSmoothing mlsing;
  double radiusOut;
  mlsing.setSearchRadius(20.32);
  radiusOut = mlsing.getSearchRadius();
  EXPECT_NEAR(20.32, radiusOut, 0.1);
}
/** @brief TEST(pclMlsSmoothingTest, setpclCloud) will
 * test the getInputCloud() and setInputCloud() method */
TEST(pclMlsSmoothingTest, setpclCloud) {
  pclMlsSmoothing mlsing;
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
  mlsing.setInputCloud(*cloud);
  mlsing.getInputCloud(*cloudFiltered);
  int originSize = cloud->width * cloud->height;
  int getInputSize = cloudFiltered->width * cloudFiltered->height;
  EXPECT_EQ(originSize, getInputSize);
  EXPECT_NEAR(-1.2, cloudFiltered->points[0].z, 0.1);
}
/** @brief TEST(pclMlsSmoothingTest, MlsFiltering) will
 * test the mlsProcess() */
TEST(pclMlsSmoothingTest, MlsFiltering) {
  pclMlsSmoothing mlsing;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointNormal>);
  cloud->width = 9;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // define point cloud
  cloud->points[0].x = 0;
  cloud->points[0].y = 0;
  cloud->points[0].z = 0;
  cloud->points[1].x = 1;
  cloud->points[1].y = 0;
  cloud->points[1].z = 0;
  cloud->points[2].x = 2;
  cloud->points[2].y = 0;
  cloud->points[2].z = 0;
  cloud->points[3].x = 0;
  cloud->points[3].y = 1;
  cloud->points[3].z = 0;
  cloud->points[4].x = 1;
  cloud->points[4].y = 1;
  cloud->points[4].z = 1.4;
  cloud->points[5].x = 2;
  cloud->points[5].y = 1;
  cloud->points[5].z = 0;
  cloud->points[6].x = 0;
  cloud->points[6].y = 2;
  cloud->points[6].z = 0;
  cloud->points[7].x = 1;
  cloud->points[7].y = 2;
  cloud->points[7].z = 0;
  cloud->points[8].x = 2;
  cloud->points[8].y = 2;
  cloud->points[8].z = 0;
  mlsing.setInputCloud(*cloud);
  mlsing.setSearchRadius(8.0);
  mlsing.mlsProcess(*cloudFiltered);
  double testZ = cloudFiltered->points[4].z;
  // cloudFiltered->points[4].normal;
  // cout << testZ << endl;
  EXPECT_GT(1.4, testZ);
}



