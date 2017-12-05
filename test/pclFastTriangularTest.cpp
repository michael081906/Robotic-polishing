// "Copyright [2017] <Michael Kam>"
/** @file pclFastTriangularTest.cpp
 *  @brief pclFastTriangularTest.cpp consists of 3 unit test cases that test the
 *  pclFastTriangular class.
 **
 *  TEST(pclFastTriangularTest, setRadius) will test the setSearchRadius() and getSearchRadius() method.
 *  TEST(pclFastTriangularTest, setpclCloud) will test the setInputCloud() and getInputCloud() method.
 *  TEST(pclFastTriangularTest, TriangularMesh) will test the reconctruct() method.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <gtest/gtest.h>
#include <pclFastTriangular.h>
#include <pclMlsSmoothing.h>
#include <vector>
/**@brief TEST(pclFastTriangularTest, setRadius) will
 * test the setSearchRadius() and getSearchRadius() method*/
TEST(pclFastTriangularTest, setRadius) {
  pclFastTriangular ft;
  double radiusOut;
  ft.setSearchRadius(20.32);
  radiusOut = ft.getSearchRadius();
  EXPECT_NEAR(20.32, radiusOut, 0.1);
}
/**@brief TEST(pclFastTriangularTest, setpclCloud) will test
 * the setInputCloud() and getInputCloud() method*/
TEST(pclFastTriangularTest, setpclCloud) {
  pclFastTriangular ft;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointNormal>);
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
  ft.setInputCloud(*cloud);
  ft.getInputCloud(*cloudFiltered);
  int originSize = cloud->width * cloud->height;
  int getInputSize = cloudFiltered->width * cloudFiltered->height;
  EXPECT_EQ(originSize, getInputSize);
  EXPECT_NEAR(-1.2, cloudFiltered->points[0].z, 0.1);
}
/**@brief TEST(pclFastTriangularTest, TriangularMesh) will
 * test the reconctruct() method*/
TEST(pclFastTriangularTest, TriangularMesh) {
  pclFastTriangular ft;
  pclMlsSmoothing mlsing;
  pcl::PolygonMesh triangles;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(
      new pcl::PointCloud<pcl::PointNormal>);
  cloud->width = 3;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  // define point cloud
  cloud->points[0].x = 3;
  cloud->points[0].y = 0;
  cloud->points[0].z = 0;
  cloud->points[1].x = 0;
  cloud->points[1].y = 0;
  cloud->points[1].z = 0;
  cloud->points[2].x = 0;
  cloud->points[2].y = 3;
  cloud->points[2].z = 0;
  mlsing.setInputCloud(*cloud);
  mlsing.setSearchRadius(8.0);
  mlsing.mlsProcess(*cloud_with_normal);
  double testX = cloud_with_normal->points[0].x;
  double testY = cloud_with_normal->points[2].y;
  ASSERT_NEAR(3, testX, 0.1);
  ASSERT_NEAR(3, testY, 0.1);
  ft.setSearchRadius(5);
  ft.setInputCloud(*cloud_with_normal);
  ft.reconctruct(triangles);
  std::vector<int> segID;
  segID = ft.getSegID();
  // std::cout << segID[0] << std::endl;
  // std::cout << segID[1] << std::endl;
  // std::cout << segID[2] << std::endl;
  ASSERT_NEAR(0, segID[0], 0.1);
  ASSERT_NEAR(0, segID[1], 0.1);
  ASSERT_NEAR(0, segID[2], 0.1);
}
