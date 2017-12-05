// "Copyright [2017] <Michael Kam>"
/** @file pclVoxelTest.cpp
 *  @brief pclVoxelTest.cpp consists of 3 unit test cases that test the
 *  pclVoxel class.
 **
 *  TEST(pclVoxelTest, setLeafValue) will test the setLeafSize() and getLeafSize() method.
 *  TEST(pclVoxelTest, pointCloudDownSampleing) will test the filterProcess() method.
 *  TEST(pclVoxelTest, setpclCloud) will test the getInputCloud() and setInputCloud() method.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include "pclVoxel.h"
#include "pclIo.h"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
/**@brief TEST(pclVoxelTest, setLeafValue) will
 * test the setLeafSize() and getLeafSize() method.*/
TEST(pclVoxelTest, setLeafValue) {
  pclVoxel pclVoxel;
  vector<float> leafSize(3);
  pclVoxel.setLeafSize(0.5, 0.4, 0.7);
  leafSize = pclVoxel.getLeafSize();
  EXPECT_NEAR(0.5, leafSize[0], 0.1);
  EXPECT_NEAR(0.4, leafSize[1], 0.1);
  EXPECT_NEAR(0.7, leafSize[2], 0.1);
}

/**@brief TEST(pclVoxelTest, pointCloudDownSampleing)
 * will test the filterProcess() method.*/
TEST(pclVoxelTest, pointCloudDownSampleing) {
  pclIo pclLoad;
  pclVoxel pclVoxel;
  int originSize = 0;
  int filteredSize = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pclLoad.readPCDfile(
      "../PCL_Matlab_Result__ROI_Smoothing_ascii.pcd");
  pclLoad.getPointCloud(*cloud);
  originSize = cloud->height * cloud->width;
  pclVoxel.setInputCloud(*cloud);
  pclVoxel.setLeafSize(0.5, 0.4, 0.7);
  pclVoxel.filterProcess(*cloudFiltered);
  filteredSize = cloudFiltered->height * cloudFiltered->width;
  EXPECT_LT(filteredSize, originSize);
}
/**@brief TEST(pclVoxelTest, setpclCloud) will
 * test the getInputCloud() and setInputCloud() method.*/
TEST(pclVoxelTest, setpclCloud) {
  pclVoxel pclVoxel;
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
  pclVoxel.setInputCloud(*cloud);
  pclVoxel.getInputCloud(*cloudFiltered);
  int originSize = cloud->width * cloud->height;
  int getInputSize = cloudFiltered->width * cloudFiltered->height;
  EXPECT_EQ(originSize, getInputSize);
  EXPECT_NEAR(-1.2, cloudFiltered->points[0].z, 0.1);
}
