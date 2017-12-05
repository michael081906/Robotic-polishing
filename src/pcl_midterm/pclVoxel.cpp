// "Copyright [2017] <Michael Kam>"
/** @file pclVoxel.cpp
 *  @brief This is the implementation of the pclVoxel class. This class
 *  consists of 5 methods. Please refer the pclVoxel.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pclVoxel.h"
#include <iostream>
#include <vector>
#include <ios>
using std::vector;
// using namespace pcl;


void pclVoxel::filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  cloudPtr = cloud.makeShared();  //!!!!! Nice work!!!
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(lx, ly, lz);
  sor.filter(cloudOut);
}

void pclVoxel::getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudOut = cloud;
}

vector<float> pclVoxel::getLeafSize() {
  vector<float> leafvalue(3);
  leafvalue[0] = lx;
  leafvalue[1] = ly;
  leafvalue[2] = lz;
  return leafvalue;
}

pclVoxel::pclVoxel() {
  lx = 0;
  ly = 0;
  lz = 0;
}

void pclVoxel::setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn) {
  cloud = cloudIn;
}

void pclVoxel::setLeafSize(float setLx, float setLy, float setLz) {
  lx = setLx;
  ly = setLy;
  lz = setLz;
}
