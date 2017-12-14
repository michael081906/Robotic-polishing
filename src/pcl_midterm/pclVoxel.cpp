// "Copyright [2017] <Michael Kam>"
/** @file pclVoxel.cpp
 *  @brief This is the implementation of the pclVoxel class. This class
 *  consists of 5 methods. Please refer the pclVoxel.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclVoxel is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclVoxel is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclVoxel.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
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
