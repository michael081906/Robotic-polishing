// "Copyright [2017] <Michael Kam>"
/** @file pclPassThrough.cpp
 *  @brief This is the implementation of the pclPassThrough class. This class consists of 7 methods.
 *  Please refer the pclPassThrough.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclPassThroguh is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclPassThroguh is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclPassThroguh.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "pclPassThrough.h"
#include <vector>
#include <iostream>

// using namespace pcl;
using std::vector;

pclPassThrough::pclPassThrough() {
  zMin = -5000;
  zMax = 5000;
  yMin = -5000;
  yMax = 5000;
  xMin = -5000;
  xMax = 5000;
}
void pclPassThrough::setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn) {
  cloud = cloudIn;
}
void pclPassThrough::getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudOut = cloud;
}

void pclPassThrough::setFilterXlimit(const float& setXMin,
                                     const float& setXMax) {
  xMax = setXMax;
  xMin = setXMin;
  pass.setFilterFieldName("x");
  pass.setFilterLimits(xMin, xMax);
}
void pclPassThrough::setFilterYlimit(const float& setYMin,
                                     const float& setYMax) {
  yMax = setYMax;
  yMin = setYMin;
  pass.setFilterFieldName("y");
  pass.setFilterLimits(yMin, yMax);
}
void pclPassThrough::setFilterZlimit(const float& setZMin,
                                     const float& setZMax) {
  zMax = setZMax;
  zMin = setZMin;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zMin, zMax);
}
void pclPassThrough::filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudPtr = cloud.makeShared();
  pass.setInputCloud(cloudPtr);
  pass.filter(cloudOut);
}

vector<float> pclPassThrough::getFilterLimit() {
  vector<float> limitValue(6);
  limitValue[0] = xMin;
  limitValue[1] = xMax;
  limitValue[2] = yMin;
  limitValue[3] = yMax;
  limitValue[4] = zMin;
  limitValue[5] = zMax;
  return limitValue;
}

