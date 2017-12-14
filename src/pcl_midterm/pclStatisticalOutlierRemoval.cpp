// "Copyright [2017] <Michael Kam>"
/** @file pclStatisticalOutlierRemoval.cpp
 *  @brief This is the implementation of the pclStatisticalOutlierRemoval class. This class
 *  consists of 7 methods. Please refer the pclStatisticalOutlierRemoval.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclStatistOutRev is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclStatistOutRev is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclStatistOutRev.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pclStatisticalOutlierRemoval.h"
#include <ios>
#include <iostream>
// using namespace pcl;
using std::vector;

pclStatistOutRev::pclStatistOutRev() {
  standDevMulThresh = 0;
  ktree = 0;
}
void pclStatistOutRev::setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn) {
  cloud = cloudIn;
}
void pclStatistOutRev::getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudOut = cloud;
}
void pclStatistOutRev::filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudPtr = cloud.makeShared();
  sor.setInputCloud(cloudPtr);
  sor.setMeanK(ktree);
  sor.setStddevMulThresh(standDevMulThresh);
  sor.filter(cloudOut);
}
int pclStatistOutRev::getMeanK() {
  return ktree;
}
void pclStatistOutRev::setMeanK(int setktree) {
  ktree = setktree;
}
double pclStatistOutRev::getStddevMulThresh() {
  return standDevMulThresh;
}
void pclStatistOutRev::setStddevMulThresh(double thresh) {
  standDevMulThresh = thresh;
}



