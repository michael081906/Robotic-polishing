// "Copyright [2017] <Michael Kam>"
/** @file pclIo.cpp
 *  @brief This is the implementation of the pclIo class. This class consists of 2 methods.
 *  Please refer the pclIo.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclIo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclIo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclIo.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "pclIo.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
// using namespace pcl;

pclIo::pclIo() {
}

int pclIo::readPCDfile(const std::string& fileName) {
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
    return (-1);
  }
  return 0;
}

void pclIo::getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut) {
  cloudOut = cloud;
}
