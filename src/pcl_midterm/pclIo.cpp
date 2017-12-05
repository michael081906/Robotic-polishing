// "Copyright [2017] <Michael Kam>"
/** @file pclIo.cpp
 *  @brief This is the implementation of the pclIo class. This class consists of 2 methods.
 *  Please refer the pclIo.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
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
