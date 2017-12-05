// "Copyright [2017] <Michael Kam>"
/** @file pclStatisticalOutlierRemoval.cpp
 *  @brief This is the implementation of the pclStatisticalOutlierRemoval class. This class
 *  consists of 7 methods. Please refer the pclStatisticalOutlierRemoval.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
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



