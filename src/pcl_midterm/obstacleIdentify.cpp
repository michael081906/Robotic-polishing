// "Copyright [2017] <Michael Kam>"
/** @file obstacleIdentify.cpp
 *  @brief This is the implementation of the obstacleIdentify class. This class consists of 7 methods.
 *  Please refer the obstacleIdentify.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <obstacleIdentify.h>

  obstacleIdentify::obstacleIdentify() {
  normalZ = 0;
  zHeight = 0;
}
void obstacleIdentify::setNormalZ(double zNormal) {
  normalZ = zNormal;
}
double obstacleIdentify::getNormalZ() {
  return normalZ;
}
void obstacleIdentify::setZHeight(double height) {
  zHeight = height;
}
double obstacleIdentify::getZHeight() {
  return zHeight;
}
void obstacleIdentify::setInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}
void obstacleIdentify::getInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudOut) {
  cloudOut = cloud;
}
void obstacleIdentify::process(pcl::PointCloud<pcl::PointNormal>& cloudOut) {
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    // obstacle
    if (cloud.points[i].normal_z > normalZ) {  // mark as obstacle
      cloudOut.push_back(cloud.points[i]);
    }
  }
}




