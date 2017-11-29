/*
 * mergeHanhsPoint.h
 *
 *  Created on: Oct 24, 2017
 *      Author: viki
 */

#ifndef INCLUDE_FINDNEARESTPOINT_H_
#define INCLUDE_FINDNEARESTPOINT_H_
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>
class findNearestPoint {

 private:
  std::vector<double> specificPoints;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer, which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;

 public:
  /**constructor */
  findNearestPoint();
  /**@brief readtext() method reads .txt file, storing the coordinate and push
   * back into current point cloud
   * @param[in] &filename reference of the filename
   * @return a vector that store the value read from .txt*/
  std::vector<double> readtext(const std::string &filename);
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  /**@brief this function calculate the nearest point in the given pointNormal.
   * Before using this function, make sure you have setInpuCloud and a specificPoints
   * instance.
   * This method refer to http://pointclouds.org/documentation/tutorials/kdtree_search.php
   * @param[in] none
   * @return none */
  void findNearestProcess(std::vector<int>& nearIndices);



};




#endif /* INCLUDE_FINDNEARESTPOINT_H_ */
