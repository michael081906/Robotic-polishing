/*
 * delaunay3.h
 *
 *  Created on: Nov 16, 2017
 *      Author: viki
 */

#ifndef INCLUDE_DELAUNAY3_H_
#define INCLUDE_DELAUNAY3_H_

#include <stdlib.h>
#include <vector>
#include <set>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>
#include "s_hull_pro.h"

class delaunay3 {
 private:
  std::vector<Shx> pts;
  Shx pt;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
 public:
  delaunay3();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  void putPointCloudIntoShx();
  void processDelaunay(std::vector<Triad>& triads);
  void getShx(std::vector<Shx>& ptsOut);
};
#endif /* INCLUDE_DELAUNAY3_H_ */
