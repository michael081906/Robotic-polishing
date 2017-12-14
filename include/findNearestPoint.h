// "Copyright [2017] <Michael Kam>"
/** @file findNearestPoint.cpp
 *  @brief This findNearestPoint.h is a header file of finding
 *   the closest point on a post smoothing surface. There are three method in this class.
 *   The goal is to get indices array which indicate the closest point with several given points.
 *   For example, I can given a arbitrary coordinate and use this class to
 *   find a closest point ID from point cloud group.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  findNearestPoint is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  findNearestPoint is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with findNearestPoint.  If not, see <http://www.gnu.org/licenses/>.
 *
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
  /**@brief vector of storing specificPoints */
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
   * This method was referred to http://pointclouds.org/documentation/tutorials/kdtree_search.php
   * @param[in] nearIndices
   * @return none */
  void findNearestProcess(std::vector<int>& nearIndices);
  /**@brief this function takes pos and stores it into private variable specificPoints
   * @param[in] pos position you want it to be search for its nearest point
   * @return none */
  void setPosition(const std::vector<float>& pos);
};

#endif /* INCLUDE_FINDNEARESTPOINT_H_ */
