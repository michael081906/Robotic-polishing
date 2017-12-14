// "Copyright [2017] <Michael Kam>"
/** @file pclFastTriangular.h
 *  @brief header file of an pclFastTriangular class.
 *
 *  This class utilizes pcl fastTriangular method to reconstruct the surface.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclFastTriangular is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclFastTriangular is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclFastTriangular.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_PCLFASTTRIANGULAR_H_
#define INCLUDE_PCLFASTTRIANGULAR_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <iostream>
#include <vector>
#include <ios>
// using namespace pcl;
/** @brief pclFastTriangular is an implementation by using pcl fastTriangular method to reconstruct the surface.
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclFastTriangular {
 private:
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer, which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;
  /**@brief GreedyProjectionTriangulation object */
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  /**@brief search range when computing the triangle */
  double searchRadius;

 public:
  /**@brief constructor */
  pclFastTriangular();
  /**@brief set a search range into searchRadius variable
   * @param[in] radius range of choosing
   * @return none */
  void setSearchRadius(double radius);
  /**@brief get a search range from searchRadius variable
   * @return search range  */
  double getSearchRadius();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudOut);
  /**@brief compute the triangles among the point cloud
   * @param[in] triangles reference of a polygonMesh
   * @return none */
  void reconctruct(pcl::PolygonMesh& triangles);
  /**@brief get triangle ID from each point cloud
   * @return vector<int> triangle ID set of point cloud */
  std::vector<int> getSegID();
};


#endif  // INCLUDE_PCLFASTTRIANGULAR_H_
