// "Copyright [2017] <Michael Kam>"
/** @file delaunay3.h
 *  @brief This delaunay3.h is a header file of the meshing triangle
 *  among the point cloud
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  delaunay3 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  delaunay3 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with delaunay3.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef INCLUDE_DELAUNAY3_H_
#define INCLUDE_DELAUNAY3_H_
#include <s_hull_pro.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <stdlib.h>
#include <vector>
#include <set>
#include <string>
/** @brief This delaunay3.h is a header file of the meshing triangle
 *  among the point cloud
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class delaunay3 {
 private:
  /**@brief container that stores Shx structure */
  std::vector<Shx> pts;
  /**@brief object of Shx structure */
  Shx pt;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
 public:
  delaunay3();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  /**@brief the function set the point cloud information in to pts
   * @param[in] none
   * @return none */
  void putPointCloudIntoShx();
  /**@brief the function processing the delaunay triangle meshing and return
   * the triads
   * @param[out] triads is a container that store the meshing results
   * @return none */
  void processDelaunay(std::vector<Triad>& triads);
  /**@brief the function return the private pts
   * @param[out] ptsOut is a container that store the structure Shx
   * @return none */
  void getShx(std::vector<Shx>& ptsOut);
};
#endif  // INCLUDE_DELAUNAY3_H_
