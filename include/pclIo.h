// "Copyright [2017] <Michael Kam>"
/** @file pclIo.h
 *  @brief header file of an pclIo class.
 *
 *  This class utilizes pcl to load .pcd file from local.
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
#ifndef INCLUDE_PCLIO_H_
#define INCLUDE_PCLIO_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>


/** @brief pclIo is an implementation by using pcl to load .pcd file from local.
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclIo {
 private:
  /**@brief object of PointXYZ */
  pcl::PointCloud<pcl::PointXYZ> cloud;

 public:
  /**constructor */
  pclIo();
  /**@brief read pcd file
   * @param[in] fileName reference of a file string
   * @return none */
  int readPCDfile(const std::string& fileName);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
};

#endif  // INCLUDE_PCLIO_H_
