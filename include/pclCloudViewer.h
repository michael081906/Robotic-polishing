// "Copyright [2017] <Michael Kam>"
/** @file pclCloudViewer.h
 *  @brief header file of an pclCloudViewer class.
 *
 *  This class utilize pcl visualization to display point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No1 can't not be compile successfully in the test case.
 *  @copyright GNU Public License.
 *
 *  pclCloudViewer is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclCloudViewer is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclCloudViewer.  If not, see <http://www.gnu.org/licenses/>. *

 */

#ifndef INCLUDE_PCLCLOUDVIEWER_H_
#define INCLUDE_PCLCLOUDVIEWER_H_

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

/** @brief pclCloudViewer is an implementation by using pcl visualization to display the point cloud
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclCloudViewer {
 private:
  /** @brief accumulate counter during display  */
  int user_data;

 public:
  /**@brief constructor */
  pclCloudViewer();
  /**@brief display point cloud
   * @param[in] cloud reference of a point cloud
   * @return none */
  void display(const pcl::PolygonMesh &triangles);
};

#endif  // INCLUDE_PCLCLOUDVIEWER_H_
