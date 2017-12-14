// "Copyright [2017] <Michael Kam>"
/** @file pclPassThroguh.h
 *  @brief header file of an pclPassThroguh class.
 *
 *  This class utilize pcl passthrough filter to extract certain region of point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  pclPassThroguh is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclPassThroguh is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclPassThroguh.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef INCLUDE_PCLPASSTHROUGH_H_
#define INCLUDE_PCLPASSTHROUGH_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <vector>
#include <ios>
// using namespace pcl;
using std::vector;
/** @brief pclPassThrough is an implementation by using pcl
 * passthrough filter to extract certain region of point cloud.
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclPassThrough {
 private:
  /**@brief object of PointXYZ */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  /**@brief shared pointer in which data type is PointXYZ  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
  /**@brief object of passThrough in which data type is PointXYZ  */
  pcl::PassThrough<pcl::PointXYZ> pass;
  /** @brief Threshold of the minimum z */
  double zMin;
  /** @brief Threshold of the maximum z */
  double zMax;
  /** @brief Threshold of the minimum y */
  double yMin;
  /** @brief Threshold of the maximum y */
  double yMax;
  /** @brief Threshold of the minimum x */
  double xMin;
  /** @brief Threshold of the maximum x */
  double xMax;

 public:
  /**@brief constructor */
  pclPassThrough();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
  /**@brief set value into xMin and xMax
   * @param[in] setXMin reference of a value to set xMin
   * @param[in] setXMax reference of a value to set xMax
   * @return none */
  void setFilterXlimit(const float& setXMin, const float& setXMax);
  /**@brief set value into yMin and yMax
   * @param[in] setYMin reference of a value to set yMin
   * @param[in] setYMax reference of a value to set yMax
   * @return none */
  void setFilterYlimit(const float& setYMin, const float& setYMax);
  /**@brief set value into zMin and zMax
   * @param[in] setZMin reference of a value to set zMin
   * @param[in] setZMax reference of a value to set zMax
   * @return none */
  void setFilterZlimit(const float& setZMin, const float& setZMax);
  /**@brief get value from zMin, zMax, yMin, yMax, xMin, xMax
   * @return vector<float> which contain six value of above variables */
  vector<float> getFilterLimit();
};

#endif  // INCLUDE_PCLPASSTHROUGH_H_
