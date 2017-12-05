// "Copyright [2017] <Michael Kam>"
/** @file pclVoxel.h
 *  @brief header file of an pclVoxel class.
 *
 *  This class utilize pcl voxel_grid filter to down sample the point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */

#ifndef INCLUDE_PCLVOXEL_H_
#define INCLUDE_PCLVOXEL_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <iostream>
#include <ios>
using std::vector;
// using namespace pcl;
/** @brief pclVoxel is an implementation
 *  by using pcl voxel_grid filter to down sample the point cloud
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclVoxel {
 private:
  /**@brief dimension of leaf x */
  float lx;
  /**@brief dimension of leaf y */
  float ly;
  /**@brief dimension of leaf z */
  float lz;
  /**@brief object of PointXYZ */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  /**@brief shared pointer in which data type is PointXYZ  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

 public:
  /**@brief constructor */
  pclVoxel();
  /**@brief set a value into lx, ly, and lz
   * @param[in] lx set the private lx
   * @param[in] ly set the private ly
   * @param[in] lz set the private lz
   * @return none */
  void setLeafSize(float lx, float ly, float lz);
  /**@brief get value from lx, ly, and lz
   * @return vector<float> which contain value of above variables */
  vector<float> getLeafSize();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
  /**@brief start down sample the point cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
};


#endif  // INCLUDE_PCLVOXEL_H_
