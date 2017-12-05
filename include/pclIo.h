// "Copyright [2017] <Michael Kam>"
/** @file pclIo.h
 *  @brief header file of an pclIo class.
 *
 *  This class utilizes pcl to load .pcd file from local.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
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
