// "Copyright [2017] <Michael Kam>"
/** @file pclCloudViewer.h
 *  @brief header file of an pclCloudViewer class.
 *
 *  This class utilize pcl visualization to display point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No1 can't not be compile successfully in the test case.
 *  @copyright GNU Public License.
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
