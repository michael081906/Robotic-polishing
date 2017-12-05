// "Copyright [2017] <Michael Kam>"
/** @file obstacleIdentify.h
 *  @brief header file of an obstacleIdentify class.
 *
 *  This class utilizes point cloud normal to identify an obstacle.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#ifndef INCLUDE_OBSTACLEIDENTIFY_H_
#define INCLUDE_OBSTACLEIDENTIFY_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
// using namespace pcl;

/** @brief obstacleIdentify is an implementation by using point cloud normal to identify an obstacle.
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class obstacleIdentify {
 private:
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer in which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;
  /** @brief Threshold of the height of z */
  double zHeight;
  /** @brief Threshold of the normal vector of z */
  double normalZ;

 public:
  /**@brief constructor */
  obstacleIdentify();
  /**@brief set threshold of normal vector z
   * @param[in] zNormal threshold of normal vector of z
   * @return none */
  void setNormalZ(double zNormal);
  /**@brief get threshold of normal vector z
   * @return threshold of normal vector z*/
  double getNormalZ();
  /**@brief set threshold height of z
   * @param[in] height threshold height of z
   * @return none */
  void setZHeight(double height);
  /**@brief get threshold of height of z
   * @return threshold of height of z*/
  double getZHeight();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudOut);
  /**@brief identify the obstacle by checking z normal vector of every point and return obstacle point cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void process(pcl::PointCloud<pcl::PointNormal>& cloudOut);
};

#endif  // INCLUDE_OBSTACLEIDENTIFY_H_
