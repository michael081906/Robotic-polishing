// "Copyright [2017] <Michael Kam>"
/** @file pclStatistOutRev.h
 *  @brief header file of an pclStatistOutRev class.
 *
 *  This class utilize pcl pclStatistOutlierRemoval filter to filter noise in point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#ifndef INCLUDE_PCLSTATISTICALOUTLIERREMOVAL_H_
#define INCLUDE_PCLSTATISTICALOUTLIERREMOVAL_H_
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>
#include <iostream>
#include <ios>
// using namespace pcl;
using std::vector;
/** @brief pclStatistOutRev is an implementation
 *  by using pcl pclStatistOutlierRemoval filter to filter noise in point cloud
 * *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class pclStatistOutRev {
 private:
  /**@brief object of StatisticalOutlierRemoval in which data
   * type is PointXYZ  */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  /**@brief object of PointXYZ */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  /**@brief shared pointer in which data type is PointXYZ  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
  /** @brief how many search point when filtering  */
  int ktree;
  /** @brief standard deviation of the filter threshold  */
  double standDevMulThresh;

 public:
  /**@brief constructor */
  pclStatistOutRev();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudIn);
  /**@brief get a point cloud data from private cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void getInputCloud(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
  /**@brief start to filter point cloud
   * @param[in] cloudOut reference of a point cloud
   * @return none */
  void filterProcess(pcl::PointCloud<pcl::PointXYZ>& cloudOut);
  /**@brief get a value of private ktree
   * @return int value of private ktree  */
  int getMeanK();
  /**@brief set a value into private ktree
   * @param[in] setktree value to set ktree
   * @return none */
  void setMeanK(int setktree);
  /**@brief get a value of private standDevMulThresh
   * @return double value of private standDevMulThresh  */
  double getStddevMulThresh();
  /**@brief set a value into private standDevMulThresh
   * @param[in] Thresh value to set standDevMulThresh
   * @return none */
  void setStddevMulThresh(double Thresh);
};

#endif  // INCLUDE_PCLSTATISTICALOUTLIERREMOVAL_H_ */
