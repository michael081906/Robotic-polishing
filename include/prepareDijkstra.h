/*
 * prepareDijkstra.h
 *
 *  Created on: Oct 30, 2017
 *      Author: viki
 */

#ifndef INCLUDE_PREPAREDIJKSTRA_H_
#define INCLUDE_PREPAREDIJKSTRA_H_
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <delaunay3.h>

// #include <pcl/io/pcd_io.h>
class prepareDijkstra {
 private:
  double** weight;
  std::vector<Triad> triads;
  std::vector<int> triPartID;
  int vertexCount;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer, which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;
 public:
  prepareDijkstra(std::vector<int>& gp3PartID);
  /**@brief weightCalculation() is a function that calculate the distance between point cloud
   * and puts the result into weight. The function will first find same triangle indices. Second,
   * the function will use the return result of the first step and calculate the distance,
   * filling in into the weight.
   * @retun none.
   */
  void weightCalculation();
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  void getWeight(double** &weightOut);
  double cal2Point(int point1, int point2);
  // TODO: Learn how to use Google Mock so that I can put some classes as private and test them.
  void distanceCalculation(std::vector<int>& same3indices);
  /**@brief findSameID is a function which find the given value, indexOfgp3, in the triPartID,
   * returning a vector that contains index of the found value.
   * @param[in] indexOfgp3 is a value to be found.
   * @return vector<int> contains index of found values in the triPartID.
   */
  std::vector<int> findSameID(int indexOfgp3);

  void setTri(std::vector<Triad>& triadsIn);
  void computeWeight();


};


#endif /* INCLUDE_PREPAREDIJKSTRA_H_ */
