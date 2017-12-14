// "Copyright [2017] <Michael Kam>"
/** @file dijkstraPQ.h
 *  @brief This dijkstraPQ.cpp is a header file of finding shortest
 *  path among the point cloud. The algorithm refer to here:
 *  http://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  dijkstraPQ is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dijkstraPQ is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with dijkstraPQ.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef INCLUDE_DIJKSTRAPQ_H_
#define INCLUDE_DIJKSTRAPQ_H_
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <delaunay3.h>

typedef std::pair<int, double> iPair;

struct position {
  float x;
  float y;
  float z;
};

/** @brief kukaControl is a header file of finding shortest
 *  path among the point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 */
class dijkstraPQ {
 private:
  /**@brief size of the point cloud */
  int V;
  /**@brief pointer of parent */
  int *parent;
  /**@brief stored index and distance  */
  std::list<std::pair<int, double> > *adj;
  /**@brief vector to stored structure Triad */
  std::vector<Triad> triads;
  /**@brief object to stored triangle ID */
  std::vector<int> triPartID;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer, which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;

 public:
  /**constructor */
  dijkstraPQ(int size3);
  /**@brief add distance in to the weighting container
   * @param[in] u is an node index of a point
   * @param[in] v is an node index of another point
   * @param[in] w is the weighting value between point u and v.
   * @return none */
  void addEdge(int u, int v, double w);
  /**@brief start computing shortest distance by using
   * dijkstra algorithm.
   * @param[in] startNode is an node index of the start point
   * @param[in] endPoint is an node index of the end point
   * @return none */
  void shortestPath(int startNode, int endPoint);
  /**@brief request to calculate a specific edge distance in a triangle
   * @param[in] same3indices contains three node index of a traingle
   * @return none */
  void distanceCalculation(std::vector<int>& same3indices);
  /**@brief calculates distance of two points
   * @param[in] point1 is the first point index
   * @param[in] point2 is the second point index
   * @return none */
  double cal2Point(int point1, int point2);
  /**@brief set Tirad into private triads
   * @param[in] contains a sets of Triad
   * @return none */
  void setTri(std::vector<Triad>& triadsIn);
  /**@brief compute the deistance between two node
   * @param[in] none
   * @return none */
  void computeWeight();
  /**@brief return a set of node index on the  path
   * @param[in] startNode is an node index of the start point
   * @param[in] endNode is an node index of the end point
   * @param[out] pathNode is a container that stores a sets of node index
   * @return none */
  void returnDijkstraPath(int startNode, int endNode,
                          std::vector<int>& pathNode);
  /**@brief input a point cloud data and set it to the private cloud
   * @param[in] cloudIn reference of a point cloud
   * @return none */
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  /**@brief return the path position
   * @param[in] startNode is an node index of the start point
   * @param[in] endNode is an node index of the end point
   * @param[out] pathPos is a container that stores a sets of position
   * @return none */
  void returnDijkstraPathPosition(int startNode, int endNode,
                                  std::vector<position>& pathPos);
};



#endif /* INCLUDE_DIJKSTRAPQ_H_ */
