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
  // function to add an edge to graph
  void addEdge(int u, int v, double w);
  // prints shortest path from s
  void shortestPath(int startNode, int endPoint);
  void distanceCalculation(std::vector<int>& same3indices);
  double cal2Point(int point1, int point2);
  void setTri(std::vector<Triad>& triadsIn);
  void computeWeight();
  void returnDijkstraPath(int startNode, int endNode,
                          std::vector<int>& pathNode);
  void setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn);
  void returnDijkstraPathPosition(int startNode, int endNode,
                                  std::vector<position>& pathPos);
};



#endif /* INCLUDE_DIJKSTRAPQ_H_ */
