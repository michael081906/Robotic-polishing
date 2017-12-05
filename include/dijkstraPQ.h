/*
 * dijkstraPQ.h
 *
 *  Created on: Nov 22, 2017
 *      Author: viki
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
  int V;
  int *parent;
  std::list<std::pair<int, double> > *adj;
  std::vector<Triad> triads;
  std::vector<int> triPartID;
  /**@brief object of PointNormal */
  pcl::PointCloud<pcl::PointNormal> cloud;
  /**@brief shared pointer, which data type is PointNormal  */
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr;

 public:
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
