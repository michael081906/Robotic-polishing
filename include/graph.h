/*
 * graph.h
 *
 *  Created on: Nov 1, 2017
 *      Author: viki
 */

#ifndef INCLUDE_GRAPH_H_
#define INCLUDE_GRAPH_H_
#include <vector>

class Graph {
 private:
  double **adjMatrix;
  int vertexCount;
  // below attributes are for Prims
  int *key;
  double *distance;
  int *parent;

 public:
  Graph(int vertexCount);
  Graph(int vertexCount, double** &weightMatrix);
  ~Graph();
  void addEdgeDirectedWeight(int i, int j, double cost);
  void removeEdgeUndirected(int i, int j);
  double isEdge(int i, int j);
  void display();
  void initializeState();
  //void setWeightMatrix(double** &weightMatrix);
  void showBasicInfo();
  void Dijkstra(int startNode, int endPoint);
  void returnDijkstraPath(int startNode, int endNode,
                          std::vector<int>& pathNode);
  double returnDijkstraPathDistance(int endNode);
  int isAllKeyTrue();  //0 means not MST, 1 means MST
  int findMinDistanceNode();

};




#endif /* INCLUDE_GRAPH_H_ */
