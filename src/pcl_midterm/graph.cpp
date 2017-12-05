/*
 * graph.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: viki
 */
#include <graph.h>
#include <iostream>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
using namespace std;
#define INT_MAX 2147483647;

Graph::Graph(int vertexCount) {
  this->vertexCount = vertexCount;

  this->key = new int[vertexCount];
  this->distance = new double[vertexCount];
  this->parent = new int[vertexCount];
// Generate the 2D array for storing the weight(distance between the vertex)
  adjMatrix = new double*[vertexCount];
  for (int i = 0; i < vertexCount; i++) {
    adjMatrix[i] = new double[vertexCount];
    for (int j = 0; j < vertexCount; j++)
      adjMatrix[i][j] = 0;
  }
}
Graph::Graph(int vertexCount, double** &weightMatrix) {
  this->vertexCount = vertexCount;
  this->key = new int[vertexCount];
  this->distance = new double[vertexCount];
  this->parent = new int[vertexCount];
  this->adjMatrix = weightMatrix;
}

Graph::~Graph() {
  for (int i = 0; i < vertexCount; i++)
    delete[] adjMatrix[i];
  delete[] adjMatrix;
}

void Graph::addEdgeDirectedWeight(int i, int j, double cost) {
  if (i >= 0 && i < vertexCount && j >= 0 && j < vertexCount) {
    adjMatrix[i][j] = cost;
    // TODO: What is "Directed" means here?
    adjMatrix[j][i] = cost;  // added by Michael 2017/11/3
  }
}
void Graph::removeEdgeUndirected(int i, int j) {
  if (i >= 0 && i < vertexCount && j >= 0 && j < vertexCount) {
    adjMatrix[i][j] = 0;
    adjMatrix[j][i] = 0;
  }
}
double Graph::isEdge(int i, int j) {
  if (i >= 0 && i < vertexCount && j >= 0 && j < vertexCount)
    return adjMatrix[i][j];
  else {
    cout << "Invalid vertex number.\n";
    return 0;
  }
}
void Graph::display() {
  int u, v;  //vertex
  cout << "\t   " << " ";
  for (u = 0; u < vertexCount; u++) {
    cout << u << " ";
  }
  for (u = 0; u < vertexCount; u++) {
    cout << "\nNode[" << (char) (u + 48) << "] -> ";
    for (v = 0; v < vertexCount; ++v) {
      cout << " " << adjMatrix[u][v];
    }
  }
  cout << "\n\n";
}
void Graph::showBasicInfo() {
  for (int i = 0; i < vertexCount; i++) {
    cout << "node: " << i << " Key: " << key[i] << " distance: " << distance[i]
         << " parent: " << parent[i] << "\n";
  }
}
int Graph::isAllKeyTrue() {
  for (int i = 0; i < this->vertexCount; i++) {
    if (this->key[i] == 0)
      return 0;  // not MST yet
  }
  return 1;  // MST done

}
int Graph::findMinDistanceNode() {
  double minDistant = INT_MAX
  ;
  int minDistantNode;

  for (int i = 0; i < vertexCount; i++) {
    if (minDistant > this->distance[i] && this->key[i] == 0) {  //0 means that node is not in MST
      minDistantNode = i;
      minDistant = this->distance[i];
      //cout<<"min: "<<minDistantNode<<"\n";
    }
  }
  //cout<<"Min Distant Node: "<<minDistantNode<<" Cost: "<<minDistant<<"\n";
  return minDistantNode;  // the next operated point index
}

void Graph::initializeState() {
  for (int i = 0; i < this->vertexCount; i++) {
    this->key[i] = 0;  // 0=not in MST, 1=yes in MST
    this->distance[i] = INT_MAX
    ;  //initially distance is Max int
    this->parent[i] = -1;  // -1=no parent, else parent info
                           //
  }
}
void Graph::Dijkstra(int startNode, int endPoint) {
  cout << "\nDijkstra Shortest Path starts . . . \n";
  // initialization is done before call this method
  /* is the weighting that accumulated. By setting this into 0 indicate is the first to calculate.
   * The initial value for the distance are all INT_MAX */
  this->distance[startNode] = 0;  //start node's distance is 0
  int minDistanceNode, i;

  // 0 means Shortest path calculation is not done yet.
  while (!this->isAllKeyTrue() || (this->key[endPoint] != 1)) {  // [Bug fixed] 2017.11.9 Michael
    //cout<<"-------------------------------\n";
    minDistanceNode = findMinDistanceNode();  // return the index of among the points
    this->key[minDistanceNode] = 1;  // this node's shortes path is done

    /* cout << "Shortest Path: " << this->parent[minDistanceNode] << "->"
         << minDistanceNode << ", Destination Node's cost is: "
     << distance[minDistanceNode] << "\n";
     */

    for (i = 0; i < vertexCount; i++) {
      if (this->isEdge(minDistanceNode, i) && this->key[i] == 0) {
        //Below is the code for relaxation
        if (this->distance[i]
            > this->distance[minDistanceNode] + adjMatrix[minDistanceNode][i]) {
          this->distance[i] = this->distance[minDistanceNode]
              + adjMatrix[minDistanceNode][i];
          this->parent[i] = minDistanceNode;  //
        }
      }
    }
    //this->showBasicInfo(); // To visualize more clearly
    // you can comment this to only show the edges of MST
  }

}

void Graph::returnDijkstraPath(int startPoint, int endPoint,
                               std::vector<int>& pathNode) {
// Make sure you have finish the Dijkstra before you run this method to get the ID
  int tempPoint;
  pathNode.push_back(endPoint);
  tempPoint = endPoint;
  while (tempPoint != startPoint) {
    tempPoint = this->parent[tempPoint];
    pathNode.push_back(tempPoint);
  }
  std::reverse(pathNode.begin(), pathNode.end());
}

double Graph::returnDijkstraPathDistance(int endNode) {
  return this->distance[endNode];
}
/*
void Graph::setWeightMatrix(double** &weightMatrix) {
  this->adjMatrix = weightMatrix;
}
 */
