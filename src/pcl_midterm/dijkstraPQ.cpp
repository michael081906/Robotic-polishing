// "Copyright [2017] <Michael Kam>"
/** @file dijkstraPQ.cpp
 *  @brief This dijkstraPQ.cpp is an implementation of finding shortest
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
#include "dijkstraPQ.h"
#include <bits/stdc++.h>
# define INF 0x3f3f3f3f

dijkstraPQ::dijkstraPQ(int size3) {
  this->V = size3;
  adj = new std::list<iPair>[size3];
  this->parent = new int[size3];
}

void dijkstraPQ::addEdge(int u, int v, double w) {
  adj[u].push_back(std::make_pair(v, w));
  adj[v].push_back(std::make_pair(u, w));
}

void dijkstraPQ::shortestPath(int startNode, int endPoint) {

  std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair> > pq;

  // Create a vector for distances and initialize all
  // distances as infinite (INF)
  std::vector<double> dist(V, INF);

  // Insert source itself in priority queue and initialize
  // its distance as 0.
  pq.push(std::make_pair(0, startNode));
  dist[startNode] = 0;

  /* Looping till priority queue becomes empty (or all
   distances are not finalized) */
  while (!pq.empty()) {
    // The first vertex in pair is the minimum distance
    // vertex, extract it from priority queue.
    // vertex label is stored in second of pair (it
    // has to be done this way to keep the vertices
    // sorted distance (distance must be first item
    // in pair)
    int u = pq.top().second;
    pq.pop();

    // 'i' is used to get all adjacent vertices of a vertex
    std::list<std::pair<int, double> >::iterator i;
    for (i = adj[u].begin(); i != adj[u].end(); ++i) {
      // Get vertex label and weight of current adjacent
      // of u.
      int v = (*i).first;
      double weight = (*i).second;

      //  If there is shorted path to v through u.
      if (dist[v] > dist[u] + weight) {
        // Updating distance of v
        dist[v] = dist[u] + weight;
        pq.push(std::make_pair(dist[v], v));
        this->parent[v] = u;
      }
    }
  }
  // Print shortest distances stored in dist[]
  // std::cout << "Vertex Distance from Source to EndPoint: " << dist[endPoint] << std::endl;
}

double dijkstraPQ::cal2Point(int point1, int point2) {
  double xD, yD, zD;
  xD = cloudPtr->points[point2].x - cloudPtr->points[point1].x;
  yD = cloudPtr->points[point2].y - cloudPtr->points[point1].y;
  zD = cloudPtr->points[point2].z - cloudPtr->points[point1].z;
  return sqrt(xD * xD + yD * yD + zD * zD);
}

void dijkstraPQ::computeWeight() {
  std::vector<int> storePointID;
  int siz = triads.size();
  for (int i = 0; i < siz; i++) {
    storePointID.push_back(triads[i].a);
    storePointID.push_back(triads[i].b);
    storePointID.push_back(triads[i].c);
    this->distanceCalculation(storePointID);
    storePointID.clear();
  }
}

void dijkstraPQ::distanceCalculation(std::vector<int>& same3indices) {
  // TODO(Michael): Write a general method for it
  // TODO(Michael): maybe we need to add some error index in order to debug in the future.

  /* Get the values from the 'same3indices'. Each values correspond to a point cloud index.
   * Use the cal2Point() to compute a distance between two point cloud. Finally, pass the result
   * into 'weight' matrix. If the size of the input equal to 3, then do the calculation(which
   * means that it is a triangle; otherwise, ignores the ID )
   */
  if (same3indices.size() == 3) {
    double dis01 = this->cal2Point(same3indices[0], same3indices[1]);
    double dis12 = this->cal2Point(same3indices[1], same3indices[2]);
    double dis02 = this->cal2Point(same3indices[0], same3indices[2]);
    this->addEdge(same3indices[0], same3indices[1], dis01);
    this->addEdge(same3indices[1], same3indices[2], dis12);
    this->addEdge(same3indices[0], same3indices[2], dis02);
  }
}

void dijkstraPQ::setTri(std::vector<Triad>& triadsIn) {
  triads = triadsIn;
}

void dijkstraPQ::returnDijkstraPath(int startPoint, int endPoint,
                                    std::vector<int>& pathNode) {  // Make sure you have finish the Dijkstra before you run this method to get the ID
  int tempPoint;
  pathNode.push_back(endPoint);
  tempPoint = endPoint;
  while (tempPoint != startPoint) {
    tempPoint = this->parent[tempPoint];
    pathNode.push_back(tempPoint);
  }
  std::reverse(pathNode.begin(), pathNode.end());
}

void dijkstraPQ::setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
  cloudPtr = cloud.makeShared();
}
void dijkstraPQ::returnDijkstraPathPosition(int startNode, int endNode,
                                            std::vector<position>& pathPos) {
  int tempPoint;
  position temPos;
  tempPoint = endNode;
  while (tempPoint != startNode) {
    temPos.x = this->cloudPtr->points[tempPoint].x;
    temPos.y = this->cloudPtr->points[tempPoint].y;
    temPos.z = this->cloudPtr->points[tempPoint].z;
    pathPos.push_back(temPos);
    tempPoint = this->parent[tempPoint];
  }
  std::reverse(pathPos.begin(), pathPos.end());

}



