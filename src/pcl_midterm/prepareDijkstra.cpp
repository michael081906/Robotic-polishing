/*
 * prepareDijkstra.cpp
 *
 *  Created on: Oct 30, 2017
 *      Author: viki
 */
#include <prepareDijkstra.h>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>


prepareDijkstra::prepareDijkstra(std::vector<int>& gp3PartID) {

  this->vertexCount = gp3PartID.size();
  // TODO: Maybe I can use shared pointer here?
  this->triPartID = gp3PartID;

  /*  std::vector<std::vector<double>> weight(vertexCount,
   std::vector<double>(vertexCount, 0));*/

  weight = (double **) malloc(this->vertexCount * sizeof(double *));
  for (int i = 0; i < this->vertexCount; i++) {
    weight[i] = (double *) malloc(this->vertexCount * sizeof(double));
    for (int j = 0; j < this->vertexCount; j++)
      weight[i][j] = 0;
  }
  // std::cout << gp3PartID.size() << std::endl;
  // std::cout << this->vertexCount << std::endl;
}

void prepareDijkstra::weightCalculation() {

  std::vector<int> sameTriangle;
  // int triNumber = (this->triPartID.size()) / 3;  // assume gp3PartID's size is three time

  /* 2017.11.6 Michael
   * Objective:find the maximum number in the vector<int>
   * https://stackoverflow.com/questions/9874802/how-can-i-get-the-max-or-min-value-in-a-vector
   */
  int triNumberMax = *std::max_element(triPartID.begin(), triPartID.end());
  std::cout << "Max number of tri: " << triNumberMax << std::endl;
  // [Code]int nPosition = 0;
  // [Comment]find the number from triPartID, from 0 ~ triNumber
  for (int i = 0; i < triNumberMax; i++) {
    // [Comment]1. Get the same triangle ID
    sameTriangle = this->findSameID(i);
    // [Comment]2. Calculate distance, filling in into the weight.
    this->distanceCalculation(sameTriangle);
    std::cout << i << std::endl;
    //this->distanceCalculation(this->findSameID(i));
  }
}

void prepareDijkstra::getWeight(double** &weightOut) {
  weightOut = weight;
}

void prepareDijkstra::setInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}

void prepareDijkstra::distanceCalculation(std::vector<int>& same3indices) {
  // TODO: Write a general method for it
  // TODO: maybe we need to add some error index in order to debug in the future.

  /* Get the values from the 'same3indices'. Each values correspond to a point cloud index.
   * Use the cal2Point() to compute a distance between two point cloud. Finally, pass the result
   * into 'weight' matrix. If the size of the input equal to 3, then do the calculation(which
   * means that it is a triangle; otherwise, ignores the ID )
   */
  if (same3indices.size() == 3) {
    double dis01 = this->cal2Point(same3indices[0], same3indices[1]);
    double dis12 = this->cal2Point(same3indices[1], same3indices[2]);
    double dis02 = this->cal2Point(same3indices[0], same3indices[2]);

    weight[same3indices[0]][same3indices[1]] = dis01;
    weight[same3indices[1]][same3indices[0]] = dis01;
    weight[same3indices[1]][same3indices[2]] = dis12;
    weight[same3indices[2]][same3indices[1]] = dis12;
    weight[same3indices[0]][same3indices[2]] = dis02;
    weight[same3indices[2]][same3indices[0]] = dis02;
  }
}

double prepareDijkstra::cal2Point(int point1, int point2) {
  cloudPtr = cloud.makeShared();
  double xD, yD, zD;
  xD = cloudPtr->points[point2].x - cloudPtr->points[point1].x;
  yD = cloudPtr->points[point2].y - cloudPtr->points[point1].y;
  zD = cloudPtr->points[point2].z - cloudPtr->points[point1].z;
  return sqrt(xD * xD + yD * yD + zD * zD);
}

std::vector<int> prepareDijkstra::findSameID(int indexOfgp3) {

  std::vector<int> v;
  std::vector<int>::iterator i = this->triPartID.begin();
  /*  while (i != v.end()) {
   std::cout << *i << endl;
   ++i;
   }*/
  i = this->triPartID.begin();
  int nPosition = 0;
  while (i != this->triPartID.end()) {
    i = find(i, this->triPartID.end(), indexOfgp3);
    if (i != this->triPartID.end()) {
      nPosition = distance(this->triPartID.begin(), i);
      v.push_back(nPosition);
      // cout << "Value " << *i;
      // cout << " found in the vector at position: " << nPosition << endl;
      ++i;
    }
  }
  return v;
}

void prepareDijkstra::setTri(std::vector<Triad>& triadsIn) {
  triads = triadsIn;
}
void prepareDijkstra::computeWeight() {
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





/* Is a initial version of findSameID  method.
 *
 vector<int> v;

 v.push_back(50);
 v.push_back(2991);
 v.push_back(23);
 v.push_back(9999);
 v.push_back(2991);
 v.push_back(2991);

 vector<int>::iterator i = v.begin();
 vector<int> pos;
 while (i != v.end()) {
 cout << *i << endl;
 ++i;
 }
 i = v.begin();
 int nPosition = 0;
 while (i != v.end()) {
 i = find(i, v.end(), 2991);
 if (i != v.end()) {
 nPosition = distance(v.begin(), i);
 cout << "Value " << *i;
 cout << " found in the vector at position: " << nPosition << endl;
 ++i;
 }
 }

 */

