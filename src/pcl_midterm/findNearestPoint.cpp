// "Copyright [2017] <Michael Kam>"
/** @file findNearestPoint.cpp
 *  @brief This findNearestPoint.cpp is an implementation of finding
 *   the closest point on a post smoothing surface. There are three method in this class.
 *   The goal is to get indices array which indicate the closest point with several given points.
 *   For example, I can given a arbitrary coordinate and use this class to
 *   find a closest point ID from point cloud group.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <findNearestPoint.h>
#include <stdlib.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <ctime>

findNearestPoint::findNearestPoint() {

}

std::vector<double> findNearestPoint::readtext(const std::string &fileName) {
  // TODO: remember how to load file in C++
  std::ifstream file(fileName);  // Create a input stream
  std::vector<std::string> readString;
  std::vector<double> readDouble;
  std::string word;
  while (file >> word) {
    readString.push_back(word);  //push back string, which defined in the vector
    // TODO: remember how to convert string into double
    this->specificPoints.push_back(atof(word.c_str()));
    readDouble.push_back(atof(word.c_str()));
  }
  return readDouble;
}

void findNearestPoint::setPosition(const std::vector<float>& pos)
{
  for (std::vector<float>::const_iterator it = pos.begin(); it < pos.end();
      ++it) {
    this->specificPoints.push_back(*it);
  }
}


void findNearestPoint::setInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}

void findNearestPoint::findNearestProcess(std::vector<int>& nearIndices) {
  // Set pointNormal as an input into kTree
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud(cloud.makeShared());
  // Create the point cloud you want to search
  pcl::PointNormal searchPoint;
  pcl::PointCloud<pcl::PointNormal> searchPoints;
  int i;
  // TODO: There must be a better way to do this
  /*  for (std::vector<double>::const_iterator iter = specificPoints.begin();
      iter != specificPoints.end(); iter + 3) {
    searchPoint.x = *iter;
    searchPoint.y = *(iter + 1);
    searchPoint.z = *(iter + 2);
    searchPoints.push_back(searchPoint);
   } */
  for (std::vector<double>::const_iterator iter = specificPoints.begin();
      iter < specificPoints.end(); ++++++iter)  // 2017.11.26 [Bug fixed] Michael. ++++++iter
      {
    searchPoint.x = *iter;
    searchPoint.y = *(iter + 1);
    searchPoint.z = *(iter + 2);
    searchPoints.push_back(searchPoint);
  }
  // Number of K point you want to search
  int K = 1;
  // Vector to store point index
  std::vector<int> pointIdxNKNSearch(K);
  // Vector to store distance between given point and Kth near point
  std::vector<float> pointNKNSquaredDistance(K);
  // const_iterator: if you only want to read
  // iterator: if you want to write
  for (pcl::PointCloud<pcl::PointNormal>::const_iterator iter = searchPoints
      .begin(); iter != searchPoints.end(); iter++) {
    kdtree.nearestKSearch(*iter, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    nearIndices.insert(nearIndices.end(), pointIdxNKNSearch.begin(),
                       pointIdxNKNSearch.end());
  }
}

