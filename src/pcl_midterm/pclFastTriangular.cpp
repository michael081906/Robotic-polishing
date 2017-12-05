// "Copyright [2017] <Michael Kam>"
/** @file pclFastTriangular.cpp
 *  @brief This is the implementation of the pclFastTriangular class. This class consists of 6 methods.
 *  Please refer the pclFastTriangular.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pclFastTriangular.h>
#include <ios>
#include <vector>
#include <iostream>
// using namespace pcl;

pclFastTriangular::pclFastTriangular() {
  searchRadius = 0;
}

void pclFastTriangular::setSearchRadius(double radius) {
  searchRadius = radius;
}
double pclFastTriangular::getSearchRadius() {
  return searchRadius;
}
void pclFastTriangular::setInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}
void pclFastTriangular::getInputCloud(
    pcl::PointCloud<pcl::PointNormal>& cloudOut) {
  cloudOut = cloud;
}
void pclFastTriangular::reconctruct(pcl::PolygonMesh& triangles) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointNormal>);
  cloudPtr = cloud.makeShared();
  tree->setInputCloud(cloudPtr);
  // Set the maximum distance between connected points (maximum edge length)
  /*
   * The method works by maintaining a list of points from which the mesh can be
   * grown (“fringe” points) and extending it until all possible points are connected.
   * It can deal with unorganized points, coming from one or multiple scans, and having
   * multiple connected parts. It works best if the surface is locally smooth and there
   * are smooth transitions between areas with different point densities.
   *
   * Triangulation is performed locally, by projecting the local neighborhood of a point
   * along the point’s normal, and connecting unconnected points. Thus, the following parameters
   * can be set:
   *
   * 1. setMaximumNearestNeighbors(unsigned) and setMu(double) control the size of the neighborhood.
   * The former defines how many neighbors are searched for, while the latter specifies the maximum
   * acceptable distance for a point to be considered, relative to the distance of the nearest point
   * (in order to adjust to changing densities). Typical values are 50-100 and 2.5-3 (or 1.5 for grids).
   *
   * 2. setSearchRadius(double) is practically the maximum edge length for every triangle. This has
   * to be set by the user such that to allow for the biggest triangles that should be possible.
   *
   * 3. setMinimumAngle(double) and setMaximumAngle(double) are the minimum and maximum angles
   * in each triangle. While the first is not guaranteed, the second is. Typical values are 10
   * and 120 degrees (in radians).
   *
   * 4.setMaximumSurfaceAgle(double) and setNormalConsistency(bool) are meant to deal with
   * the cases where there are sharp edges or corners and where two sides of a surface run very
   * close to each other. To achieve this, points are not connected to the current point if their
   * normals deviate more than the specified angle (note that most surface normal estimation methods
   * produce smooth transitions between normal angles even at sharp edges). This angle is computed
   * as the angle between the lines defined by the normals (disregarding the normal’s direction)
   * if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee
   * consistently oriented normals. Typically, 45 degrees (in radians) and false works on most datasets.

   */
  gp3.setSearchRadius(searchRadius);
  gp3.setMu(5);  // specifies the maximum acceptable distance for a point to be considered
  gp3.setMaximumNearestNeighbors(20);  // how many neighbors are searched for
  gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);  // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3);  // 120 degrees
  gp3.setNormalConsistency(false);
  // Get result
  gp3.setInputCloud(cloudPtr);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(triangles);
}

std::vector<int> pclFastTriangular::getSegID() {
  return gp3.getPartIDs();
}

