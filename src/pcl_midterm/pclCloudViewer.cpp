// "Copyright [2017] <Michael Kam>"
/** @file pclCloudViewer.cpp
 *  @brief This is the implementation of the pclCloudViewer class. This class consists of 1 method.
 *  Please refer the pclCloudViewer.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pclCloudViewer.h"
#include <iostream>

pclCloudViewer::pclCloudViewer() {
  user_data = 0;
}

void pclCloudViewer::display(
    const pcl::PolygonMesh &triangles) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> gp3viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  gp3viewer->setBackgroundColor(0, 0, 0);
  gp3viewer->addPolygonMesh(triangles, "meshes", 0);
  gp3viewer->addCoordinateSystem(1.0);
  while (!gp3viewer->wasStopped()) {
    gp3viewer->spinOnce(100);
  }
}

