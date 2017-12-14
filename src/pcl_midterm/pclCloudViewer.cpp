// "Copyright [2017] <Michael Kam>"
/** @file pclCloudViewer.cpp
 *  @brief This is the implementation of the pclCloudViewer class. This class consists of 1 method.
 *  Please refer the pclCloudViewer.h for more detail.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  This class utilize pcl visualization to display point cloud.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No1 can't not be compile successfully in the test case.
 *  @copyright GNU Public License.
 *
 *  pclCloudViewer is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  pclCloudViewer is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with pclCloudViewer.  If not, see <http://www.gnu.org/licenses/>. *
 *
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

