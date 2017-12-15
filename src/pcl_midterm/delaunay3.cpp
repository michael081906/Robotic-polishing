// "Copyright [2017] <Michael Kam>"
/** @file delaunay3.h
 *  @brief This delaunay3.cpp is an implement file of the meshing triangle
 *  among the point cloud
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 *
 *  delaunay3 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  delaunay3 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with delaunay3.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <delaunay3.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <string>
#include <algorithm>

delaunay3::delaunay3() {
}

void delaunay3::setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}

void delaunay3::putPointCloudIntoShx() {
  int siz = cloud.size();
  for (int v = 0; v < siz; v++) {
    pt.id = v;
    pt.r = static_cast<float>(cloud.points[v].y);
    pt.c = static_cast<float>(cloud.points[v].z);
    pts.push_back(pt);
  }
}

void delaunay3::getShx(std::vector<Shx>& ptsOut) {
  ptsOut = pts;
}

void delaunay3::processDelaunay(std::vector<Triad>& triads) {
  s_hull_pro(pts, triads);
}

