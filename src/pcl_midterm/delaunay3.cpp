/*
 * delaunay3.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: viki
 */
#include <iostream>
#include <set>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>
#include <stdio.h>

#include "delaunay3.h"

delaunay3::delaunay3() {
}

void delaunay3::setInputCloud(pcl::PointCloud<pcl::PointNormal>& cloudIn) {
  cloud = cloudIn;
}

void delaunay3::putPointCloudIntoShx()
{
  int siz = cloud.size();
  for (int v = 0; v < siz; v++) {
    pt.id = v;
    pt.r = (float) cloud.points[v].y;
    pt.c = (float) cloud.points[v].z;
    pts.push_back(pt);
  }
}

void delaunay3::getShx(std::vector<Shx>& ptsOut) {
  ptsOut = pts;
}

void delaunay3::processDelaunay(std::vector<Triad>& triads) {
  s_hull_pro(pts, triads);
}






