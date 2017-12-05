// "Copyright [2017] <Michael Kam>"
/** @file main.cpp
 *  @brief main.cpp is a main file of shell-app.
 **
 *  This main file demonstrate using point cloud library to detect obstacles.
 *  This demo follow the procedures in the activity diagram under the UML directory.
 *
 *  @author Michael Kam (michael081906)
 *  @bug No known bugs.
 *  @copyright GNU Public License.
 */
#include "s_hull_pro.h"
#include "pclIo.h"
#include "pclVoxel.h"
#include "pclMlsSmoothing.h"
#include "pclPassThrough.h"
#include "pclStatisticalOutlierRemoval.h"
#include "pclFastTriangular.h"
#include "pclCloudViewer.h"
#include "obstacleIdentify.h"
#include "mergeHanhsPoint.h"
#include "graph.h"
#include "delaunay3.h"
#include "prepareDijkstra.h"
#include "dijkstraPQ.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <iostream>
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <hash_set>
#include <ctype.h>
#include <string.h>
#include <string>
#include <sys/types.h>
#include <hash_set>
#include <set>
#include <vector>
#include <fstream>
#include <math.h>
#include <time.h>

using namespace std;

bool pointSortPredicate(const Shx& a, const Shx& b) {
  if (a.r < b.r)
    return true;
  else if (a.r > b.r)
    return false;
  else if (a.c < b.c)
    return true;
  else
    return false;
}
;

bool pointComparisonPredicate(const Shx& a, const Shx& b) {
  return a.r == b.r && a.c == b.c;
}

int main() {

  if (0) {

    cerr << "new delaunay triangulation method test" << endl;

    std::vector<Shx> pts;
    Shx pt;
    srand(1);

    for (int v = 0; v < 5; v++) {
      pt.id = v;
      pt.r = (float) rand();  // pts.txt
      pt.c = (float) rand();

      pts.push_back(pt);
    }

    //std::sort(pts.begin(), pts.end(), pointSortPredicate);
    //std::vector<Shx>::iterator newEnd = std::unique(pts.begin(), pts.end(),
    //                                                pointComparisonPredicate);
    //pts.resize(newEnd - pts.begin());

    write_Shx(pts, "pts.txt");

    std::vector<Triad> triads;

    s_hull_pro(pts, triads);

    write_Triads(triads, "triangles.txt");

    exit(0);
  }

  if (1) {
    /*
     int r = 30000, c = 30000, i, j, count;

     double **arr = (double **) malloc(r * sizeof(double *));
     for (i = 0; i < r; i++)
     arr[i] = (double *) malloc(c * sizeof(double));



     count = 0;
     for (i = 0; i < r; i++)
     for (j = 0; j < c; j++)
     arr[i][j] = ++count;  // OR *(*(arr+i)+j) = ++count
     /*
     for (i = 0; i < r; i++)
     for (j = 0; j < c; j++)
     printf("%f\n ", arr[i][j]);
     */
    // http://www.geeksforgeeks.org/dynamically-allocate-2d-array-c/
    /*
     double** weight;
     int vertexCount = 40000;
     weight = new double*[vertexCount];
     for (int i = 0; i < vertexCount; i++) {
     weight[i] = new double[vertexCount];
     for (int j = 0; j < vertexCount; j++)
     weight[i][j] = 0;
     }*/
    // 0. Initialization
    pclIo pclLoad;
    pclCloudViewer pclView;
    pclPassThrough pt;
    pclVoxel vx;
    pclStatistOutRev sor;
    obstacleIdentify oi;
    pclMlsSmoothing mls;
    pclFastTriangular ft;
    pcl::PolygonMesh triangles;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointXYZ>);
    //  1. Load pcd file
    std::cout << " 1. Loading pcd file, please wait..." << std::endl;
    //pclLoad.readPCDfile("../PCL_Test_Mat.pcd");
    pclLoad.readPCDfile("../kidney3dots3_p1.pcd");
    // pclLoad.readPCDfile("../PCL_Matlab_Result__ROI_Smoothing_ascii.pcd");
    pclLoad.getPointCloud(*cloud_out);
    std::cout << " Loading completed" << std::endl;
    //  2. Remove the noise of point cloud
    std::cout << " 2. Removing the noise of point cloud, please wait..."
              << std::endl;
    sor.setInputCloud(*cloud_out);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filterProcess(*cloud_out);
    //*******************************************************************
    // 2017.11.9 Added Michael
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_out);
    outrem.setRadiusSearch(3);
    outrem.setMinNeighborsInRadius(100);
    // apply filter
    outrem.filter(*cloud_out);
    //*******************************************************************
    std::cout << " Removing completed" << std::endl;
    /*****************************************************************/
    //  4. Down sample the point cloud
    std::cout << " 4. Down sampling the point cloud, please wait..."
              << std::endl;
    vx.setInputCloud(*cloud_out);
    vx.setLeafSize(0.2, 0.2, 0.2);
    vx.filterProcess(*cloud_out);
    std::cout << " Down sampling completed" << std::endl;
    //  3. Extract certain region of point cloud
    std::cout << " 3. Extracting certain region of point cloud, please wait..."
              << std::endl;
    pt.setInputCloud(*cloud_out);
    pt.setFilterXlimit(4.5, 23.0);
    pt.filterProcess(*cloud_out);
    pt.setInputCloud(*cloud_out);
    pt.setFilterYlimit(5.0, 16.5);
    pt.filterProcess(*cloud_out);
    pt.setInputCloud(*cloud_out);
    pt.setFilterZlimit(7, 11.0);
    pt.filterProcess(*cloud_out);
    std::cout << " Extracting completed" << std::endl;
    /*
     pcl::visualization::CloudViewer viewer("Cloud of Raw data");
     viewer.showCloud(cloud_out);
     int user_data = 0;
     do {
     user_data++;
     } while (!viewer.wasStopped());
     /*****************************************************************/
    //  5. Smooth the point cloud
    std::cout << " 5. Smoothing the point cloud, please wait..." << std::endl;
    mls.setInputCloud(*cloud_out);
    mls.setSearchRadius(4);
    mls.mlsProcess(*cloud_with_normal);
    std::cout << " Smoothing completed" << std::endl;
    //  7. Mesh the obstacle
    std::cout << " 7. Meshing the obstacle, please wait..." << std::endl;
    ft.setInputCloud(*cloud_with_normal);
    ft.setSearchRadius(4);
    ft.reconctruct(triangles);
    std::vector<int> gp33 = ft.getSegID();
    int sizegp3 = gp33.size();
    int sizePointCloud = cloud_with_normal->size();
    std::cout << " Meshing completed..." << std::endl;
    /* for (int i = 0; i < gp33.size(); i++)
     std::cout << gp33[i] << " ";
     */
    // std::cout << " size of PointNormal: " << sizePointCloud << std::endl;
    std::cout << " size of gp3: " << sizegp3 << std::endl;
    //  8. Show the result
    //pclView.display(triangles);
    // 9.mergeHanhsPoint
    std::cout << " 9. Merging point of selected point, please wait..."
              << std::endl;
    mergeHanhsPoint mg;
    std::vector<int> selectedPoint;
    mg.readtext("../kidney3dots3_pointCluster.txt");
    mg.setInputCloud(*cloud_with_normal);
    mg.findNearestProcess(selectedPoint);
    std::cout << " 9. Merging completed" << std::endl;
    std::vector<int> size3;
    size3 = ft.getSegID();
    std::cout << selectedPoint[0] << " " << selectedPoint[1] << " "
              << selectedPoint[2] << " " << size3.size() << std::endl;
    std::cout << " 10. Delaunay3 function..." << std::endl;
    std::vector<Triad> triads;
    std::vector<Shx> ptsOut;
    delaunay3 dy3;
    dy3.setInputCloud(*cloud_with_normal);
    dy3.putPointCloudIntoShx();
    dy3.processDelaunay(triads);
    dy3.getShx(ptsOut);
    write_Triads(triads, "triangles.txt");
    write_Shx(ptsOut, "pts.txt");
    std::cout << " 10. Delaunay3 function completed..." << std::endl;
    int start = selectedPoint[1];
    int end = selectedPoint[2];
    std::vector<int> path;
    dijkstraPQ dPQ(size3.size());
    dPQ.setInputCloud(*cloud_with_normal);
    dPQ.setTri(triads);
    dPQ.computeWeight();
    dPQ.shortestPath(start, end);
    dPQ.returnDijkstraPath(start, end, path);
    std::vector<int>::iterator route = path.begin();
    while (route != path.end()) {
      std::cout << *route << " ";
      ++route;
    }
    std::cout << std::endl;
    std::vector<position> POS;
    dPQ.returnDijkstraPathPosition(start, end, POS);
    std::vector<position>::iterator routePos = POS.begin();

    while (routePos != POS.end()) {
      std::cout << (*routePos).x << " ";  // p.80
      ++routePos;
    }
    std::cout << std::endl;
    routePos = POS.begin();
    while (routePos != POS.end()) {
      std::cout << (*routePos).y << " ";
      ++routePos;
    }
    std::cout << std::endl;
    routePos = POS.begin();
    while (routePos != POS.end()) {
      std::cout << (*routePos).z << " ";
      ++routePos;
    }

    if (0) {
      std::cout << " 11. Calculating weighting function..." << std::endl;
    double** wei;
    prepareDijkstra pD(size3);
    pD.setInputCloud(*cloud_with_normal);
    pD.setTri(triads);
    pD.computeWeight();
    pD.getWeight(wei);
    std::cout << " 11. Calculation completed" << std::endl;
    Graph Dpath(size3.size(), wei);
     Dpath.initializeState();
    Dpath.Dijkstra(start, end);
    Dpath.returnDijkstraPath(start, end, path);
    std::vector<int>::iterator route = path.begin();
      std::cout << "Distance = " << Dpath.returnDijkstraPathDistance(end);
      std::cout << std::endl;
     while (route != path.end()) {
      std::cout << *route << " ";
     ++route;
    }
    }
    return 0;
  }

}
