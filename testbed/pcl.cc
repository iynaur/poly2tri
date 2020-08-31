// Coding: UTF-8(BOM)
#pragma execution_character_set("utf-8")

#include <iostream>
#include "jsonfileopt.h"
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/format.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <bits/stdc++.h>

//#define PCL_NO_PRECOMPILE
#include <pcl/surface/gp3.h>

#include "../poly2tri/poly2tri.h"
using namespace p2t;


using namespace std;


vector<pcl::Vertices> getedge(vector<pcl::Vertices> polygons){
  map<pair<int, int>, int> mp;
  for (pcl::Vertices &v : polygons){
    assert(v.vertices.size() == 3);
    for (int i=0; i<3; ++i){
      for (int j=i+1; j<3; ++j){
        int u = v.vertices[i];
        int w = v.vertices[j];
        if (u<w) swap(u, w);
        mp[make_pair(u, w)]++;
      }
    }
  }

  vector<pcl::Vertices> edges;
  for (auto &p : mp){
    assert(p.second < 3);
    if (p.second == 1){
      pcl::Vertices v;
      v.vertices = vector<uint32_t>({p.first.first, p.first.second, p.first.second});
      edges.push_back(v);
    }
  }

  return edges;


}

bool isClosure(vector<pcl::Vertices> polygons){
  map<pair<int, int>, int> mp;
  for (pcl::Vertices &v : polygons){
    assert(v.vertices.size() == 3);
    for (int i=0; i<3; ++i){
      for (int j=i+1; j<3; ++j){
        int u = v.vertices[i];
        int w = v.vertices[j];
        if (u<w) swap(u, w);
        mp[make_pair(u, w)]++;
      }
    }
  }

  int dummy;
  for (auto &p : mp){
    if(p.second != 2){
      dummy = p.second;
    }

  }

  return true;


}

int main(int argc, char* argv[]){
  pcl::ConcaveHull<pcl::PointXYZ> hull;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
                                           pcl::PointCloud<pcl::PointXYZ>);
  Json::Value rt;
  jsonfileopt::file2json("./position.json", rt);

  std::unordered_map<string, vector<string>> mp;
  Json::Value p_list = rt["position"];
  int n = p_list.size();
  vector<string> labs({"x", "y", "z"});

  for (int i = 0; i<n; ++i){
    Json::Value seg_list = p_list[i]["position" + std::to_string(i)];
    vector<string> pv;
    for (int j=0; j<2; ++j){
      string ps;
      for (string lab : labs){
        string ns = seg_list[j][lab].asString();
        int trac = 9;
        if (ns.size() > trac) ns.resize(trac);
        ps += ns + " ";
      }
      pv.push_back(ps);
    }
    mp[pv[0]].push_back(pv[1]);
    mp[pv[1]].push_back(pv[0]);
  }
  assert(mp.size() == n);

  vector<string> poly;
  auto it = mp.begin();
  poly.push_back(it->first);
  poly.push_back(it->second[0]);
  while(poly.back() != poly.front()){
    vector<string> nb = mp[poly.back()];
    for (string nxt : nb){
      if (nxt != poly[poly.size() - 2]){
        poly.push_back(nxt);
        break;
      }
    }
  }
  poly.pop_back();

  vector<p2t::Point*> polyline;

  for (string ns : poly) {
    stringstream ss;
    ss<<ns;
    double x, y; ss>>x>>y;

    // Create a simple bounding box
    polyline.push_back(new Point(x,y));
  }

  CDT* cdt = new CDT(polyline);
  cdt->Triangulate();


  auto triangles = cdt->GetTriangles();
  auto mAp = cdt->GetMap();

//  pcl::io::loadPCDFile("region.pcd", *cloud);
  if (0)
  {
    float mi = 0; float ma = 2*M_PI - 0.5;
    for (float dist : {0.7f, 1.0f} ){
      for (float a = mi; a < ma; a += 0.1){
        cloud->push_back(pcl::PointXYZ(dist*sin(a), dist*cos(a), 0));
      }
    }
    for (float dist = 0.7; dist <= 1.0; dist+=0.1)
    {

      cloud->push_back(pcl::PointXYZ(dist*sin(mi), dist*cos(mi), 0));
      cloud->push_back(pcl::PointXYZ(dist*sin(ma), dist*cos(ma), 0));
    }

  }
  hull.setInputCloud(cloud);
  hull.setDimension(3);
  string ds = "0.5";
  if (argc > 1) ds = argv[1];
  stringstream ss;
  ss<<ds;

  double alpha;
  ss>>alpha;
  hull.setAlpha(alpha);
  vector<pcl::Vertices> polygons;
  pcl::PointCloud<pcl::PointXYZ>::Ptr surface(new
                                           pcl::PointCloud<pcl::PointXYZ>);
  hull.reconstruct(*surface, polygons);
  isClosure(polygons);

  pcl::visualization::PCLVisualizer viewer;
  viewer.addPolygonMesh<pcl::PointXYZ>(surface, polygons);
  while (!viewer.wasStopped()){
    viewer.spinOnce();
    std::this_thread::sleep_for(chrono::microseconds(100));

  }
  viewer.resetStoppedFlag();
  viewer.removeAllShapes();
  viewer.removeAllPointClouds();
  return 0;


  if (0){
    pcl::ConvexHull<pcl::PointXYZ> hull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
                                             pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("region.pcd", *cloud);
    hull.setInputCloud(cloud);
    hull.setDimension(3);


    hull.reconstruct(*surface, polygons);
    isClosure(polygons);

    viewer.addPolygonMesh<pcl::PointXYZ>(surface, polygons);
    while (!viewer.wasStopped()){
      viewer.spinOnce();
      std::this_thread::sleep_for(chrono::microseconds(100));

    }
    viewer.resetStoppedFlag();
    viewer.removeAllShapes();
    viewer.removeAllPointClouds();
  }

  {
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> hull;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new
                                             pcl::PointCloud<pcl::PointNormal>);
//    pcl::io::loadPCDFile("region.pcd", *cloud);
    if (1)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new
                                               pcl::PointCloud<pcl::PointXYZ>);
      for (float dist : {0.7f, 1.0f} ){
        for (float a = 0; a < M_PI; a += 0.1){
          cloud3->push_back(pcl::PointXYZ(dist*sin(a), dist*cos(a), 0));
        }
      }
      for (float d = 0.7; d <= 1.0; d+=0.1)
      {
        cloud3->push_back(pcl::PointXYZ(0, d, 0));
        cloud3->push_back(pcl::PointXYZ(0, -d, 0));
      }
      pcl::copyPointCloud(*cloud3 , *cloud);

    }
    {
      pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal>::Ptr
          ne(new pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal>);
      ne->setInputCloud(cloud);
      ne->setKSearch(40);
      ne->compute(*cloud);
    }
    hull.setInputCloud(cloud);
    hull.setMu(3);
    hull.setMaximumNearestNeighbors(1000);
    hull.setMaximumAngle(M_PI);
    hull.setMinimumAngle(0.001);
    hull.setSearchRadius(alpha);

    hull.reconstruct(polygons);

    auto edges =  getedge(polygons);

    viewer.addPolygonMesh<pcl::PointNormal>(cloud, polygons);
    while (!viewer.wasStopped()){
      using namespace std::chrono;
      viewer.spinOnce();
      std::this_thread::sleep_for(chrono::microseconds(100));
    }
    viewer.resetStoppedFlag();
    viewer.removeAllShapes();
    viewer.removeAllPointClouds();
  }

}
