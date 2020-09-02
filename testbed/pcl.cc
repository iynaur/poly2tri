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


double angle(p2t::Point* p, p2t::Point* a, p2t::Point* b){
  Point from = *a - *p;
  Point to = *b - *p;
  double l = from.Length();
  double s = to.Length();
  if (l < s) swap(l, s);
  double S_mark = Cross(from, to);
  double S = fabs(S_mark);
  double d = (from - to).Length();
  double h = S / d;

  double dmin;
  if (l*l - s*s > d*d) dmin = s;
  else dmin = h;
  if (dmin < 0.01) return 10;

  return asin( S_mark / l / s);
}


bool inPoly(p2t::Point* p, const vector<p2t::Point*> &polyline){
  double tot = 0;
  int n = polyline.size();

  for (int i=0; i<n; ++i){
    Point* a = polyline[i];
    Point* b = polyline[(i+1)%n];
    double ang = angle(p, a, b);
    if (ang == 10)
      return false;
    tot += angle(p, a, b);
  }
  if (fabs(tot) < M_PI) {
    assert(fabs(tot) < 0.1);
    return false;
  }
  assert(fabs(tot) > 2*M_PI - 0.1);
  return true;
}

vector<p2t::Point*> generateRandomPoints(const vector<p2t::Point*> &polyline, double gridSize){
  double xmin = 10000, xmax = -xmin, ymin = 10000, ymax = -ymin;
  for (p2t::Point* p : polyline){
    xmin = min(xmin, p->x);
    xmax = max(xmax, p->x);
    ymin = min(ymin, p->y);
    ymax = max(ymax, p->y);
  }

  vector<p2t::Point*> add;
  for (double x = xmin; x<xmax; x+=gridSize)
    for (double y = ymin; y<ymax; y+= gridSize){
      Point *p = new Point(x, y);
      if (inPoly(p, polyline))
        add.push_back(p);
    }
  return add;
}


int main(int argc, char* argv[]){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
                                           pcl::PointCloud<pcl::PointXYZ>);
  Json::Value rt;
  jsonfileopt::file2json("./position.json", rt);

  std::unordered_map<string, vector<string>> mp;
  std::unordered_map<string, pair<int, int>> mid;
  Json::Value p_list = rt["position"];
  int n = p_list.size();
  vector<string> labs({"x", "y"});

  std::unordered_map<string, string> to_real;
  vector<float> data;
  for (int i = 0; i<n; ++i){
//    Json::Value seg_list = p_list[i]["position" + std::to_string(i)];

    Json::Value root = p_list[i];
    if( root.size() == 0 ) continue;
    Json::Value seg_list = *root.begin();
    vector<string> pv;
    for (int j=0; j<2; ++j){

      for (string lab : labs){
        string ns = seg_list[j][lab].asString();

        data.push_back(atof(ns.c_str()));
      }
      cloud->push_back(pcl::PointXYZ(data[data.size() - 2], data.back(), 0));

    }

  }
  assert(cloud->size() == 2*n);
  assert(data.size() == 4*n);
  int *ids;
//  vector<int> dbg;
  tri(data.data(), n, &ids/*, dbg*/);

  vector<pcl::Vertices> polygons;


//  ids = dbg.data();
  int c = ids[0];
  int base = 1 + c;
  for (int i = 0; i<c; ++i){
    if (i>0) base += ids[i]*6;
    int *tri = ids + base;
    for (int j = 0; j< ids[i+1]; ++j){
      pcl::Vertices vet;
      for(int ang = 0; ang <3; ++ang)
      {
        assert(tri[6*j + 2*ang] < n);
        assert(tri[6*j + 2*ang + 1] < 2);
        int id = tri[6*j + 2*ang]*2 + tri[6*j + 2*ang + 1];
        assert(id < 2*n);

        vet.vertices.push_back(id);
      }
      polygons.push_back(vet);
    }
  }

//  pcl::io::loadPCDFile("region.pcd", *cloud);





  pcl::visualization::PCLVisualizer viewer;
  viewer.addPolygonMesh<pcl::PointXYZ>(cloud, polygons);
  while (!viewer.wasStopped()){
    viewer.spinOnce();
    std::this_thread::sleep_for(chrono::microseconds(100));

  }
  viewer.resetStoppedFlag();
  viewer.removeAllShapes();
  viewer.removeAllPointClouds();
  return 0;


}
