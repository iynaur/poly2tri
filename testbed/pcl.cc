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
  pcl::ConcaveHull<pcl::PointXYZ> hull;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
                                           pcl::PointCloud<pcl::PointXYZ>);
  Json::Value rt;
  jsonfileopt::file2json("./position.json", rt);

  std::unordered_map<string, vector<string>> mp;
  std::unordered_map<string, pair<int, int>> mid;
  Json::Value p_list = rt["position"];
  int n = p_list.size();
  vector<string> labs({"x", "y", "z"});

  std::unordered_map<string, string> to_real;
  int trac = 6;
  for (int i = 0; i<n; ++i){
    Json::Value seg_list = p_list[i]["position" + std::to_string(i)];
    vector<string> pv;
    for (int j=0; j<2; ++j){
      string ps;
      string os;
      for (string lab : labs){
        string ns = seg_list[j][lab].asString();

        string ori = ns;
        if (ns.size() > trac)
        {
          int nsize = trac;
          if (ns[0] == '-') nsize++;
          ns.resize(nsize);
        }
        ps += ns + " ";
        os += ori + " ";
      }
      pv.push_back(ps);
      to_real[ps] = os;
      mid[ps] = {i, j};
    }
    mp[pv[0]].push_back(pv[1]);
    mp[pv[1]].push_back(pv[0]);
  }
  cout<<trac<<" "<<mp.size()<<" "<<n<<endl;
  assert(mp.size() == n);

  vector<vector<string>> polys;

  while(mp.size())

  {
    vector<string> poly; // point , id
    auto it = mp.begin();
    poly.push_back(it->first);
    poly.push_back(it->second[0]);
    mp.erase(it);
    while(poly.back() != poly.front()){
      string torm = poly.back();
      vector<string> nb = mp[poly.back()];
      for (string nxt : nb){
        if (nxt != poly[poly.size() - 2]){
          poly.push_back(nxt);
          mp.erase(torm);
          break;
        }
      }
    }
    poly.pop_back();
    polys.push_back(poly);
  }

  vector<pcl::Vertices> polygons;

  bool addPoint = 0;
  for (int i = 0; i< polys.size(); ++i) {
    auto poly = polys[i];
    vector<p2t::Point*> polyline;

    for (string ns : poly) {
      stringstream ss;
      ss<<to_real[ns];
      double x, y, z; ss>>x>>y>>z;

      // Create a simple bounding box
      polyline.push_back(new Point(x,y));
//      cloud->push_back(pcl::PointXYZ(x,y,z));
    }

    CDT* cdt = new CDT(polyline);
    vector<p2t::Point*> add;
    if (addPoint) {
      add = generateRandomPoints(polyline, 0.04);
      for(auto p : add) cdt->AddPoint(p);
    }
    cout<<"out size "<<polyline.size()<<" in size "<<add.size()<<endl;
    cout<<"expect tri "<<polyline.size() - 2 + 2*add.size()<<endl;
    cdt->Triangulate();


//    auto triangles = cdt->GetTriangles();
    vector<p2t::Point*> totp;
    std::vector<std::vector<int> > triids;
    if (addPoint) triids = cdt->GetTrianglesIndex(totp);
    else {
      triids = cdt->GetTrianglesIndexOfUnsortInput();
      totp = polyline;
    }
    auto mAp = cdt->GetMap();

    if(!addPoint){
      for (vector<int> tri : triids){
        for(int id : tri){
          cout<<mid[poly[id]].first<<" "<<mid[poly[id]].second<<" ";
        }
        cout<<"\n";
      }
      cout<<"========\n";
    }
    cout<<"point size "<<totp.size()<<" tri size "<<triids.size()<<endl;

    int base = cloud->size();
    for (auto p : totp){
      cloud->push_back(pcl::PointXYZ(p->x,p->y, 0));
    }
    for (vector<int> tri : triids){
      pcl::Vertices vet;
      for(auto id : tri) vet.vertices.push_back(id + base);
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
