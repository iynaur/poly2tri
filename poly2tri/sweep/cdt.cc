/* 
 * Poly2Tri Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "cdt.h"
#include <unordered_map>
#include <iostream>
#include <cstring>
#include <limits>

using namespace std;

namespace p2t {

CDT::CDT(std::vector<Point*> polyline)
{
  sweep_context_ = new SweepContext(polyline);
  sweep_ = new Sweep;
  unsort_polyline = polyline;
}

void CDT::AddHole(std::vector<Point*> polyline)
{
  sweep_context_->AddHole(polyline);
}

void CDT::AddPoint(Point* point) {
  sweep_context_->AddPoint(point);
}

void CDT::Triangulate()
{
  sweep_->Triangulate(*sweep_context_);
}

std::vector<p2t::Triangle*> CDT::GetTriangles()
{
  return sweep_context_->GetTriangles();
}

std::vector<std::vector<int> > CDT::GetTrianglesIndex(std::vector<Point*> &totp)
{
  totp = sweep_context_->GetPoints();
  std::vector<p2t::Triangle*> tri = GetTriangles();
  int n = totp.size();
  std::unordered_map<Point*, int> mp;
  for (int i=0; i<n; ++i){
    mp[totp[i]] = i;
  }
  std::vector<std::vector<int> > triid;
  for (p2t::Triangle* pt : tri){
    std::vector<int> ptid(3);
    for (int i=0; i<3; ++i){
      Point* pp = pt->GetPoint(i);
      ptid[i] = mp[pp];
    }
    triid.push_back(ptid);
  }
  return triid;
}

std::vector<std::vector<int> > CDT::GetTrianglesIndexOfUnsortInput()
{
  std::vector<p2t::Triangle*> tri = GetTriangles();
    int n = unsort_polyline.size();
    std::unordered_map<Point*, int> mp;
    for (int i=0; i<n; ++i){
      mp[unsort_polyline[i]] = i;
    }
    std::vector<std::vector<int> > triid;
    for (p2t::Triangle* pt : tri){
      std::vector<int> ptid(3);
      for (int i=0; i<3; ++i){
        Point* pp = pt->GetPoint(i);
        ptid[i] = mp[pp];
      }
      triid.push_back(ptid);
    }
    return triid;
}

std::list<p2t::Triangle*> CDT::GetMap()
{
  return sweep_context_->GetMap();
}

CDT::~CDT()
{
  delete sweep_context_;
  delete sweep_;
}

struct pair_hash {
  template <typename T1, typename T2>
  std::size_t operator () (const std::pair<T1, T2> &p) const {
    std::size_t h1 = std::hash<T1>()(p.first);
    std::size_t h2 = std::hash<T2>()(p.second);
    return h1 ^ h2;
  }
};

bool tri(float *seg, int seglen, int **index/*, vector<int> &dbg*/)
{
  typedef long long lint;
  typedef vector<int> vi;
  typedef pair<int,int> pii;
  typedef vector<lint> vl;
  typedef pair<lint,lint> pll;

  std::unordered_map<pll, float*, pair_hash> to_real;
  std::unordered_map<pll, vector<pll>, pair_hash> mp;
  std::unordered_map<pll, pair<int, int>, pair_hash> mid;


  int trac = 6;
  lint B = 1e8;
  for (int i = 0; i< seglen; ++i){
    vl lseg(4);
    for (int j = 0; j<4; ++j) lseg[j] = B * seg[i*4 + j];
    pll from = {lseg[0], lseg[1]};
    pll to = {lseg[2], lseg[3]};
    to_real[from] = seg+i*4;
    to_real[to] = seg+i*4+2;
    mp[from].push_back(to);
    mp[to].push_back(from);
    mid[from] = {i, 0};
    mid[to] = {i, 1};
  }

  //check map
  //assert(mp.size() == seglen);
  {
    vector<pll> leafs;
    for (auto it = mp.begin(); it != mp.end(); ++it){
      assert(it->second.size() <= 2);
      if (it->second.size() == 1){
        leafs.push_back(it->first);
      }
    }
    assert(leafs.size() % 2 == 0);
    cout<<"LEAF SIZE "<<leafs.size()<<endl;
    for (pll lf : leafs){
      pll nearest_nb = lf;
      double mins = std::numeric_limits<double>::max();
      for (pll nb : leafs) if (nb != lf){
        double dist = fabs(nb.first - lf.first) + fabs(nb.second - lf.second);
        if (dist < mins){
          mins = dist;
          nearest_nb = nb;
        }
      }
      assert(nearest_nb != lf);
      mp[lf].push_back(nearest_nb);
    }
    for (auto it = mp.begin(); it != mp.end(); ++it){
      assert(it->second.size() == 2);

    }
  }

  vector<vector<pll>> polys;

  while(mp.size())
  {
    vector<pll> poly;
    auto it = mp.begin();
    poly.push_back(it->first);
    poly.push_back(it->second[0]);
    mp.erase(it);
    while(poly.back() != poly.front()){
      pll torm = poly.back();
      vector<pll> nb = mp[poly.back()];
      for (pll nxt : nb){
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

  vi ans;
  ans.push_back(polys.size());
   for (int i = 0; i< polys.size(); ++i) {
     ans.push_back(0);
   }
  for (int i = 0; i< polys.size(); ++i) {
    auto poly = polys[i];
    vector<p2t::Point*> polyline;

    for (pll ns : poly) {
      polyline.push_back(new  Point(1.0 * ns.first/B, 1.0*ns.second/B));
    }

    CDT* cdt = new CDT(polyline);
    vector<p2t::Point*> add;

    cdt->Triangulate();


//    auto triangles = cdt->GetTriangles();
    vector<p2t::Point*> totp;
    std::vector<std::vector<int> > triids;
    {
      triids = cdt->GetTrianglesIndexOfUnsortInput();
      totp = polyline;
    }


    ans[i+1] = (triids.size());
    {
      for (vector<int> tri : triids){
        for(int id : tri){

          ans.push_back(mid[poly[id]].first);
          ans.push_back(mid[poly[id]].second);
        }

      }

    }
  }

  int *ids = new int[ans.size()];
  memcpy(ids, ans.data(), ans.size() * sizeof(int));
  *index = ids;
//  dbg = ans;
  return true;
}

}

