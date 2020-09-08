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
 
#ifndef CDT_H
#define CDT_H

#include "advancing_front.h"
#include "sweep_context.h"
#include "sweep.h"

/**
 * 
 * @author Mason Green <mason.green@gmail.com>
 *
 */

/**
 *  @tparam  seg    data of segments, every segment has 4 float datas.
 *  @tparam  seglen     count of segments, seg should contain 4*seglen float numbers.
 *  @tparam  index   result of triangulars index.
 *  index[0] = cnt : number of polygons
 *  index[1] - index[cnt] number of triangulars in each polygon. there will be 6*index[1] int
 * data for polygon 1.
 *  index[cnt+1] - ...   triangulars index, each triangular is pressed by 6 numbers:
 * seg id of angular 0
 * point id of prev seg
 * seg id of angular 1
 * point id of prev seg
 * ...
 */
#ifdef linux
#define API
#else
#define API __declspec(dllexport)
#endif


extern "C"
{
 API int tri(float *seg, int seglen, int ** index/*, std::vector<int> &dbg*/);
 API void freeIndex(int *p);
 API int qtGui();
}

 
namespace p2t {

class CDT
{
public:

  /**
   * Constructor - add polyline with non repeating points
   * 
   * @param polyline
   */
  CDT(std::vector<Point*> polyline);
  
   /**
   * Destructor - clean up memory
   */
  ~CDT();
  
  /**
   * Add a hole
   * 
   * @param polyline
   */
  void AddHole(std::vector<Point*> polyline);
  
  /**
   * Add a steiner point
   * 
   * @param point
   */
  void AddPoint(Point* point);
  
  /**
   * Triangulate - do this AFTER you've added the polyline, holes, and Steiner points
   */
  void Triangulate();
  
  /**
   * Get CDT triangles
   */
  std::vector<Triangle*> GetTriangles();

  std::vector<std::vector<int> > GetTrianglesIndex(std::vector<Point *> &totp);
  std::vector<std::vector<int> > GetTrianglesIndexOfUnsortInput();
  
  /**
   * Get triangle map
   */
  std::list<Triangle*> GetMap();

  private:

  /**
   * Internals
   */
   
  SweepContext* sweep_context_;
  Sweep* sweep_;
  std::vector<Point*> unsort_polyline;
};

}

#endif
