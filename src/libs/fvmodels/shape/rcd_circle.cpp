
/***************************************************************************
 *  rcd_circle.cpp - Implementation of a circle shape finder
 *
 *  Created: Thu May 16 00:00:00 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <fvmodels/shape/rcd_circle.h>
#include <cmath>
#include <sys/time.h>
#include <cstdlib>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define TBY_GRAYSCALE
#ifdef TBY_GRAYSCALE
#define TEST_IF_IS_A_PIXEL(x) ((x)>230)
#else
#define TEST_IF_IS_A_PIXEL(x) ((x)==0)
#endif // TBY_GRAYSCALE

#define TBY_SQUARED_DIST(x1,y1,x2,y2)                   \
  (((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2)))
#define TBY_RADIUS_DIFF(x1, y1, x2, y2, r)                      \
  (((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2))-(r)*(r))

/** @class RcdCircleModel <fvmodels/shape/rcd_circle.h>
 * RCD circle model from the following literature
 *  An Efficient Randomized Algorithm for Detecting Circles
 */

/** Create a new circle model which uses RCD to detect circles
 * @param max_failures Max. number of failures
 * @param min_pixels Min number of available edge pixels
 * @param min_interpix_dist Min distance between chosen pixels
 * @param max_dist_p4 Max. distance of fourth pixel to circle
 * @param max_dist_a Max. distance for all other pixels to circle
 * @param hw_ratio Ratio height/width
 * @param hollow_rate size of the hollow window in the ROI.
 * @param max_time Maximum runtime per loop
 */
RcdCircleModel::RcdCircleModel(unsigned int max_failures,
                               unsigned int min_pixels,
                               unsigned int min_interpix_dist,
                               unsigned int max_dist_p4,
                               unsigned int max_dist_a,
                               float        hw_ratio,
                               float        hollow_rate,
                               float        max_time
                               )
{

  RCD_MAX_FAILURES       = max_failures;
  RCD_MIN_PIXELS         = min_pixels;
  RCD_MIN_INTERPIX_DIST  = min_interpix_dist;
  RCD_MAX_DIST_P4        = max_dist_p4;
  RCD_MAX_DIST_A         = max_dist_a;
  RCD_HW_RATIO           = hw_ratio;
  RCD_MAX_TIME           = max_time;
  RCD_ROI_HOLLOW_RATE    = hollow_rate;

}

/** Destrcutor. */
RcdCircleModel::~RcdCircleModel(void)
{
  m_Circles.clear();
}

int RcdCircleModel::parseImage( unsigned char* buf,
                                ROI *roi )
{

  unsigned char *buffer = roi->get_roi_buffer_start(buf);
  unsigned char *line_start = buffer;

  unsigned int     x, y;
  vector<upoint_t>  pixels,
    remove_list;
  unsigned int     f = 0;       // number of failures
  int              count;       // number of pixels on the circle
  int              num_circles = 0;
  struct timeval   start, now;

  // clear all the remembered circles
  m_Circles.clear();

  const unsigned int roi_hollow_top     = (int)(roi->height * ((1.0f - RCD_ROI_HOLLOW_RATE) / 2));
  const unsigned int roi_hollow_bottom  = roi->height - roi_hollow_top;
  const unsigned int roi_hollow_left    = (int)(roi->width * ((1.0f - RCD_ROI_HOLLOW_RATE) / 2));
  const unsigned int roi_hollow_right   = roi->width - roi_hollow_left;

  // First, find all the pixels on the edges,
  // and store them in the 'pixels' vector.
  // NEW: excluding the hollow window
  buffer = roi->get_roi_buffer_start(buf);
  line_start = buffer;

  // Find the boundary of the ball,
  // following used for ball pixel threshold.
  unsigned int boundary_right   = 0;
  unsigned int boundary_bottom  = 0;

  gettimeofday(&start, NULL);

  pixels.clear();

  // top "1/3"
  for (y = 0; y < roi_hollow_top; ++y) {
    for (x = 0; x < roi->width; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
        upoint_t pt={x, y};
        pixels.push_back(pt);
        if (x > boundary_right) boundary_right = x;
        boundary_bottom = y;
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    line_start += roi->line_step;
    buffer = line_start;
  }
  // middle "1/3"
  for (y = roi_hollow_top; y < roi_hollow_bottom; ++y) {
    for (x = 0; x < roi_hollow_left; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
        upoint_t pt={x, y};
        pixels.push_back(pt);
        if (x > boundary_right) boundary_right = x;
        boundary_bottom = y;
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    buffer+=(roi_hollow_right - roi_hollow_left);
    for (x = roi_hollow_right; x < roi->width; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
        upoint_t pt={x, y};
        pixels.push_back(pt);
        if (x > boundary_right) boundary_right = x;
        boundary_bottom = y;
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    line_start += roi->line_step;
    buffer = line_start;
  }
  // bottom "1/3"
  for (y = roi_hollow_bottom; y < roi->height; ++y) {
    for (x = 0; x < roi->width; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
        upoint_t pt={x, y};
        pixels.push_back(pt);
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    line_start += roi->line_step;
    buffer = line_start;
  }

  // Then perform the RCD algorithm
  upoint_t p[4];
  center_in_roi_t center;
  float radius;
  vector< upoint_t >::iterator pos;

  if (pixels.size() < RCD_MIN_PIXELS) {
    return 0;
  }

  do {

    // Pick four points, and move them to the remove_list.
    for (int i=0; i < 4; ++i) {
      int ri = rand() % ((int)pixels.size());
      pos = pixels.begin() + ri;
      p[i] = *pos; // use * operator of iterator
      pixels.erase(pos);
      remove_list.push_back(p[i]);
    }

    if (TBY_SQUARED_DIST(p[1].x, p[1].y, p[2].x, p[2].y) < RCD_MIN_INTERPIX_DIST ||
        TBY_SQUARED_DIST(p[2].x, p[2].y, p[0].x, p[0].y) < RCD_MIN_INTERPIX_DIST ||
        TBY_SQUARED_DIST(p[0].x, p[0].y, p[1].x, p[1].y) < RCD_MIN_INTERPIX_DIST ) {
      // there are two points that are too near
      // to each other
      ++f;

      // restore all the pixels in remove_list to pixels
      pixels.push_back(p[0]);
      pixels.push_back(p[1]);
      pixels.push_back(p[2]);
      pixels.push_back(p[3]);

      remove_list.clear();

      gettimeofday(&now, NULL);
      continue;
    }

    // Now calculate the center and radius
    // based on the first three points.
    calcCircle(p[0], p[1], p[2], center, radius);

    // Test if the fourth point on this circle
    int r = (int)sqrt(TBY_SQUARED_DIST(center.x, center.y, pixels[3].x, pixels[3].y));
    int dist = (int)(r - radius);
    dist = (dist >= 0) ? dist : -dist;
    if (radius <= 0 || (unsigned int)dist > RCD_MAX_DIST_P4 ) {
      ++f;

      //restore all the pixels
      pixels.push_back(p[0]);
      pixels.push_back(p[1]);
      pixels.push_back(p[2]);
      pixels.push_back(p[3]);

      remove_list.clear();

      gettimeofday(&now, NULL);
      continue;
    }

    // count how many pixels are on the circle
    count=0;
    for (unsigned int i=0; i < pixels.size(); ++i) {
      int r = (int)sqrt(TBY_SQUARED_DIST(center.x, center.y, pixels[i].x, pixels[i].y));
      int dist = (int)(r-radius);
      dist = (dist >= 0) ? dist : -dist;
      if ((unsigned int)dist <= RCD_MAX_DIST_A) {
        pos = pixels.begin() + i;
        ++count;
        // move this pixel to the remove_list
        remove_list.push_back(pixels[i]);
        pixels.erase(pos--); // need "--"? not sure yet!
      }
    }

    // test if there are enough points on the circle
    // to convince us that this is indeed a circle
    if ( (float)count >
         ( boundary_right > boundary_bottom ?  boundary_right : boundary_bottom ) * RCD_HW_RATIO ) {
      // this is indeed a circle
      if ( radius > TBY_CIRCLE_RADIUS_MIN &&
           radius < TBY_CIRCLE_RADIUS_MAX  ) {
        // only ball of size in the range are saved in the candidate pool.

        // use least square fitting to reduce error
        Circle c;
        c.fitCircle(remove_list);
        c.count = count;

        // save circle
        m_Circles.push_back(c);
      }
      remove_list.clear();
      ++num_circles;
    } else {
      // not a circle, charge a failure
      ++f;

      while(!remove_list.empty()) {
        pixels.push_back(remove_list.back());
        remove_list.pop_back();
      }
      gettimeofday(&now, NULL);
      continue;
    }

      gettimeofday(&now, NULL);

      diff_sec  = now.tv_sec  - start.tv_sec;
      diff_usec = now.tv_usec - start.tv_usec;
      if (diff_usec < 0) {
        diff_sec  -= 1;
        diff_usec += 1000000;
      }

      f_diff_sec = diff_sec + diff_usec / 1000000.f;

  } while( (f < RCD_MAX_FAILURES) && (pixels.size() > RCD_MIN_PIXELS) &&
           ( f_diff_sec < RCD_MAX_TIME) );

  return num_circles;
}

int RcdCircleModel::getShapeCount(void) const
{
  return m_Circles.size();
}

Circle* RcdCircleModel::getShape(int id) const
{
  if (id < 0 || (unsigned int)id >= m_Circles.size())
    {
      return NULL;
    }
  else
    {
      return const_cast<Circle*>(&m_Circles[id]); // or use const Shape* def?!...
    }
}

Circle* RcdCircleModel::getMostLikelyShape(void) const
{
  int cur=0;
  switch (m_Circles.size())
    {
    case 0:
      return NULL;
    case 1:
      return const_cast<Circle*>(&m_Circles[0]); // or use const Shape* def?!...
    default:
      for (unsigned int i=1; i < m_Circles.size(); ++i)
        if (m_Circles[i].radius > m_Circles[cur].radius)
          cur = i;
      return const_cast<Circle*>(&m_Circles[cur]); // or use const Shape* definition?!...
    }
}

void RcdCircleModel::calcCircle(
                                const upoint_t& p1,
                                const upoint_t& p2,
                                const upoint_t& p3,
                                center_in_roi_t& center,
                                float& radius)
// Given three points p1, p2, p3,
// this function calculates the center and radius
// of the circle that is determined
{
  const int &x1=p1.x, &y1=p1.y, &x2=p2.x, &y2=p2.y, &x3=p3.x, &y3=p3.y;
  float dx, dy;
  int div = 2*((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));

  if (div == 0)
    {
      // p1, p2 and p3 are in a straight line.
      radius = -1.0;
      return;
    }
  center.x =    ((float)((x2*x2+y2*y2-x1*x1-y1*y1)*(y3-y1)
                         -(x3*x3+y3*y3-x1*x1-y1*y1)*(y2-y1))
                 /div);
  center.y =    ((float)((x2-x1)*(x3*x3+y3*y3-x1*x1-y1*y1)
                         -(x3-x1)*(x2*x2+y2*y2-x1*x1-y1*y1))
                 /div);
  dx = center.x - x1;
  dy = center.y - y1;
  radius        =       (float)sqrt(dx*dx+dy*dy);
}

} // end namespace firevision
