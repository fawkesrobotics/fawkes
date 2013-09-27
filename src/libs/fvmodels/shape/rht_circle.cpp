
/***************************************************************************
 *  rht_circle.cpp - Implementation of a circle shape finder
 *                   with Randomized Hough Transform
 *
 *  Created: Tue Jun 28 00:00:00 2005
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *                   Tim Niemueller [www.niemueller.de]
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

#include <fvmodels/shape/rht_circle.h>

#include <cmath>
#include <sys/time.h>

using namespace std;
using namespace fawkes;

#define TEST_IF_IS_A_PIXEL(x) ((x)>230)

#define TBY_SQUARED_DIST(x1,y1,x2,y2) \
    (((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2)))
#define TBY_RADIUS_DIFF(x1, y1, x2, y2, r) \
    (sqrt(((x1)-(x2))*((x1)-(x2))+((y1)-(y2))*((y1)-(y2))-(r)*(r)))


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

const float RhtCircleModel::RHT_MIN_RADIUS = 40.0;
const float RhtCircleModel::RHT_MAX_RADIUS = 500.0;

/** @class RhtCircleModel <fvmodels/shape/rht_circle.h>
 * Randomized Hough-Transform circle model.
 */

/** Constructor. */
RhtCircleModel::RhtCircleModel(void)
{
}


/** Destructor. */
RhtCircleModel::~RhtCircleModel(void)
{
  m_Circles.clear();
}

/**************************************************************
 * In this function I implement the circle detection algorithm
 * from the following literature
 *  Randomized Hough Transform
 **************************************************************/
int
RhtCircleModel::parseImage( unsigned char* buf,
          ROI *roi )
{

  unsigned char *buffer = roi->get_roi_buffer_start(buf);

  unsigned int     x, y;
  vector<upoint_t>  pixels;
  struct timeval   start, end;

  // clear the accumulator
  accumulator.reset();

  // clear all the remembered circles
  m_Circles.clear();

  // The following constants are used as stopping criteria
  const int             RHT_MAX_TIME    = 10000; // = 10ms (is given in microseconds)
  const int   RHT_MAX_ITER  =  1000; // Maximal number of iterations.

  // The following constant is used for eliminating circles with too few votes
  const float   RHT_MIN_VOTE_RATE = 0.0f;

  // The following constants are used for RHT accumulator precision
  const int   RHT_XY_SCALE  = 8;
  const int   RHT_RADIUS_SCALE= 8;

  // All the pixels with a less distance difference than below
  // are taken into account for circle fitting.
  const float   RHT_FITTING_DIST_DIFF = 15.0f;

  // The following constant is used for the size of the hollow window in the ROI.
  const float   ROI_HOLLOW_RATE = 0.70f;

  const unsigned int roi_hollow_top = (int)(roi->height * ((1.0f - ROI_HOLLOW_RATE) / 2));
  const unsigned int roi_hollow_bottom  = roi->height - roi_hollow_top;
  const unsigned int roi_hollow_left  = (int)(roi->width * ((1.0f - ROI_HOLLOW_RATE) / 2));
  const unsigned int roi_hollow_right = roi->width - roi_hollow_left;

  // First, find all the pixels on the edges,
  // and store them in the 'pixels' vector.
  // NEW: excluding the hollow window
  unsigned char *line_start = buffer;

  gettimeofday(&start, NULL);
  end.tv_usec = start.tv_usec;

  // top "1/3"
  for (y = 0; y < roi_hollow_top; ++y) {
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
  // middle "1/3"
  for (y = roi_hollow_top; y < roi_hollow_bottom; ++y) {
    for (x = 0; x < roi_hollow_left; ++x) {
      if (TEST_IF_IS_A_PIXEL(*buffer)) {
  upoint_t pt={x, y};
  pixels.push_back(pt);
      }
      // NOTE: this assumes roi->pixel_step == 1
      ++buffer;
    }
    buffer+=(roi_hollow_right - roi_hollow_left);
    for (x = roi_hollow_right; x < roi->width; ++x) {
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

  // Then perform the RHT algorithm
  upoint_t p[3];
  center_in_roi_t center;
  float radius;
  vector< upoint_t >::iterator pos;
  int num_iter = 0;
  int num_points = (int) pixels.size();
  if (num_points == 0) {
    // No pixels found => no edge => no circle
    return 0;
  }

  while( (num_iter++ < RHT_MAX_ITER) &&
   ( ((end.tv_usec - start.tv_usec) < RHT_MAX_TIME) ||
     ((end.tv_usec + 1000000 - start.tv_usec) < RHT_MAX_TIME) )
           // this only works if time constraint smaller than 500ms
   ) {

    // Pick three points, and move them to the remove_list.
    for (int i=0; i < 3; ++i) {
      int ri = rand() % num_points;
      pos = pixels.begin() + ri;
      p[i] = *pos; // use * operator of iterator
    }

    // Now calculate the center and radius
    // based on the three points.
    calcCircle(p[0], p[1], p[2], center, radius);

    // Accumulate this circle to the 3-D space...
    if (radius > RHT_MIN_RADIUS && radius < RHT_MAX_RADIUS) {
      accumulator.accumulate((int)(center.x / RHT_XY_SCALE),
           (int)(center.y / RHT_XY_SCALE),
           (int)(radius / RHT_RADIUS_SCALE));
    }

    gettimeofday(&end, NULL);
  }

  // Find the most dense region, and decide on the ball.
  int max, x_max, y_max, r_max;
  max = accumulator.getMax(x_max, y_max, r_max);

  cout << "Max vote is with " << max << " of the " << num_iter << " ones." << endl;

  // Only when votes exceeds a ratio will it be considered a real circle
  if (max > num_iter * RHT_MIN_VOTE_RATE)
  {
    center_in_roi_t center;
    center.x = (float)(x_max * RHT_XY_SCALE + RHT_XY_SCALE/2);
    center.y = (float)(y_max * RHT_XY_SCALE + RHT_XY_SCALE/2);
    float c_r = (float)(r_max * RHT_RADIUS_SCALE + RHT_RADIUS_SCALE/2);

    // With circle fitting
    for(vector< upoint_t >::iterator pos = pixels.begin(); pos != pixels.end(); )
    {
      if (TBY_RADIUS_DIFF(pos->x, pos->y, center.x, center.y, c_r)
        > RHT_FITTING_DIST_DIFF)
      {
        pixels.erase(pos);
      }
      else
      {
        pos++;
      }
    }

    Circle c;
    c.fitCircle(pixels);
    c.count = max;
    m_Circles.push_back(c);

    /*
    // Without circle fitting
    m_Circles.push_back(Circle(center, c_r, max));
    */

    return 0;
  }

  // Return... here in this algorithm, we only find at most one most likely circle,
  // (if none found, return-ed above)
  // so the return value here is always 1
  return 1;
}


int
RhtCircleModel::getShapeCount(void) const
{
  return m_Circles.size();
}

Circle *
RhtCircleModel::getShape(int id) const
{
  if (id < 0 || (unsigned int)id >= m_Circles.size()) {
    return NULL;
  } else {
    return const_cast<Circle*>(&m_Circles[id]); // or use const Shape* def?!...
  }
}


Circle *
RhtCircleModel::getMostLikelyShape(void) const
{
  if (m_Circles.size() == 0) {
    return NULL;
  } else if (m_Circles.size() == 1) {
    return const_cast<Circle*>(&m_Circles[0]); // or use const Shape* def?!...
  } else {
    int cur=0;
    for (unsigned int i=1; i < m_Circles.size(); ++i) {
      if (m_Circles[i].count > m_Circles[cur].count) {
  cur = i;
      }
    }
    return const_cast<Circle*>(&m_Circles[cur]); // or use const Shape* definition?!...
  }
}


void
RhtCircleModel::calcCircle(
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
  center.x =  ((float)((x2*x2+y2*y2-x1*x1-y1*y1)*(y3-y1)
       -(x3*x3+y3*y3-x1*x1-y1*y1)*(y2-y1))
     /div);
  center.y =  ((float)((x2-x1)*(x3*x3+y3*y3-x1*x1-y1*y1)
       -(x3-x1)*(x2*x2+y2*y2-x1*x1-y1*y1))
     /div);
  dx = center.x - x1;
  dy = center.y - y1;
  radius  = (float)sqrt(dx*dx+dy*dy);
}

} // end namespace firevision
