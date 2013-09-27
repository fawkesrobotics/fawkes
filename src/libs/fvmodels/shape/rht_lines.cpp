
/***************************************************************************
 *  rht_lines.cpp - Implementation of a lines shape finder
 *                   with Randomized Hough Transform
 *
 *  Created: Mon Sep 26 2005 09:52:00
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

#include <utils/math/angle.h>
#include <sys/time.h>
#include <fvmodels/shape/rht_lines.h>

using namespace std;
using namespace fawkes;

#define TEST_IF_IS_A_PIXEL(x) ((x)>230)

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class RhtLinesModel <fvmodels/shape/rht_lines.h>
 * Randomized Hough-Transform line model.
 */

/** Constructor. */
RhtLinesModel::RhtLinesModel(float max_time, int max_iter, unsigned int nr_candidates, float angle_from, float angle_range, int r_scale, float min_votes_ratio, int min_votes)
{
  RHT_MAX_TIME =  max_time; // max_time is given in ms but we need microseconds, thus * 1000
  RHT_MAX_ITER =  max_iter; // Maximal number of iterations.

  RHT_NR_CANDIDATES   = nr_candidates;

  RHT_R_SCALE         = r_scale;

  RHT_MIN_VOTES       = min_votes;
  RHT_MIN_VOTES_RATIO = min_votes_ratio;

  RHT_ANGLE_FROM      = angle_from -  (floor(angle_from  / (2 * M_PI )) * (2 * M_PI));
  RHT_ANGLE_RANGE     = angle_range - (floor(angle_range / (2 * M_PI )) * (2 * M_PI));
  RHT_ANGLE_INCREMENT = RHT_ANGLE_RANGE / RHT_NR_CANDIDATES;
}


/** Destructor. */
RhtLinesModel::~RhtLinesModel(void)
{
  m_Lines.clear();
}

/**************************************************************
 * In this function we implement a lines detection algorithm
 **************************************************************/
int
RhtLinesModel::parseImage( unsigned char *buf,
         ROI *roi            )
{
  unsigned char *buffer = roi->get_roi_buffer_start(buf);

  struct timeval   start, now;

  // clear the accumulator
  accumulator.reset();

  // clear all the remembered lines
  m_Lines.clear();

  // First, find all the edge pixels,
  // and store them in the 'pixels' vector.
  unsigned char *line_start = buffer;
  unsigned int     x, y;
  vector<upoint_t>  pixels;

  gettimeofday(&start, NULL);

  for (y = 0; y < roi->height; ++y) {
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
  upoint_t p;
  float r, phi; // used for line representation
  vector< upoint_t >::iterator pos;
  int num_iter = 0;
  if (pixels.size() == 0) {
    // No edge pixels found => no lines
    return 0;
  }



  do {
    // in order to prevent float exception, pixels.size() must be non-zero
    if (pixels.size() > 0) {
      int ri = rand() % pixels.size();
      pos = pixels.begin() + ri;
      p = *pos;
      pixels.erase(pos);

      for (unsigned int i = 0; i < RHT_NR_CANDIDATES; ++i) {
  phi = RHT_ANGLE_FROM + i * RHT_ANGLE_INCREMENT;
  r   = p.x * cos( phi )  +   p.y * sin( phi );

  int angle = (int)round(fawkes::rad2deg( phi ));

  accumulator.accumulate( (int)round(r / RHT_R_SCALE),
        angle,
        0 );
      }

      gettimeofday(&now, NULL);

      diff_sec  = now.tv_sec  - start.tv_sec;
      diff_usec = now.tv_usec - start.tv_usec;
      if (diff_usec < 0) {
  diff_sec  -= 1;
  diff_usec += 1000000;
      }

      f_diff_sec = diff_sec + diff_usec / 1000000.f;

    } // end if
  } while( ( ++num_iter < RHT_MAX_ITER) &&
     ( f_diff_sec < RHT_MAX_TIME) );

  // Find the most dense region, and decide on the lines
  int max, r_max, phi_max, any_max;
  max = accumulator.getMax(r_max, phi_max, any_max);

  roi_width = roi->width;
  roi_height = roi->height;

  LineShape l(roi->width, roi->height);
  l.r   = r_max * RHT_R_SCALE;
  l.phi = phi_max;
  l.count = max;
  m_Lines.push_back( l );

  return 1;
}


int
RhtLinesModel::getShapeCount(void) const
{
  return m_Lines.size();
}

LineShape *
RhtLinesModel::getShape(int id) const
{
  if (id < 0 || (unsigned int)id >= m_Lines.size()) {
    return NULL;
  } else {
    return const_cast<LineShape*>(&m_Lines[id]); // or use const Shape* def?!...
  }
}


LineShape *
RhtLinesModel::getMostLikelyShape(void) const
{
  if (m_Lines.size() == 0) {
    return NULL;
  } else if (m_Lines.size() == 1) {
    return const_cast<LineShape*>(&m_Lines[0]); // or use const Shape* def?!...
  } else {
    int cur=0;
    for (unsigned int i=1; i < m_Lines.size(); ++i) {
      if (m_Lines[i].count > m_Lines[cur].count) {
  cur = i;
      }
    }
    return const_cast<LineShape*>(&m_Lines[cur]); // or use const Shape* definition?!...
  }
}


/** Get shapes.
 * @return vector of shapes
 */
vector< LineShape > *
RhtLinesModel::getShapes()
{
  int votes = (int)(accumulator.getNumVotes() * (float)RHT_MIN_VOTES_RATIO);

  if ( RHT_MIN_VOTES > votes ) {
    votes = RHT_MIN_VOTES;
  }

  vector< LineShape > * rv = new vector< LineShape >();

  vector< vector< int > > *rht_nodes = accumulator.getNodes( votes );
  vector< vector< int > >::iterator node_it;

  LineShape l(roi_width, roi_height);

  for (node_it = rht_nodes->begin(); node_it != rht_nodes->end(); ++node_it) {
    l.r   = node_it->at(0) * RHT_R_SCALE;
    l.phi = node_it->at(1);
    // we do not use val 2 here!
    l.count = node_it->at(3);
    l.calcPoints();
    rv->push_back( l );
  }

  return rv;
}

} // end namespace firevision
