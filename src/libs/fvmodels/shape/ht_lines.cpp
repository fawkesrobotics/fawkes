
/***************************************************************************
 *  ht_lines.cpp - Implementation of a lines shape finder
 *                 with Randomized Hough Transform
 *
 *  Created: Fri Jan 13 12:42:53 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *                        Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#include <cmath>
#include <sys/time.h>
#include <fvmodels/shape/ht_lines.h>
#include <utils/math/angle.h>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define TEST_IF_IS_A_PIXEL(x) ((x)>230)

/** @class HtLinesModel <fvmodels/shape/ht_lines.h>
 * Hough-Transform line matcher.
 */

/** Constructor.
 * @param nr_candidates number of candidates
 * @param angle_from slope of lines from
 * @param angle_range angle range
 * @param r_scale r scale
 * @param min_votes_ratio min votes ratio
 * @param min_votes minimum number of votes for a candidate to consider
 */
HtLinesModel::HtLinesModel(unsigned int nr_candidates, float angle_from, float angle_range, int r_scale, float min_votes_ratio, int min_votes)
{
  RHT_NR_CANDIDATES   = nr_candidates;

  RHT_R_SCALE         = r_scale;

  RHT_MIN_VOTES       = min_votes;
  RHT_MIN_VOTES_RATIO = min_votes_ratio;

  RHT_ANGLE_FROM      = angle_from -  (floor(angle_from  / (2 * M_PI )) * (2 * M_PI));
  RHT_ANGLE_RANGE     = angle_range - (floor(angle_range / (2 * M_PI )) * (2 * M_PI));
  RHT_ANGLE_INCREMENT = RHT_ANGLE_RANGE / RHT_NR_CANDIDATES;
}


/** Destructor. */
HtLinesModel::~HtLinesModel(void)
{
  m_Lines.clear();
}

int
HtLinesModel::parseImage( unsigned char *buf,
         ROI *roi            )
{
  unsigned char *buffer = roi->get_roi_buffer_start(buf);

  // clear the accumulator
  accumulator.reset();

  // clear all the remembered lines
  m_Lines.clear();

  // First, find all the edge pixels,
  // and store them in the 'pixels' vector.
  unsigned char *line_start = buffer;
  unsigned int     x, y;
  vector<upoint_t>  pixels;

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
  if (pixels.size() == 0) {
    // No edge pixels found => no lines
    return 0;
  }


  while (pixels.size() > 0) {
    p = pixels.back();
    pixels.pop_back();

    for (unsigned int i = 0; i < RHT_NR_CANDIDATES; ++i) {
      phi = RHT_ANGLE_FROM + i * RHT_ANGLE_INCREMENT;
      r   = p.x * cos( phi )  +   p.y * sin( phi );

      int angle = (int)round(fawkes::rad2deg( phi ));

      accumulator.accumulate( (int)round(r / RHT_R_SCALE),
            angle,
            0 );
    }
  }


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
HtLinesModel::getShapeCount(void) const
{
  return m_Lines.size();
}

LineShape *
HtLinesModel::getShape(int id) const
{
  if (id < 0 || (unsigned int)id >= m_Lines.size()) {
    return NULL;
  } else {
    return const_cast<LineShape*>(&m_Lines[id]); // or use const Shape* def?!...
  }
}


LineShape *
HtLinesModel::getMostLikelyShape(void) const
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


/** Get all lines found.
 * @return vector with all line shapes.
 */
vector< LineShape > *
HtLinesModel::getShapes()
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
