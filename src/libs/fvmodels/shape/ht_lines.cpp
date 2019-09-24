
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

#include <fvmodels/shape/ht_lines.h>
#include <sys/time.h>
#include <utils/math/angle.h>

#include <cmath>

using namespace std;
using namespace fawkes;

namespace firevision {

#define TEST_IF_IS_A_PIXEL(x) ((x) > 230)

/** @class HtLinesModel <fvmodels/shape/ht_lines.h>
 * Hough-Transform line matcher.
 */

/** Constructor.
  * Creates a new HtLinesModel instance
  * @param nr_candidates the nr of candidates that is considered per pixel (the hole angle
  *                      range is devided in this many parts/lines
  * @param angle_from The angle to start the candidates from, given in rad, 0 is straight up
  * @param angle_range the angle range the candidates are taken from starting at angle_from,
  *                    given in rad, can be used for example to only search for horizontal lines
  * @param r_scale This can be done to reduce the size of the hough space and to map more lines
  *                to one line
  * @param min_votes_ratio The minimum ratio num_votes_per_line/total_num_votes that we have to
  *                        have before a point in the hough space is considered to be a line,
  *                        this may actually be higher if you use min_votes and set it to a higher
  *                        number (set min_votes to 0 to only use min_votes_ration)
  * @param min_votes the minimum number of votes a point in the hough space has to have before it
  *                  is considered to be a line. The number may actually be higher if min_votes_ratio
  *                  is set too high (set min_votes_ration to 0 to use only min_votes)
  */
HtLinesModel::HtLinesModel(unsigned int nr_candidates,
                           float        angle_from,
                           float        angle_range,
                           int          r_scale,
                           float        min_votes_ratio,
                           int          min_votes)
{
	RHT_NR_CANDIDATES = nr_candidates;

	RHT_R_SCALE = r_scale;

	RHT_MIN_VOTES       = min_votes;
	RHT_MIN_VOTES_RATIO = min_votes_ratio;

	RHT_ANGLE_FROM      = angle_from - (floor(angle_from / (2 * M_PI)) * (2 * M_PI));
	RHT_ANGLE_RANGE     = angle_range - (floor(angle_range / (2 * M_PI)) * (2 * M_PI));
	RHT_ANGLE_INCREMENT = RHT_ANGLE_RANGE / RHT_NR_CANDIDATES;
}

/** Destructor. */
HtLinesModel::~HtLinesModel(void)
{
	m_Lines.clear();
}

int
HtLinesModel::parseImage(unsigned char *buf, ROI *roi)
{
	unsigned char *buffer = roi->get_roi_buffer_start(buf);

	// clear the accumulator
	accumulator.reset();

	// clear all the remembered lines
	m_Lines.clear();

	// First, find all the edge pixels,
	// and store them in the 'pixels' vector.
	unsigned char *  line_start = buffer;
	unsigned int     x, y;
	vector<upoint_t> pixels;

	for (y = 0; y < roi->height; ++y) {
		for (x = 0; x < roi->width; ++x) {
			if (TEST_IF_IS_A_PIXEL(*buffer)) {
				upoint_t pt = {x, y};
				pixels.push_back(pt);
			}
			// NOTE: this assumes roi->pixel_step == 1
			++buffer;
		}
		line_start += roi->line_step;
		buffer = line_start;
	}

	// Then perform the RHT algorithm
	upoint_t                   p;
	float                      r, phi; // used for line representation
	vector<upoint_t>::iterator pos;
	if (pixels.size() == 0) {
		// No edge pixels found => no lines
		return 0;
	}

	while (pixels.size() > 0) {
		p = pixels.back();
		pixels.pop_back();

		for (unsigned int i = 0; i < RHT_NR_CANDIDATES; ++i) {
			phi = RHT_ANGLE_FROM + i * RHT_ANGLE_INCREMENT;
			r   = p.x * cos(phi) + p.y * sin(phi);

			int angle = (int)round(fawkes::rad2deg(phi));

			accumulator.accumulate((int)round(r / RHT_R_SCALE), angle, 0);
		}
	}

	// Find the most dense region, and decide on the lines
	int max, r_max, phi_max, any_max;
	max = accumulator.getMax(r_max, phi_max, any_max);

	roi_width  = roi->width;
	roi_height = roi->height;

	LineShape l(roi->width, roi->height);
	l.r     = r_max * RHT_R_SCALE;
	l.phi   = phi_max;
	l.count = max;
	m_Lines.push_back(l);

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
		return const_cast<LineShape *>(&m_Lines[id]); // or use const Shape* def?!...
	}
}

LineShape *
HtLinesModel::getMostLikelyShape(void) const
{
	if (m_Lines.size() == 0) {
		return NULL;
	} else if (m_Lines.size() == 1) {
		return const_cast<LineShape *>(&m_Lines[0]); // or use const Shape* def?!...
	} else {
		int cur = 0;
		for (unsigned int i = 1; i < m_Lines.size(); ++i) {
			if (m_Lines[i].count > m_Lines[cur].count) {
				cur = i;
			}
		}
		return const_cast<LineShape *>(&m_Lines[cur]); // or use const Shape* definition?!...
	}
}

/** Get all lines found.
 * @return vector with all line shapes.
 */
vector<LineShape> *
HtLinesModel::getShapes()
{
	int votes = (int)(accumulator.getNumVotes() * (float)RHT_MIN_VOTES_RATIO);

	if (RHT_MIN_VOTES > votes) {
		votes = RHT_MIN_VOTES;
	}

	vector<LineShape> *rv = new vector<LineShape>();

	vector<vector<int>> *         rht_nodes = accumulator.getNodes(votes);
	vector<vector<int>>::iterator node_it;

	LineShape l(roi_width, roi_height);

	for (node_it = rht_nodes->begin(); node_it != rht_nodes->end(); ++node_it) {
		l.r   = node_it->at(0) * RHT_R_SCALE;
		l.phi = node_it->at(1);
		// we do not use val 2 here!
		l.count = node_it->at(3);
		l.calcPoints();
		rv->push_back(l);
	}

	return rv;
}

} // end namespace firevision
