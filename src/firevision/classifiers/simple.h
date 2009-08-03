
/***************************************************************************
 *  simple.h - Header for ReallySimpleClassifier
 *
 *  Created: Wed May 18 11:39:10 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CLASSIFIERS_SIMPLE_H_
#define __FIREVISION_CLASSIFIERS_SIMPLE_H_

#include <classifiers/classifier.h>

class ScanlineModel;
class ColorModel;

class SimpleColorClassifier : public Classifier
{
 public:
  SimpleColorClassifier(ScanlineModel *scanline_model,
			ColorModel *color_model,
			unsigned int min_num_points=6,
			unsigned int box_extent = 50,
			bool upward = false,
			unsigned int neighbourhood_min_match = 8,
			unsigned int grow_by = 10                );

  virtual std::list< ROI > * classify();

  virtual void get_mass_point_of_ball( ROI *roi, fawkes::point_t *massPoint );

  /** Sets the object of interest (hint_t)
   * @param hint Object of interest
   */
  virtual void set_hint (hint_t hint);
  virtual void add_hint (hint_t hint);

 private:
  unsigned int consider_neighbourhood(unsigned int x, unsigned int y, color_t what);

  unsigned char *src;

  unsigned int width;
  unsigned int height;

  unsigned int neighbourhood_min_match;
  unsigned int grow_by;

  bool         modified;
  unsigned int min_num_points;
  unsigned int box_extent;

  bool         upward;

  ScanlineModel *scanline_model;
  ColorModel    *color_model;

  std::list<color_t> colors_of_interest;
};

#endif
