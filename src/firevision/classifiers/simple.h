
/***************************************************************************
 *  simple.h - Header for ReallySimpleClassifier
 *
 *  Generated: Wed May 18 11:39:10 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_COLORCLASSIFIER_SIMPLE_H_
#define __FIREVISION_COLORCLASSIFIER_SIMPLE_H_

#include "classifiers/classifier.h"

class ScanlineModel;
class ColorModel;

class ReallySimpleClassifier : public Classifier
{
 public:

  ReallySimpleClassifier(unsigned int width, unsigned int height,
			 ScanlineModel *scanline_model,
			 ColorModel *color_model,
			 unsigned int min_num_points=6,
			 unsigned int box_extent = 50,
			 unsigned int neighbourhood_min_match = 8,
                         unsigned int grow_by = 10                );

  /* NOTE: This buffer must be YUV422_PLANAR!
   */
  virtual void setSrcBuffer(unsigned char *buf);

  virtual const char *  getName() const;

  virtual std::list< ROI > * classify();

  /* Method "getMassPointOfBall"
   *   calculates mass point of all orange pixels
   *   that occur within "roi",
   *   and writes result to "*massPoint"
   */
  virtual void getMassPointOfBall( ROI *roi, 
				   cart_coord_t *massPoint );

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

  ScanlineModel *scanline_model;
  ColorModel    *color_model;

};

#endif
