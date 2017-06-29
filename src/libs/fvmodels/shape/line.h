
/***************************************************************************
 *  line.h - Header of circle shape model
 *
 *  Created: Tue Sep 27 11:25:35 2005
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

#ifndef __FIREVISION_MODELS_SHAPE_LINE_H_
#define __FIREVISION_MODELS_SHAPE_LINE_H_

#include <vector>
#include <iostream>

#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>
#include <fvmodels/shape/shapemodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class HtLinesModel;
class RhtLinesModel;

class LineShape : public Shape
{
 friend HtLinesModel;
 friend RhtLinesModel;

 public:
  LineShape(unsigned int roi_width, unsigned int roi_height);
  ~LineShape();

  void printToStream(std::ostream &stream);
  void setMargin( unsigned int margin );
  bool isClose(unsigned int in_roi_x, unsigned int in_roi_y);

  void calcPoints();
  void getPoints(int *x1, int *y1, int *x2, int *y2);

 private:
  float r;
  float phi;
  int			count;
  unsigned int          margin;
  int                   max_length;

  unsigned int  roi_width;
  unsigned int  roi_height;

  float last_calc_r;
  float last_calc_phi;

  int x1;
  int y1;
  int x2;
  int y2;


};

} // end namespace firevision

#endif // __FIREVISION_MODELS_SHAPE_LINE_H_
