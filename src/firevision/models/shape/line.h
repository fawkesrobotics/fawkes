
/***************************************************************************
 *  line.h - Header of circle shape model
 *
 *  Generated: Tue Sep 27 2005 11:25:35
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *                   Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_LINE_H_
#define __FIREVISION_MODELS_SHAPE_LINE_H_

#include <vector>
#include <iostream>

#include <fvutils/types.h>
#include <fvutils/roi.h>
#include <models/shape/shapemodel.h>

class LineShape : public Shape
{
  friend class HtLinesModel;
  friend class RhtLinesModel;
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

#endif // __FIREVISION_MODELS_SHAPE_LINE_H_

