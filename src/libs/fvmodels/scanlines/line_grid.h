
/***************************************************************************
 *  line_grid.h - Scanline model implementation: line grid
 *
 *  Created: Wen Mar 25 17:31:00 2009
 *  Copyright  2009 Christof Rath <c.rath@student.tugraz.at>
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

#ifndef __FIREVISION_SCANLINE_LINE_GRID_H_
#define __FIREVISION_SCANLINE_LINE_GRID_H_

#include "scanlinemodel.h"
#include <fvutils/base/types.h>
#include <fvutils/color/yuv.h>

#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class ScanlineLineGrid : public ScanlineModel
{
 private:
  typedef std::list<fawkes::upoint_t> point_list_t;

 public:
  ScanlineLineGrid(unsigned int width, unsigned int height,
                   unsigned int offset_hor, unsigned int offset_ver,
                   ROI* roi = NULL, unsigned int gap = 0);
  virtual ~ScanlineLineGrid();

  fawkes::upoint_t operator*();
  fawkes::upoint_t * operator->();
  fawkes::upoint_t * operator++();
  fawkes::upoint_t * operator++(int);

  bool finished();
  void reset();
  const char * get_name();
  unsigned int get_margin();

  virtual void set_robot_pose(float x, float y, float ori);
  virtual void set_pan_tilt(float pan, float tilt);

  virtual void set_dimensions(unsigned int width, unsigned int height, ROI* roi = NULL);
  virtual void set_offset(unsigned int offset_x, unsigned int offset_y);
  virtual void set_grid_params(unsigned int width, unsigned int height,
                             unsigned int offset_hor, unsigned int offset_ver, ROI* roi = NULL);
  virtual void set_roi(ROI* roi = NULL);

 private:
  unsigned int __width;
  unsigned int __height;
  unsigned int __offset_ver;
  unsigned int __offset_hor;
  unsigned int __next_pixel;

  ROI*         __roi;

  point_list_t           __point_list;
  point_list_t::iterator __cur;

  void calc_coords();
};

} // end namespace firevision

#endif //__FIREVISION_SCANLINE_LINE_GRID_H_
