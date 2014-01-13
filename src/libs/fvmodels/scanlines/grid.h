
/***************************************************************************
 *  grid.h - Scanline model implementation: grid
 *
 *  Created: Sun May 08 21:54:49 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_SCANLINE_GRID_H_
#define __FIREVISION_SCANLINE_GRID_H_

#include <fvmodels/scanlines/scanlinemodel.h>
#include <fvutils/base/roi.h>
#include <fvutils/base/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ScanlineGrid : public ScanlineModel
{

 public:

        ScanlineGrid(unsigned int width, unsigned int height,
               unsigned int offset_x, unsigned int offset_y,
               ROI* roi = NULL, bool horizontal_grid = true);
        virtual ~ScanlineGrid();

  fawkes::upoint_t    operator*();
  fawkes::upoint_t *  operator->();
  fawkes::upoint_t *  operator++();
  fawkes::upoint_t *  operator++(int);

  bool          finished();
  void          reset();
  const char *  get_name();
  unsigned int  get_margin();

  virtual void  set_robot_pose(float x, float y, float ori);
  virtual void  set_pan_tilt(float pan, float tilt);
  virtual void  set_roi(ROI* roi = NULL);

  void setDimensions(unsigned int width, unsigned int height, ROI* roi = NULL);
  void setOffset(unsigned int offset_x, unsigned int offset_y);
  void setGridParams(unsigned int width, unsigned int height,
                     unsigned int offset_x, unsigned int offset_y,
                     ROI* roi = NULL, bool horizontal_grid = true);

 private:
  unsigned int width;
  unsigned int height;
  unsigned int offset_x;
  unsigned int offset_y;

  ROI* roi;

  bool horizontal_grid;
  bool more_to_come;

  fawkes::upoint_t coord;
  fawkes::upoint_t tmp_coord;

  void calc_next_coord();
};

} // end namespace firevision

#endif
