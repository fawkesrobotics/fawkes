
/***************************************************************************
 *  line_grid.cpp - Implementation of the line grid scanline model
 *
 *  Created: Wed Mar 25 17:31:00 2009
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

#include <fvmodels/scanlines/line_grid.h>

#include <fvutils/base/roi.h>
#include <fvutils/draw/drawer.h>
#include <core/exceptions/software.h>

#include <cstring>

using fawkes::upoint_t;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ScanlineLineGrid <fvmodels/scanlines/line_grid.h>
 * Grid of scan lines.
 * A grid of scan lines (i.e. horizontal and/or vertical lines) instead of only
 * points on the grid crossings.
 * The behavior of the ScanlineGrid (grid.h) class can be modeled if offset_hor
 * is set to the same value as offset_x in the Grid class, offset_ver = 0 and
 * gap is set to offset_y - 1. The advantage of doing this is a performance gain
 * as the LineGrid is pre-calculated and getting the next point is only an
 * iterator increment.
 */

/** Constructor.
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 * @param gap Gap between two points on the line
 */
ScanlineLineGrid::ScanlineLineGrid(unsigned int width, unsigned int height,
         unsigned int offset_hor, unsigned int offset_ver,
         ROI* roi, unsigned int gap)
{
  __roi = NULL;
  __next_pixel = gap + 1;
  set_grid_params(width, height,
                offset_hor, offset_ver, roi);
  //reset is done in set_grid_params ()
}

/** Destructor
 */
ScanlineLineGrid::~ScanlineLineGrid()
{
  delete __roi;
}

upoint_t
ScanlineLineGrid::operator*()
{
  return *__cur;
}

upoint_t*
ScanlineLineGrid::operator->()
{
  return &*__cur;
}

void
ScanlineLineGrid::calc_coords()
{
  __point_list.clear();
  bool more_to_come = true;
  upoint_t coord;
  unsigned int next_px;

  if (__offset_hor > 0) //horizontal lines
  {
    more_to_come = true;
    next_px = std::min(__next_pixel, __offset_ver ? __offset_ver : __width);
    coord.x = __roi->start.x;
    coord.y = __roi->start.y + ((__roi->height - 1) % __offset_hor) / 2; //Center the horizontal lines in the image
    __point_list.push_back(coord);

    while (more_to_come) {
      if (coord.x < (__roi->image_width - next_px))
      {
        coord.x += next_px;
      }
      else
      {
        if (coord.y < (__roi->image_height - __offset_hor))
        {
          coord.x = __roi->start.x;
          coord.y += __offset_hor;
        }
        else
        {
          more_to_come = false;
        }
      }

      if (more_to_come) __point_list.push_back(coord);
    }
  }

  if (__offset_ver > 0) //vertical lines
  {
    more_to_come = true;
    next_px = std::min(__next_pixel, __offset_hor ? __offset_hor : __height);
    coord.x = __roi->start.x + ((__roi->width - 1) % __offset_ver) / 2; //Center the vertical lines in the image
    coord.y = __roi->start.y;
    __point_list.push_back(coord);

    while (more_to_come) {
      if (coord.y < (__roi->image_height - next_px))
      {
        coord.y += next_px;
      }
      else
      {
        if (coord.x < (__roi->image_width - __offset_ver))
        {
          coord.x += __offset_ver;
          coord.y = __roi->start.y;
        }
        else
        {
          more_to_come = false;
        }
      }

      if (more_to_come) __point_list.push_back(coord);
    }
  }

  reset();
}

upoint_t *
ScanlineLineGrid::operator++()
{
  if (__cur != __point_list.end()) ++__cur;
  return __cur != __point_list.end() ? &*__cur : &__point_list.back();
}

upoint_t *
ScanlineLineGrid::operator++(int)
{
  if (__cur != __point_list.end()) {
    upoint_t *res = &*__cur++;
    return res;
  }
  else return &__point_list.back();
}

bool
ScanlineLineGrid::finished()
{
  return __cur == __point_list.end();
}

void
ScanlineLineGrid::reset()
{
  __cur = __point_list.begin();
}

const char *
ScanlineLineGrid::get_name()
{
  return "ScanlineModel::LineGrid";
}


unsigned int
ScanlineLineGrid::get_margin()
{
  return std::max(__offset_ver, __offset_hor);
}


void
ScanlineLineGrid::set_robot_pose(float x, float y, float ori)
{
  // ignored
}


void
ScanlineLineGrid::set_pan_tilt(float pan, float tilt)
{
  // ignored
}


/** Sets the dimensions of the grid.
 * Set width and height of scanline grid. Implicitly resets the grid.
 *
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_dimensions(unsigned int width, unsigned int height, ROI* roi)
{
  __width  = width;
  __height = height;

  set_roi(roi);
}

/** Sets the region-of-interest.
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_roi(ROI* roi)
{
  delete __roi;

  if (!roi) __roi = new ROI(0, 0, __width, __height, __width, __height);
  else
  {
    __roi = roi;
    //Use roi image width/height as grid boundary
    __roi->set_image_width(__roi->start.x + __roi->width);
    __roi->set_image_height(__roi->start.y + __roi->height);

    if (__roi->image_width > __width)
      throw fawkes::OutOfBoundsException("ScanlineLineGrid: ROI is out of grid bounds!", __roi->image_width, 0, __width);
    if (__roi->image_height > __height)
      throw fawkes::OutOfBoundsException("ScanlineLineGrid: ROI is out of grid bounds!", __roi->image_height, 0, __height);
  }

  calc_coords();
}

/** Sets offset.
 * Set horizontal and vertical offset by which the pointer in the grid is advanced.
 * This function implicitly resets the grid.
 *
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 */
void
ScanlineLineGrid::set_offset(unsigned int offset_hor, unsigned int offset_ver)
{
  __offset_hor = offset_hor;
  __offset_ver = offset_ver;

  calc_coords();
}


/** Set all grid parameters.
 * Set width, height, horizontal and vertical offset by which the pointer in the
 * grid is advanced.
 * Implicitly resets the grid.
 *
 * @param width Width of grid (most likely equal to image_width)
 * @param height Height of grid (most likely equal to image_height)
 * @param offset_hor Offset between horizontal lines (set to 0 to get only vertical lines)
 * @param offset_ver Offset between vertical lines (set to 0 to get only horizontal lines)
 * @param roi The grid will only be calculated within the roi (if NULL the grid gets
 *            calculated over the complete width/height).
 *            The provided object will be deleted by ScanlineLineGrid!
 */
void
ScanlineLineGrid::set_grid_params(unsigned int width, unsigned int height,
                            unsigned int offset_hor, unsigned int offset_ver,
                            ROI* roi)
{
  __offset_hor = offset_hor;
  __offset_ver = offset_ver;

  set_dimensions(width, height, roi);
}

} // end namespace firevision
