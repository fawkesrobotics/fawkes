
/***************************************************************************
 *  line_grid.cpp - Implementation of the line grid scanline model
 *
 *  Generated: Wed Mar 25 17:31:00 2009
 *  Copyright  2009 Christof Rath <c.rath@student.tugraz.at>
 *
 *  $Id$
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

#include "line_grid.h"

#include <fvutils/base/roi.h>
#include <fvutils/draw/drawer.h>
#include <core/exceptions/software.h>

#include <cstring>

using fawkes::point_t;

/** @class ScanlineLineGrid line_grid.h <models/scanlines/line_grid.h>
 * Grid of scan lines.
 * A grid of scan lines (i.e. horizontal and/or vertical lines) instead of only
 * points on the grid crossings.
 */

/** Constructor.
 * @param width width of grid
 * @param height height of grid
 * @param offset_x x offset between lines (set to 0 to get only vertical lines)
 * @param offset_y y offset between lines (set to 0 to get only horizontal lines)
 * @param roi the grid will only be calculated within the roi (if NULL the roi
 *            will be from offset_x/2,offset_y/2 to width-offset_x/2,height-offset_y/2).
 *            The provided object will be deleted by ScanlineLineGrid!
 * @param gap between the points on the line
 */
ScanlineLineGrid::ScanlineLineGrid(unsigned int width, unsigned int height,
			   unsigned int offset_x, unsigned int offset_y,
			   ROI* roi, unsigned int gap)
{
  __roi = NULL;
  __next_pixel = gap + 1;
  setGridParams(width, height,
                offset_x, offset_y, roi);
  //reset is done in setGridParams ()
}

/** Destructor
 */
ScanlineLineGrid::~ScanlineLineGrid()
{
  delete __roi;
}

point_t
ScanlineLineGrid::operator*()
{
  return *__cur;
}

point_t*
ScanlineLineGrid::operator->()
{
  return &*__cur;
}

void
ScanlineLineGrid::calc_coords()
{
  __point_list.clear();
  bool more_to_come = true;
  point_t coord;
  unsigned int next_px;

  if (__offset_y > 0) //horizontal lines
  {
    more_to_come = true;
    next_px = std::min(__next_pixel, __offset_x ? __offset_x : __width);
    coord.x = __roi->start.x;
    coord.y = __roi->start.y;
    __point_list.push_back(coord);

    while (more_to_come) {
      if (coord.x < (__roi->image_width - __next_pixel))
      {
        coord.x += next_px;
      }
      else
      {
        if (coord.y < (__roi->image_height - __offset_y))
        {
          coord.x = __roi->start.x;
          coord.y += __offset_y;
        }
        else
        {
          more_to_come = false;
        }
      }

      if (more_to_come) __point_list.push_back(coord);
    }
  }

  if (__offset_x > 0) //vertical lines
  {
    more_to_come = true;
    next_px = std::min(__next_pixel, __offset_y ? __offset_y : __height);
    coord.x = __roi->start.x;
    coord.y = __roi->start.y;
    __point_list.push_back(coord);

    while (more_to_come) {
      if (coord.y < (__roi->image_height - __next_pixel))
      {
        coord.y += next_px;
      }
      else
      {
        if (coord.x < (__roi->image_width - __offset_x))
        {
          coord.x += __offset_x;
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

point_t *
ScanlineLineGrid::operator++()
{
  if (__cur != __point_list.end()) ++__cur;
  return __cur != __point_list.end() ? &*__cur : &__point_list.back();
}

point_t *
ScanlineLineGrid::operator++(int)
{
  if (__cur != __point_list.end()) {
    point_t *res = &*__cur++;
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
  return std::max(__offset_x, __offset_y);
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


/** Set dimensions.
 * Set width and height of scanline grid. Implicitly resets the grid.
 * @param width width
 * @param height height
 * @param roi the grid will only be calculated within the roi (if NULL the roi
 *            will be from 0,0 to width,height). The object will be deleted by
 *            ScanlineLineGrid!
 */
void
ScanlineLineGrid::setDimensions(unsigned int width, unsigned int height, ROI* roi)
{
  __width  = width;
  __height = height;

  delete __roi;

  if (!roi) __roi = new ROI(__offset_x / 2, __offset_y / 2, __width - __offset_x / 2, __height - __offset_y / 2, __width, __height);
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


/** Set offset.
 * Set X and Y offset by which the pointer in the grid is advanced. Implicitly resets the grid.
 * @param offset_x offset_x
 * @param offset_y offset_y
 */
void
ScanlineLineGrid::setOffset(unsigned int offset_x, unsigned int offset_y)
{
  __offset_x = offset_x;
  __offset_y = offset_y;

  calc_coords();
}


/** Set all grid parameters.
 * Set width, height, X and Y offset by which the pointer in the grid is advanced.
 * Implicitly resets the grid.
 * @param width width
 * @param height height
 * @param offset_x offset_x
 * @param offset_y offset_y
 * @param roi the grid will only be calculated within the roi (if NULL the roi
 *            will be from 0,0 to width,height). The object will be deleted by
 *            ScanlineLineGrid!
 */
void
ScanlineLineGrid::setGridParams(unsigned int width, unsigned int height,
                            unsigned int offset_x, unsigned int offset_y,
                            ROI* roi)
{
  __offset_x = offset_x;
  __offset_y = offset_y;

  setDimensions(width, height, roi);
}

/** Draws the grid into a buffer
 * @param yuv422_planar the pointer to the image buffer
 * @param color the color to use
 */
void
ScanlineLineGrid::draw_grid(unsigned char *yuv422_planar, YUV_t color)
{
  Drawer d;
  d.set_buffer(yuv422_planar, __width, __height);
  d.set_color(color);

  for(point_list_t::const_iterator it = __point_list.begin(); it != __point_list.end(); ++it) {
    d.color_point(it->x, it->y);
  }
}

