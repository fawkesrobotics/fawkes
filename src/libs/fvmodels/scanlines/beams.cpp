
/***************************************************************************
 *  beams.cpp - Scanline model implementation: beams
 *
 *  Created: Tue Apr 17 21:09:46 2007
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

#include <core/exception.h>
#include <fvmodels/scanlines/beams.h>

#include <cmath>

using fawkes::upoint_t;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ScanlineBeams <fvmodels/scanlines/beams.h>
 * Raytraced beams scanline model.
 * This model uses a defined number of beams shot from the bottom of the image
 * towards the top using Bresenham. With this you can have kind of a radar-like
 * scanline model. Additionally the starting points at the bottom can be
 * distributed over the full width of the image which alles for a scan aligned
 * to the image.
 *
 * To ease the calculation of the finished state the very last point is traversed
 * twice.
 *
 * @author Tim Niemueller
 */

/** Construtor.
 * @param image_width image width
 * @param image_height image height
 * @param start_x x coordinate of the starting point, ignored if distributed (see below)
 * @param start_y y coordinate of the starting point, this is the lowest points of the
 * the lines and should thus be close to the bottom of the image
 * @param stop_y Y coordinate for stopping the traversal
 * @param offset_y number of pixel to advance in Y-direction per iteration
 * @param distribute_start_x set to true, to distribute the start x coordinates
 * equidistant over the whole width of the image.
 * @param angle_from angle to start the scan at, a straight vertical line means
 * zero rad, clock-wise positive, in radians
 * @param angle_range the range to use to distribute the beams, clockwise positive,
 * in radians
 * @param num_beams number of beams to use
 * @exception Exception thrown if parameters are out of bounds
 */
ScanlineBeams::ScanlineBeams(unsigned int image_width, unsigned int image_height,
           unsigned int start_x, unsigned int start_y,
           unsigned int stop_y, unsigned int offset_y,
                             bool distribute_start_x,
           float angle_from, float angle_range,
                             unsigned int num_beams)
{
  if ( start_y < stop_y )  throw fawkes::Exception("start_y < stop_y");
  if ( (stop_y > image_height) || (start_y > image_height) ) {
    throw fawkes::Exception("(stop_y > height) || (start_y > height)");
  }

  this->start_x = start_x;
  this->start_y = start_y;
  this->angle_from = angle_from;
  this->angle_range = angle_range;
  this->num_beams = num_beams;
  this->stop_y = stop_y;
  this->offset_y = offset_y;
  this->image_width = image_width;
  this->image_height = image_height;
  this->distribute_start_x = distribute_start_x;

  reset();
}


upoint_t
ScanlineBeams::operator*()
{
  return coord;
}

upoint_t*
ScanlineBeams::operator->()
{
  return &coord;
}


bool
ScanlineBeams::finished()
{
  return _finished;
}


void
ScanlineBeams::advance()
{

  while ( ! _finished && (first_beam < last_beam) ) {

    unsigned int x_start = beam_current_pos[next_beam].x;
    unsigned int y_start = beam_current_pos[next_beam].y;

    unsigned int x_end = beam_end_pos[next_beam].x;
    unsigned int y_end = beam_end_pos[next_beam].y;

    int x, y, dist, xerr, yerr, dx, dy, incx, incy;

    // calculate distance in both directions
    dx = x_end - x_start;
    dy = y_end - y_start;

    // Calculate sign of the increment
    if(dx < 0) {
      incx = -1;
      dx = -dx;
    } else {
      incx = dx ? 1 : 0;
    }

    if(dy < 0) {
      incy = -1;
      dy = -dy;
    } else {
      incy = dy ? 1 : 0;
    }

    // check which distance is larger
    dist = (dx > dy) ? dx : dy;

    // Initialize for loops
    x = x_start;
    y = y_start;
    xerr = dx;
    yerr = dy;

    /* Calculate and draw pixels */
    unsigned int offset = 0;
    while ( (x >= 0) && ((unsigned int )x < image_width) && ((unsigned int)y > stop_y) &&
      (offset < offset_y) ) {
      ++offset;

      xerr += dx;
      yerr += dy;

      if(xerr > dist) {
  xerr -= dist;
  x += incx;
      }

      if(yerr>dist) {
  yerr -= dist;
  y += incy;
      }
    }
    if ( (y < 0) || (unsigned int)y <= stop_y ) {
      _finished = true;
      break;
    }
    if ( x < 0 ) {
      first_beam = ++next_beam;
      continue;
    }
    if ( (unsigned int)x > image_width ) {
      last_beam = next_beam - 1;
      next_beam = first_beam;
      continue;
    }

    coord.x = x;
    coord.y = y;

    beam_current_pos[next_beam] = coord;

    if ( next_beam < last_beam) {
      ++next_beam;
    } else {
      next_beam = first_beam;
    }
    break;
  }

}


upoint_t *
ScanlineBeams::operator++()
{
  advance();
  return &coord;
}


upoint_t *
ScanlineBeams::operator++(int i)
{
  tmp_coord.x = coord.x;
  tmp_coord.y = coord.y;
  advance();
  return &tmp_coord;
}


void
ScanlineBeams::reset()
{
  _finished = false;

  beam_current_pos.clear();
  if ( distribute_start_x ) {
    unsigned int offset_start_x = image_width / (num_beams - 1);
    for (unsigned int i = 0; i < num_beams; ++i) {
      coord.x = i * offset_start_x;
      coord.y = start_y;
      beam_current_pos.push_back(coord);
    }
    coord.x = beam_current_pos[0].x;
    coord.y = beam_current_pos[0].y;
  } else {
    coord.x = start_x;
    coord.y = start_y;
    beam_current_pos.resize( num_beams, coord );
  }


  beam_end_pos.clear();
  next_beam = 0;
  float angle_between_beams = angle_range / num_beams;
  for (unsigned int i = 0; i < num_beams; ++i) {
    float diff_y = beam_current_pos[i].y - stop_y;
    float diff_x = diff_y * tan( angle_from + (float)i * angle_between_beams );
    upoint_t end_point;
    end_point.y = stop_y;
    end_point.x = (int)roundf(diff_x) + start_x;
    beam_end_pos.push_back(end_point);
  }
  first_beam = 0;
  last_beam = beam_end_pos.size() - 1;
}

const char *
ScanlineBeams::get_name()
{
  return "ScanlineModel::Beams";
}


unsigned int
ScanlineBeams::get_margin()
{
  return offset_y;
}

} // end namespace firevision
