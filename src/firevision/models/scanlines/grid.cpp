
/***************************************************************************
 *  grid.cpp - Implementation of the grid scanline model
 *
 *  Generated: Tue Feb 22 10:36:39 2005
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

#include <models/scanlines/grid.h>

#include <cstring>

/** @class ScanlineGrid <models/scanlines/grid.h>
 * Scanline Grid.
 * A grid as scanline points. The crossings of the lines are the scanline
 * points.
 */

/** Constructor.
 * @param width width of grid
 * @param height height of grid
 * @param offset_x x offset between lines
 * @param offset_y y offset between lines
 */
ScanlineGrid::ScanlineGrid(unsigned int width, unsigned int height,
			   unsigned int offset_x, unsigned int offset_y)
{
  this->width = width;
  this->height = height;
  this->offset_x = offset_x;
  this->offset_y = offset_y;
  coord.x = coord.y = 0;
}

point_t
ScanlineGrid::operator*()
{
  return coord;
}

point_t*
ScanlineGrid::operator->()
{
  return &coord;
}

point_t *
ScanlineGrid::operator++()
{
  if (coord.x < (width - offset_x)) {
    coord.x += offset_x;
  } else if (coord.y < (height - offset_y)) {
    coord.x = 0;
    coord.y += offset_y;
  } else {
    // finished
  }
  return &coord;
}

point_t *
ScanlineGrid::operator++(int)
{
  memcpy(&tmp_coord, &coord, sizeof(point_t));
  if (coord.x < (width - offset_x)) {
    coord.x += offset_x;
  } else if (coord.y < (height - offset_y)) {
    coord.x = 0;
    coord.y += offset_y;
  } else {
    // finished
  }
  return &tmp_coord;
}

bool
ScanlineGrid::finished()
{
  return ( (coord.y >= (height - offset_y)) && (coord.x >= (width - offset_x)) );
}

void
ScanlineGrid::reset()
{
  coord.x = coord.y = 0;
}

const char *
ScanlineGrid::getName()
{
  return "ScanlineModel::Grid";
}


unsigned int
ScanlineGrid::getMargin()
{
  return (offset_x > offset_y) ? offset_x : offset_y;
}


void
ScanlineGrid::setRobotPose(float x, float y, float ori)
{
  // ignored
}


void
ScanlineGrid::setPanTilt(float pan, float tilt)
{
  // ignored
}


/** Set dimensions.
 * Set width and height of scanline grid. Implicitly resets the grid.
 * @param width width
 * @param height height
 */
void
ScanlineGrid::setDimensions(unsigned int width, unsigned int height)
{
  this->width  = width;
  this->height = height;
  // reset
  coord.x = coord.y = 0;
}


/** Set offset.
 * Set X and Y offset by which the pointer in the grid is advanced. Implicitly resets the grid.
 * @param offset_x offset_x
 * @param offset_y offset_y
 */
void
ScanlineGrid::setOffset(unsigned int offset_x, unsigned int offset_y)
{
  this->offset_x = offset_x;
  this->offset_y = offset_y;
  // reset
  coord.x = coord.y = 0;
}


/** Set all grid parameters.
 * Set width, height, X and Y offset by which the pointer in the grid is advanced.
 * Implicitly resets the grid.
 * @param width width
 * @param height height
 * @param offset_x offset_x
 * @param offset_y offset_y
 */
void
ScanlineGrid::setGridParams(unsigned int width, unsigned int height,
			    unsigned int offset_x, unsigned int offset_y)
{
  this->width  = width;
  this->height = height;
  this->offset_x = offset_x;
  this->offset_y = offset_y;
  // reset
  coord.x = coord.y = 0;
}
