/***************************************************************************
 *  field_lines.cpp - Container for field lines
 *
 *  Created:  22.09.2008
 *  Copyright 2008 Christof Rath <christof.rath@gmail.com>
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "field_lines.h"
#include <fvutils/draw/drawer.h>
#include <core/exceptions/software.h>

#include <cmath>

using fawkes::cart_coord_2d_t;
using fawkes::field_line_t;
using std::min;
using std::max;

/** @class FieldLines field_lines.h </nao_fawkes/src/firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
 * This class acts as a container for lines on a soccer field.
 *
 * @fn void FieldLines::init()
 * Initializes the field (creates all field lines)
 *
 * @author Christof Rath
 */

/**
 * Creates a new FieldLines container.
 *
 * @param field_length Length of the soccer field [m]
 * @param field_width  Width of the soccer field [m]
 * @param line_width   Width of a single line [m]
 */
FieldLines::FieldLines(float field_length, float field_width, float line_width): std::list<field_line_t>()
{
  __field_length = field_length;
  __field_width  = field_width;
  __line_width   = line_width;
  __field_offsets.x = 12345;
}

/**
 * Destructor
 */
FieldLines::~FieldLines()
{
}

/**
 * Field length getter
 * @return The length of the soccer field
 */
float
FieldLines::get_field_length()
{
  return __field_length;
}

/**
 * Field width getter
 * @return The width of the soccer field
 */
float
FieldLines::get_field_width()
{
  return __field_width;
}

/**
 * Line width getter
 * @return The width of a single field line
 */
float
FieldLines::get_line_width()
{
  return __line_width;
}

/**
 * Offset getter.
 * The field's offset (x,y) is usually zero as the soccer field is symetrically. But in some cases
 * only a part of the field is used and then we need the offset to place the field at the center of
 * a debug image.
 * @return The offest of the field's center.
 */
cart_coord_2d_t
FieldLines::get_field_offsets()
{
  if (__field_offsets.x == 12345) throw fawkes::IllegalArgumentException("Field offsets not calculated! Call FieldLines::calc_offsets() at the end of init()");

  return __field_offsets;
}

/**
 * Adds the piecewise approximation of a circle.
 * @param r radius of the circle
 * @param pieces number of pieces
 * @param center_x center x of the circle
 * @param center_y center y of the circle
 * @param theta_start used if only a part of a circle should be drawn
 * @param theta_end used if only a part of a circle should be drawn
 */
void
FieldLines::add_circle(float r, unsigned int pieces, float center_x, float center_y, float theta_start, float theta_end)
{
  if (!theta_end) theta_end = 2 * M_PI;

  float theta_diff = theta_end - theta_start;
  float theta_piece = theta_diff / pieces;

  cart_coord_2d_t initial = { 12345, 12345 };
  cart_coord_2d_t start   = { 0, 0 };
  cart_coord_2d_t end     = { 0, 0 };

  unsigned int i = 0;
  for (float t = theta_start; i < pieces; t += theta_piece, ++i) {
    if (initial.x == 12345) {
      initial.x = r * cosf(t) + center_x;
      initial.y = r * sinf(t) + center_y;

      start = initial;
      continue;
    }

    end.x = r * cosf(t) + center_x;
    end.y = r * sinf(t) + center_y;

    push_back(field_line_t(start, end));
    start = end;
  }

  push_back(field_line_t(start, initial));
}

/**
 * Calculates the field's offsets
 */
void
FieldLines::calc_offsets()
{
  cart_coord_2d_t mins = { 0, 0 };
  cart_coord_2d_t maxs = { 0, 0 };

  float f;

  for (FieldLines::iterator it = begin(); it != end(); ++it) {
    //x-Axis
    f = min(it->start.x, it->end.x);
    if (f < mins.x) mins.x = f;
    f = max(it->start.x, it->end.x);
    if (f > maxs.x) maxs.x = f;

    //y-Axis
    f = min(it->start.y, it->end.y);
    if (f < mins.y) mins.y = f;
    f = max(it->start.y, it->end.y);
    if (f > maxs.y) maxs.y = f;
  }

  __field_offsets.x = -(mins.x + maxs.x) / 2.f;
  __field_offsets.y = -(mins.y + maxs.y) / 2.f;
}

/**
 * Draws the field lines to a SharedMemoryImageBuffer
 *
 * @param target the SharedMemoryImageBuffer
 * @param color of the lines
 * @param draw_landscape if true (default) the field is supposed to be landscape
 * @param scale the conversation factor between [m] and [px] (if 0 this value gets calculated)
 */
void
FieldLines::draw_lines(SharedMemoryImageBuffer *target, YUV_t color, bool draw_landscape, float scale)
{
  if (!scale) {
    if (draw_landscape) scale = std::min(target->width() / __field_length, target->height() / __field_width);
    else scale = std::min(target->width() / __field_width, target->height() / __field_length);
  }

  cart_coord_2d_t f_offs = get_field_offsets();
  int f_off_x = static_cast<int>(f_offs.x * scale);
  int f_off_y = static_cast<int>(f_offs.y * scale);

  unsigned int off_x = std::max(0, static_cast<int>(target->width() / 2) + f_off_x);
  unsigned int off_y = std::max(0, static_cast<int>(target->height() / 2) + f_off_y);

  Drawer d;
  d.set_buffer(target->buffer(), target->width(), target->height());
  d.set_color(color);

  for (FieldLines::iterator it = begin(); it != end(); ++it) {
    unsigned int sx = static_cast<unsigned int>((draw_landscape ? (*it).start.x : (*it).start.y) * scale);
    unsigned int sy = static_cast<unsigned int>((draw_landscape ? (*it).start.y : (*it).start.x) * scale);
    unsigned int ex = static_cast<unsigned int>((draw_landscape ? (*it).end.x : (*it).end.y) * scale);
    unsigned int ey = static_cast<unsigned int>((draw_landscape ? (*it).end.y : (*it).end.x) * scale);

    d.draw_line(off_x + sx, off_y + sy, off_x + ex, off_y + ey);
  }
}







/** @class FieldLines6x4 field_lines.h </nao_fawkes/src/firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
 * This class implements the 6 by 4 meter SPL field according to the 2008 roules
 *
 * @author Christof Rath
 */

/**
 * Contructor.
 * @param length of the soccer field
 * @param width of the soccer field
 */
FieldLines6x4::FieldLines6x4(float length, float width): FieldLines(length, width, 0.05f)
{
  init();
  calc_offsets();
}

FieldLines6x4::~FieldLines6x4()
{
}

void
FieldLines6x4::init()
{
  //opponent goal line (corner to corner)
  push_back(field_line_t(2.975f, 1.975f, 2.975f, -1.975f));
  //opponent hor penalty area line
  push_back(field_line_t(2.425f, 1.f, 2.425f, -1.f));
  //opponent vert penalty area lines
  push_back(field_line_t(2.975f, 1.f, 2.425f, 1.f));
  push_back(field_line_t(2.975f, -1.f, 2.425f, -1.f));

  //center line
  push_back(field_line_t(0.f, 1.957f, 0.f, -1.975f));
  //side lines
  push_back(field_line_t(2.975f, 1.975f, -2.975f, 1.975f));
  push_back(field_line_t(2.975f, -1.975f, -2.975f, -1.975f));

  //center circle (approximated by 8 lines from )
  add_circle(0.65, 12);

  //own goal line (corner to corner)
  push_back(field_line_t(-2.975f, 1.975f, -2.975f, -1.975f));
  //own hor penalty area line
  push_back(field_line_t(-2.425f, 0.975f, -2.425f, -0.975f));
  //opponent vert penalty area lines
  push_back(field_line_t(-2.975f, 0.975f, -2.425f, 0.975f));
  push_back(field_line_t(-2.975f, -0.975f, -2.425f, -0.975f));
}








/** @class FieldLinesCityTower field_lines.h </nao_fawkes/src/firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
 * This class implements the test field in Graz, Austria at the CityTower.
 * The field is not symmetrical!
 *
 * @author Christof Rath
 */

/**
 * Constructor.
 * @param length of the soccer field
 * @param width of the soccer field
 */
FieldLinesCityTower::FieldLinesCityTower(float length, float width): FieldLines(length, width, 0.09f)
{
  init();
  calc_offsets();
}

FieldLinesCityTower::~FieldLinesCityTower()
{
}

void
FieldLinesCityTower::init()
{
  //opponent goal line (corner to corner)
  push_back(field_line_t(4.97f, 2.455f, 4.97f, -2.455f));
  //opponent hor penalty area line
  push_back(field_line_t(3.82f, 1.49f, 3.82f, -1.49f));
  //opponent vert penalty area lines
  push_back(field_line_t(4.97f,  1.49f, 3.82f,  1.49f));
  push_back(field_line_t(4.97f, -1.49f, 3.82f, -1.49f));

  //center line
  push_back(field_line_t(0.f, 2.455f, 0.f, -2.455f));
  //side lines
  push_back(field_line_t(4.97f,  2.455f, -1.44f,  2.455f));
  push_back(field_line_t(4.97f, -2.455f, -1.44f, -2.455f));

  //center circle (approximated by 12 lines from )
  add_circle(1.1, 12);

/* Not Available...
  //own goal line (corner to corner)
  push_back(field_line_t(-2.975f, 1.975f, -2.975f, -1.975f));
  //own hor penalty area line
  push_back(field_line_t(-2.425f, 0.975f, -2.425f, -0.975f));
  //opponent vert penalty area lines
  push_back(field_line_t(-2.975f, 0.975f, -2.425f, 0.975f));
  push_back(field_line_t(-2.975f, -0.975f, -2.425f, -0.975f));
*/
}

