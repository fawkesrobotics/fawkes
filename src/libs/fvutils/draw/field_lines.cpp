/***************************************************************************
 *  field_lines.cpp - Container for field lines
 *
 *  Created:  Mon Sep 22 12:00:00 2008
 *  Copyright 2008 Christof Rath <christof.rath@gmail.com>
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

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FieldLines <fvutils/draw/field_lines.h>
 * This class acts as a container for lines on a soccer field.
 *
 * @fn void FieldLines::init()
 * Initializes the field (creates all field lines)
 *
 * @fn float FieldLines::get_field_length() const
 * Field length getter
 * @return The length of the soccer field
 *
 * @fn float FieldLines::get_field_width() const
 * Field width getter
 * @return The width of the soccer field
 *
 * @fn cart_coord_2d_t FieldLines::get_field_offsets() const
 * Offset getter.
 * The field's offset (x,y) is usually zero as the soccer field is symetrically. But in some cases
 * only a part of the field is used and then we need the offset to place the field at the center of
 * a debug image.
 * @return The offest of the field's center.
 *
 * @fn const field_circles_t& FieldLines::get_circles() const
 * Get circles.
 * @return reference to a std::list of arcs and/or circles on the field
 *
 * @author Christof Rath
 */
/** @var float FieldLines::_field_name
 * The name of the field
 */
/** @var float FieldLines::_line_width
 * The width of the field lines
 */
/** @var float FieldLines::_field_length
 * The total length of the field (actually of the field lines)
 */
/** @var float FieldLines::_field_width
 * The total width of the field (actually of the field lines)
 */
/** @var fawkes::cart_coord_2d_t FieldLines::_field_offsets
 * The center offset (used to draw unsymmetrically fields - usually zero)
 */
/** @var field_circles_t FieldLines::_field_circles
 * A std::list of arcs and/or circles on the field
 */

/**
 * Creates a new FieldLines container.
 * @param field_name   The name of the field
 * @param field_length Length of the soccer field [m]
 * @param field_width  Width of the soccer field [m]
 * @param line_width   Width of a single line [m]
 */
FieldLines::FieldLines(std::string field_name, float field_length, float field_width, float line_width):
  std::list<field_line_t>(),
  _field_name(field_name)
{
  _field_length = field_length;
  _field_width  = field_width;
  _line_width   = line_width;
  _field_offsets.x = 12345;
}

/**
 * Destructor
 */
FieldLines::~FieldLines()
{
}

/**
 * Line width getter
 * @return The width of a single field line
 */
float
FieldLines::get_line_width() const
{
  return _line_width;
}

/** Returns the field name
 * @return The field name
 */
const std::string&
FieldLines::get_name() const
{
  return _field_name;
}


/**
 * Calculates the field's offsets
 */
void
FieldLines::calc_offsets()
{
  cart_coord_2d_t mins(0, 0);
  cart_coord_2d_t maxs(0, 0);

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

  _field_offsets.x = -(mins.x + maxs.x) / 2.f;
  _field_offsets.y = -(mins.y + maxs.y) / 2.f;
}







/** @class FieldLines6x4 field_lines.h <firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
 * This class implements the 6 by 4 meter SPL field according to the 2008 roules
 *
 * @author Christof Rath
 */

/**
 * Contructor.
 * @param length of the soccer field
 * @param width of the soccer field
 */
FieldLines6x4::FieldLines6x4(float length, float width):
  FieldLines("FieldLines6x4", length, width, 0.05f)
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
  push_back(field_line_t(3.f, 2.f, 3.f, -2.f));
  //opponent hor penalty area line
  push_back(field_line_t(2.4f, 1.5f, 2.4f, -1.5f));
  //opponent vert penalty area lines
  push_back(field_line_t(3.f,  1.5f, 2.4f,  1.5f));
  push_back(field_line_t(3.f, -1.5f, 2.4f, -1.5f));

  //opponent penalty point
  push_back(field_line_t(1.2f,  0.05f, 1.2f, -0.05f));
  push_back(field_line_t(1.15f, 0.f,   1.25f, 0.f));

  //center line
  push_back(field_line_t(0.f, 2.f, 0.f, -2.f));
  //side lines
  push_back(field_line_t(3.f,  2.f, -3.f,  2.f));
  push_back(field_line_t(3.f, -2.f, -3.f, -2.f));

  //center circle (approximated by 12 lines from )
  _field_circles.push_back(fawkes::arc_t(0.6f, 0.f, 0.f));

  //own goal line (corner to corner)
  push_back(field_line_t(-3.f, 2.f, -3.f, -2.f));
  //own hor penalty area line
  push_back(field_line_t(-2.4f, 1.5f, -2.4f, -1.5f));
  //own vert penalty area lines
  push_back(field_line_t(-3.f,  1.5f, -2.4f,  1.5f));
  push_back(field_line_t(-3.f, -1.5f, -2.4f, -1.5f));

  //own penalty point
  push_back(field_line_t(-1.2f,  0.05f, -1.2f, -0.05f));
  push_back(field_line_t(-1.15f, 0.f,   -1.25f, 0.f));
}








/** @class FieldLinesCityTower field_lines.h <firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
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
FieldLinesCityTower::FieldLinesCityTower(float length, float width):
  FieldLines("FieldLinesCityTower", length, width, 0.09f)
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
  _field_circles.push_back(fawkes::arc_t(1.1f, 0.f, 0.f));

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









/** @class FieldLinesCityTowerSeminar field_lines.h <firevision/apps/nao_loc/field_lines.cpp/field_lines.h>
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
FieldLinesCityTowerSeminar::FieldLinesCityTowerSeminar(float length, float width):
  FieldLines("FieldLinesCityTowerSeminar", length, width, 0.05f)
{
  init();
  calc_offsets();
}

FieldLinesCityTowerSeminar::~FieldLinesCityTowerSeminar()
{
}

void
FieldLinesCityTowerSeminar::init()
{
  //opponent goal line (corner to corner)
  push_back(field_line_t(2.725f, 1.825f, 2.725f, -1.825f));
  //opponent hor penalty area line
  push_back(field_line_t(2.125f, 1.5f, 2.125f, -1.5f));
  //opponent vert penalty area lines
  push_back(field_line_t(2.725f,  1.5f, 2.125f,  1.5f));
  push_back(field_line_t(2.725f, -1.5f, 2.125f, -1.5f));

  //opponent penalty point
  push_back(field_line_t(0.925f, 0.05f, 0.925f, -0.05f));
  push_back(field_line_t(0.875f, 0.f,   0.975f,  0.f));

  //center line
  push_back(field_line_t(0.f, 1.825f, 0.f, -1.825f));
  //side lines
  push_back(field_line_t(2.725f,  1.825f, -2.725f,  1.825f));
  push_back(field_line_t(2.725f, -1.825f, -2.725f, -1.825f));

  //center circle (approximated by 12 lines from )
  _field_circles.push_back(fawkes::arc_t(0.57f, 0.f, 0.f));


  //own goal line (corner to corner)
  push_back(field_line_t(-2.725f, 1.825f, -2.725f, -1.825f));
  //own hor penalty area line
  push_back(field_line_t(-2.125f, 1.5f, -2.125f, -1.5f));
  //own vert penalty area lines
  push_back(field_line_t(-2.725f,  1.5f, -2.125f,  1.5f));
  push_back(field_line_t(-2.725f, -1.5f, -2.125f, -1.5f));

  //own penalty point
  push_back(field_line_t(-0.925f, 0.05f, -0.925f, -0.05f));
  push_back(field_line_t(-0.875f, 0.f,   -0.975f,  0.f));
}


} // end namespace firevision
