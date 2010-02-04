/***************************************************************************
 *  field.cpp - Encapsulates a soccer field
 *
 *  Created:  Tue Sep 23 12:00:00 2008
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

#include <fvutils/draw/field.h>

#include <core/exceptions/software.h>

#include <cmath>
#include <cstring>
#include <stdio.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Field <fvutils/draw/field.h>
 * This class is used to describe a soccer field.
 *
 * @fn const FieldLines& Field::get_lines() const
 * Field lines getter
 * @return the field lines object
 *
 * @author Christof Rath
 */

/** Dummy constructor */
Field::Field(FieldLines *lines, bool manage_lines_memory)
{
  __lines = lines;
  __manage_lines_memory = manage_lines_memory;
}

/**
 * Destructor.
 */
Field::~Field()
{
  if (__manage_lines_memory) delete __lines;
}


/**
 * Field length getter
 * @return the length of the soccer field
 */
float
Field::get_field_length()const
{
  return __lines->get_field_length();
}


/**
 * Field width getter
 * @return the width of the soccer field
 */
float
Field::get_field_width() const
{
  return __lines->get_field_width();
}


/**
 * Prints the information to the console
 * @param in_mm if true all units that have been [m] are now [mm]
 */
void
Field::print(bool in_mm) const
{
  printf("Field lines (start-x -y end-x -y):\n==================================\n");
  for (FieldLines::const_iterator it = __lines->begin(); it != __lines->end(); ++it) {
    if (in_mm) printf("%d %d %d %d\n", static_cast<int>(it->start.x * 1000), static_cast<int>(it->start.y * 1000), static_cast<int>(it->end.x * 1000), static_cast<int>(it->end.y * 1000));
    else       printf("%0.03f %0.03f %0.03f %0.03f\n", it->start.x, it->start.y, it->end.x, it->end.y);
  }
  printf("\n");

  printf("Field circles (center-x/y radius start/end angle):\n=============================================\n");
  for (field_circles_t::const_iterator it = __lines->get_circles().begin(); it != __lines->get_circles().end(); ++it) {
    if (in_mm) printf("%d %d %d %0.03f %0.03f\n", static_cast<int>(it->center.x * 1000), static_cast<int>(it->center.y * 1000), static_cast<int>(it->radius * 1000), it->start_phi, it->end_phi);
    else       printf("%0.03f %0.03f %0.03f %0.03f %0.03f\n", it->center.x, it->center.y, it->radius, it->start_phi, it->end_phi);
  }
  printf("\n\n");
}

/**
 * Returns the corresponding Field object
 *
 * @param field_name the name of the field
 * @param field_length the area of interest around the field
 * @param field_width the area of interest around the field
 * @return the Field object pointer
 */
Field*
Field::field_for_name(std::string field_name, float field_length, float field_width)
{
  if (field_name == "Field6x4") return new Field(new FieldLines6x4(field_length, field_width));
  else if (field_name == "FieldCityTower") return new Field(new FieldLinesCityTower(field_length, field_width));
  else if (field_name == "FieldCityTowerSeminar") return new Field(new FieldLinesCityTowerSeminar(field_length, field_width));
  else throw fawkes::IllegalArgumentException("Unknown field name! Please set field_name to a valid value (see field.h)");
}

} // end namespace firevision
