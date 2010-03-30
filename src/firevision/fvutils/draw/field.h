
/***************************************************************************
 *  field.h - Encapsulates a soccer field
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

#ifndef __FVUTILS_DRAW_FIELD_H_
#define __FVUTILS_DRAW_FIELD_H_

#include <fvutils/draw/field_lines.h>

#include <string>
#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef std::list<fawkes::cart_coord_2d_t> fld_line_points_t;

class Field
{
public:
  ~Field();

  const FieldLines& get_lines() const { return *__lines; }
  float get_field_length() const;
  float get_field_width() const;

  void print(bool in_mm) const;

  static Field* field_for_name(std::string field_name, float field_length, float field_width);

private:
  Field(FieldLines *lines, bool manage_lines_memory = true);

  FieldLines  *__lines;
  bool         __manage_lines_memory;
};

} // end namespace firevision

#endif
