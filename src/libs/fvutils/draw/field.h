
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

#ifndef _FVUTILS_DRAW_FIELD_H_
#define _FVUTILS_DRAW_FIELD_H_

#include <fvutils/draw/field_lines.h>

#include <list>
#include <string>

namespace firevision {

typedef std::list<fawkes::cart_coord_2d_t> fld_line_points_t;

class Field
{
public:
	~Field();

	const FieldLines &
	get_lines() const
	{
		return *lines_;
	}
	float get_field_length() const;
	float get_field_width() const;

	void print(bool in_mm) const;

	static Field *field_for_name(std::string field_name, float field_length, float field_width);

private:
	Field(FieldLines *lines, bool manage_lines_memory = true);

	FieldLines *lines_;
	bool        manage_lines_memory_;
};

} // end namespace firevision

#endif
