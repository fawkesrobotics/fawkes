/***************************************************************************
 *  field_lines.h - Container for field lines
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

#ifndef __FVUTILS_DRAW_FIELD_LINES_H__
#define __FVUTILS_DRAW_FIELD_LINES_H__

#include <utils/math/types.h>
#include <list>
#include <string>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

typedef std::list<fawkes::arc_t> field_circles_t;

class FieldLines: public std::list<fawkes::field_line_t>
{
public:
  virtual ~FieldLines();

  float get_line_width() const;
  float get_field_length() const { return _field_length; }
  float get_field_width() const { return _field_width; }
  fawkes::cart_coord_2d_t get_field_offsets() const { return _field_offsets; }
  const field_circles_t& get_circles() const { return _field_circles; }
  const std::string& get_name() const;

protected:
  FieldLines(std::string field_name, float field_length, float field_width, float line_width);
  virtual void init() = 0;

  void calc_offsets();

  std::string             _field_name;
  float                   _line_width;
  float                   _field_length;
  float                   _field_width;
  fawkes::cart_coord_2d_t _field_offsets;
  field_circles_t         _field_circles;
};

class FieldLines6x4: public FieldLines
{
public:
  FieldLines6x4(float length, float width);
  virtual ~FieldLines6x4();

private:
  virtual void init();
};

class FieldLinesCityTower: public FieldLines
{
public:
  FieldLinesCityTower(float length, float width);
  virtual ~FieldLinesCityTower();

private:
  virtual void init();
};

class FieldLinesCityTowerSeminar: public FieldLines
{
public:
  FieldLinesCityTowerSeminar(float length, float width);
  virtual ~FieldLinesCityTowerSeminar();

private:
  virtual void init();
};

} // end namespace firevision

#endif
