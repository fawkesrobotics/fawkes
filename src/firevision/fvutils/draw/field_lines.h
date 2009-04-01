/***************************************************************************
 *  field_lines.h - Container for field lines
 *
 *  Created:  Mon Sep 22 12:00:00 2008
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

#ifndef __NAO_UTILS_FIELD_LINES_H__
#define __NAO_UTILS_FIELD_LINES_H__

#include <utils/math/types.h>
#include <list>

typedef std::list<fawkes::arc_t> field_circles_t;

class FieldLines: public std::list<fawkes::field_line_t>
{
public:
  virtual ~FieldLines();

  float get_line_width() const;
  float get_field_length() const { return __field_length; }
  float get_field_width() const { return __field_width; }
  fawkes::cart_coord_2d_t get_field_offsets() const { return __field_offsets; }
  const field_circles_t& get_circles() const { return __field_circles; }

protected:
  FieldLines(float field_length, float field_width, float line_width);
  virtual void init() = 0;

  void calc_offsets();

  float                   __line_width;
  float                   __field_length;
  float                   __field_width;
  fawkes::cart_coord_2d_t __field_offsets;
  field_circles_t         __field_circles;
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
#endif /* __NAO_UTILS_FIELD_LINES_H__ */
