/***************************************************************************
 *  field_lines.h - Container for field lines
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

#ifndef __FIREVISION_APPS_NAO_LOC_FIELD_LINES__
#define __FIREVISION_APPS_NAO_LOC_FIELD_LINES__

#include <utils/math/types.h>
#include <list>
#include <fvutils/color/yuv.h>
#include <fvutils/ipc/shm_image.h>


class FieldLines: public std::list<fawkes::field_line_t>
{
public:
  virtual ~FieldLines();

  float get_line_width();
  float get_field_length();
  float get_field_width();
  fawkes::cart_coord_2d_t get_field_offsets();

  void draw_lines(SharedMemoryImageBuffer *target, YUV_t color, bool draw_landscape = true, float scale = 0);

protected:
  FieldLines(float field_length, float field_width, float line_width);

  void calc_offsets();
  void add_circle(float r, unsigned int pieces = 8, float center_x = 0, float center_y = 0, float theta_start = 0, float theta_end = 0);
  virtual void init() = 0;

private: //fields
  float __line_width;
  float __field_length;
  float __field_width;
  fawkes::cart_coord_2d_t __field_offsets;
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
#endif /* __FIREVISION_APPS_NAO_LOC_FIELD_LINES__ */
