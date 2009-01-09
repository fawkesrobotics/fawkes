/***************************************************************************
 *  field.h - Encapsulates a soccer field
 *
 *  Created:  23.09.2008
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

#ifndef __FIREVISION_APPS_NAO_LOC_FIELD__
#define __FIREVISION_APPS_NAO_LOC_FIELD__

#include "field_lines.h"

#include <utils/math/types.h>
#include <fvutils/color/yuv.h>

#include <list>
#include <string>

class SharedMemoryImageBuffer;

typedef std::list<fawkes::cart_coord_2d_t> fld_line_points;

class Field
{
public:
  virtual ~Field();

  FieldLines* get_lines() const;
  float get_field_length() const;
  float get_field_width() const;

  void set_own_pos(fawkes::field_pos_t own_position);
  void set_own_pos_est(fawkes::field_pos_t own_position_estimate);
  void clear_own_pos();

  virtual void set_line_points(const fld_line_points *points);
  
  void set_color_background(YUV_t color);
  void set_color_field(YUV_t color);
  void set_color_lines(YUV_t color);
  void set_color_line_points(YUV_t color);
  void set_color_own_pos(YUV_t color);
  void set_color_own_pos_est(YUV_t color);

  virtual void draw_field(SharedMemoryImageBuffer *target, bool draw_background = true, bool draw_landscape = true) const;

  virtual void print(bool in_mm) const;
  
  virtual float get_scale(unsigned int img_width, unsigned int img_height, bool draw_landscape = true) const;

  static Field* field_for_name(std::string field_name, float field_length, float field_width);

protected:
  Field(FieldLines *lines, bool destroy_on_delete = true);
  virtual void draw_line_points(SharedMemoryImageBuffer *target, bool draw_landscape = true, float scale = 0) const;
  
private: //Methods
  inline virtual void draw_cross(SharedMemoryImageBuffer *target, unsigned int x, unsigned int y, YUV_t color, unsigned int bar_length = 2) const;

private: //Members
  FieldLines           *__lines;
  fawkes::field_pos_t   __own_position, __own_pos_est;
  bool                  __destroy_on_delete;

  fld_line_points       __points;
  
  YUV_t __c_background;
  YUV_t __c_field;
  YUV_t __c_lines;
  YUV_t __c_line_points;
  YUV_t __c_own_pos;
  YUV_t __c_own_pos_est;
};

#endif /* __FIREVISION_APPS_NAO_LOC_FIELD__ */
