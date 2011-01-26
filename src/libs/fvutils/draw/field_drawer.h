/***************************************************************************
 *  field_drawer.h - Encapsulates a soccer field
 *
 *  Created:  Tue Sep 23 00:00:00 2009
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

#ifndef __FVUTILS_DRAWER_FIELD_DRAWER_H__
#define __FVUTILS_DRAWER_FIELD_DRAWER_H__

#include <fvutils/draw/field.h>

#include <utils/math/types.h>
#include <fvutils/color/yuv.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


class SharedMemoryImageBuffer;

class FieldDrawer
{
public:
  FieldDrawer(const FieldLines &lines);
  virtual ~FieldDrawer();

  void set_head_yaw(float head_yaw);
  void set_own_pos(fawkes::field_pos_t own_position);
  void set_own_pos_est(fawkes::field_pos_t own_position_estimate);

  void set_line_points(const fld_line_points_t *points);
  void set_line_points_est(const fld_line_points_t *points_est);

  void set_color_background(YUV_t color);
  void set_color_field(YUV_t color);
  void set_color_lines(YUV_t color);
  void set_color_line_points(YUV_t color);
  void set_color_line_points_est(YUV_t color);
  void set_color_own_pos(YUV_t color);
  void set_color_own_pos_est(YUV_t color);

  virtual void draw_field(unsigned char *yuv422_planar, unsigned int img_width, unsigned int img_height,
                          bool draw_background = true, bool draw_landscape = true);

protected:
  inline void clear_own_pos();
  inline float get_scale(unsigned int img_width, unsigned int img_height, bool draw_landscape = true) const;
  virtual void draw_line_points(bool draw_landscape = true, float scale = 0) const;
  virtual void draw_lines(YUV_t color, bool draw_landscape = true, float scale = 0) const;

  unsigned char *_img_buffer;
  unsigned int   _img_width;
  unsigned int   _img_height;

private: //Members
  const FieldLines     &__lines;
  fawkes::field_pos_t   __own_position, __own_pos_est;
  float                 __head_yaw;

  const fld_line_points_t     *__points;
  const fld_line_points_t     *__points_est;

  YUV_t __c_background;
  YUV_t __c_field;
  YUV_t __c_lines;
  YUV_t __c_line_points;
  YUV_t __c_line_points_est;
  YUV_t __c_own_pos;
  YUV_t __c_own_pos_est;
};

} // end namespace firevision

#endif
