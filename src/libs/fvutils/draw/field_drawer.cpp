/***************************************************************************
 *  field_drawer.cpp - Drawer for a soccer field
 *
 *  Created:  Tue Sep 23 00:00:00 2008
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

#include <fvutils/draw/field_drawer.h>

#include <core/exceptions/software.h>
#include <fvutils/base/roi.h>
#include <fvutils/draw/drawer.h>
#include <fvutils/ipc/shm_image.h>

#include <cmath>
#include <cstring>
#include <stdio.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FieldDrawer <fvutils/draw/field_drawer.h>
 * This class is used to draw a soccer field.
 *
 * @author Christof Rath
 */
/** @var float FieldDrawer::_img_buffer
 * The pointer to the target image buffer
 */
/** @var float FieldDrawer::_img_width
 * The width of the target image buffer
 */
/** @var float FieldDrawer::_img_height
 * The height of the target image buffer
 */

/**
 * Created a new field object
 *
 * @param lines the field lines container
 */
FieldDrawer::FieldDrawer(const FieldLines &lines) :
  __lines(lines)
{
  __points     = NULL;
  __points_est = NULL;

  clear_own_pos();

  set_color_background(YUV_t::black());
  set_color_field(YUV_t::green());
  set_color_lines(YUV_t::white());

  set_color_own_pos(YUV_t::cyan());
  set_color_line_points(YUV_t::cyan());

  set_color_own_pos_est(YUV_t::yellow()); //yellowish
  set_color_line_points_est(YUV_t::yellow());
}

/**
 * Destructor.
 */
FieldDrawer::~FieldDrawer()
{
}


/**
 * Sets the angular offset between body and head (along the body axis)
 * @param head_yaw angular offset
 */
void
FieldDrawer::set_head_yaw(float head_yaw)
{
  __head_yaw = head_yaw;
}

/**
 * Own position setter.
 * Sets the (calculated) own position on the field
 * @param own_position as calculated by the localization
 */
void
FieldDrawer::set_own_pos(field_pos_t own_position)
{
  __own_position = own_position;
}

/**
 * Own position estimate setter.
 * Sets the position estimate (e.g. by triangulation, odometry, ...)
 * @param own_position_estimate as estimated
 */
void
FieldDrawer::set_own_pos_est(field_pos_t own_position_estimate)
{
  __own_pos_est     = own_position_estimate;
}

/**
 * Clears the own position.
 * Used (e.g.) if the own position couldn't be calculated
 */
void
FieldDrawer::clear_own_pos()
{
  __own_position.ori  = 12345;
  __own_pos_est.ori   = 12345;
  __head_yaw          = 12345;
  __points            = NULL;
  __points_est        = NULL;

  _img_buffer = NULL;
  _img_width  = 0;
  _img_height = 0;
}

/**
 * Setter for detected line points
 *
 * @param points a list of line points (relative to the center of the field!)
 */
void
FieldDrawer::set_line_points(const fld_line_points_t *points)
{
  __points = points;
}

/**
 * Setter for detected line points
 *
 * @param points_est a list of line points (relative to the center of the field!)
 */
void
FieldDrawer::set_line_points_est(const fld_line_points_t *points_est)
{
  __points_est = points_est;
}


/**
 * Calculates the conversion factor between field size and image size
 *
 * @param img_width      of the target image
 * @param img_height     of the target image
 * @param draw_landscape true if the image should be drawn landscape
 * @return the conversion factor
 */
float
FieldDrawer::get_scale(unsigned int img_width, unsigned int img_height, bool draw_landscape) const
{
  float f_width  = (draw_landscape ? __lines.get_field_length() : __lines.get_field_width());
  float f_height = (draw_landscape ? __lines.get_field_width() : __lines.get_field_length());
  return std::min(img_width / f_width, img_height / f_height);
}

/**
 * Sets the background color (outside the field)
 * @param color to be used
 */
void
FieldDrawer::set_color_background(YUV_t color)
{
  __c_background = color;
}

/**
 * Sets the field color
 * @param color to be used
 */
void
FieldDrawer::set_color_field(YUV_t color)
{
  __c_field = color;
}

/**
 * Sets the lines color
 * @param color to be used
 */
void
FieldDrawer::set_color_lines(YUV_t color)
{
  __c_lines = color;
}

/**
 * Sets the line points color
 * @param color to be used
 */
void
FieldDrawer::set_color_line_points(YUV_t color)
{
  __c_line_points = color;
}

/**
 * Sets the line points color
 * @param color to be used
 */
void
FieldDrawer::set_color_line_points_est(YUV_t color)
{
  __c_line_points_est = color;
}

/**
 * Sets the own position color
 * @param color to be used
 */
void
FieldDrawer::set_color_own_pos(YUV_t color)
{
  __c_own_pos = color;
}

/**
 * Sets the own position estimates color
 * @param color to be used
 */
void
FieldDrawer::set_color_own_pos_est(YUV_t color)
{
  __c_own_pos_est = color;
}



/**
 * Draws the field (including the own position [est]).
 * The position [est] and line points [est] gets reseted after drawing
 *
 * @param yuv422_planar the image buffer
 * @param img_width the image width
 * @param img_height the image height
 * @param draw_background true if the background (field and border) should be drawn
 * @param draw_landscape true if the field should be drawn landscape
 */
void
FieldDrawer::draw_field(unsigned char *yuv422_planar, unsigned int img_width, unsigned int img_height,
                        bool draw_background, bool draw_landscape)
{
  _img_buffer = yuv422_planar;
  _img_width  = img_width;
  _img_height = img_height;

  float f_width  = (draw_landscape ? __lines.get_field_length() : __lines.get_field_width());
  float f_height = (draw_landscape ? __lines.get_field_width() : __lines.get_field_length());
  float scale = std::min(_img_width / f_width, _img_height / f_height);

  if (draw_background) {
    unsigned int draw_width  = static_cast<unsigned int>(f_width * scale);
    unsigned int draw_height = static_cast<unsigned int>(f_height * scale);
    unsigned int u_offset = _img_width * _img_height;
    unsigned int v_offset = u_offset + u_offset / 2;

    if (_img_width == draw_width) {//use memcpy
      unsigned int offset = (_img_height - draw_height) / 2;
      memset(_img_buffer, __c_background.Y, offset * _img_width);
      memset(_img_buffer + offset * _img_width, __c_field.Y, draw_height * _img_width);
      memset(_img_buffer + (offset + draw_height) * _img_width, __c_background.Y, offset * _img_width);

      offset /= 2;
      draw_height /= 2;

      memset(_img_buffer + u_offset, __c_background.U, offset * _img_width);
      memset(_img_buffer + u_offset + offset * _img_width, __c_field.U, draw_height * _img_width);
      memset(_img_buffer + u_offset + (offset + draw_height) * _img_width, __c_background.U, offset * _img_width);

      memset(_img_buffer + v_offset, __c_background.V, offset * _img_width);
      memset(_img_buffer + v_offset + offset * _img_width, __c_field.V, draw_height * _img_width);
      memset(_img_buffer + v_offset + (offset + draw_height) * _img_width, __c_background.V, offset * _img_width);
    } else {
      //center the field
      unsigned int sx = (_img_width - draw_width) / 2;
      unsigned int sy = (_img_height - draw_height) / 2;

      ROI f_roi(sx,sy, draw_width,draw_height, _img_width,_img_height);
      for (unsigned int x = 0; x < _img_width; ++x) {
        for (unsigned int y = 0; y < _img_height; ++y) {
          if (f_roi.contains(x, y)) {
            _img_buffer[y * _img_width + x] = __c_field.Y;
            _img_buffer[(y * _img_width + x) / 2 + u_offset] = __c_field.U;
            _img_buffer[(y * _img_width + x) / 2 + v_offset] = __c_field.V;
          } else {
            _img_buffer[y * _img_width + x] = __c_background.Y;
            _img_buffer[(y * _img_width + x) / 2 + u_offset] = __c_background.U;
            _img_buffer[(y * _img_width + x) / 2 + v_offset] = __c_background.V;
          }
        }
      }
    }
  } else {
    unsigned int size = _img_width * _img_height;
    memset(_img_buffer, 0, size);
    memset(_img_buffer + size, 128, size);
  } //END: if (draw_background)


  draw_lines(__c_lines, draw_landscape, scale);

  cart_coord_2d_t f_offs = __lines.get_field_offsets();
  unsigned int center_x = std::max(0, static_cast<int>(_img_width / 2) + static_cast<int>(f_offs.x * scale));
  unsigned int center_y = std::max(0, static_cast<int>(_img_height / 2) + static_cast<int>(f_offs.y * scale));

  if (__own_pos_est.ori != 12345) {
    Drawer d;
    d.set_buffer(_img_buffer, _img_width, _img_height);
    d.set_color(__c_own_pos_est);
    unsigned int r = _img_width / 40;
    int x = static_cast<int>(__own_pos_est.x * scale);
    int y = static_cast<int>(__own_pos_est.y * scale);
    int dx = static_cast<int>(r * cosf(__own_pos_est.ori));
    int dy = static_cast<int>(r * sinf(__own_pos_est.ori));

    if (draw_landscape) {
      x += center_x;
      y = center_y - y;
      d.draw_circle(x, y, r);
      d.draw_line(x, y, x + dx, y - dy);
    } else {
      x += center_y;
      y = center_x - y;
      d.draw_circle(y, x, r);
      d.draw_line(y, x, y + dy, x - dx);
    }

    if(__head_yaw != 12345) {
      int hx = static_cast<int>(r * cosf(__own_pos_est.ori + __head_yaw));
      int hy = static_cast<int>(r * sinf(__own_pos_est.ori + __head_yaw));
      int hdx = static_cast<int>((r + 4) * cosf(__own_pos_est.ori + __head_yaw));
      int hdy = static_cast<int>((r + 4) * sinf(__own_pos_est.ori + __head_yaw));

      if (draw_landscape) d.draw_line(x + hx, y - hy, x + hdx, y - hdy);
      else d.draw_line(y + hy, x - hx, y + hdy, x - hdx);
    }
  }

  if (__own_position.ori != 12345) {
    Drawer d;
    d.set_buffer(_img_buffer, _img_width, _img_height);
    d.set_color(__c_own_pos);
    unsigned int r = _img_width / 40;
    int x   = static_cast<int>(__own_position.x * scale);
    int y   = static_cast<int>(__own_position.y * scale);
    int dx  = static_cast<int>(r * cosf(__own_position.ori));
    int dy  = static_cast<int>(r * sinf(__own_position.ori));

    if (draw_landscape) {
      x += center_x;
      y = center_y - y;
      d.draw_circle(x, y, r);
      d.draw_line(x, y, x + dx, y - dy);
    } else {
      x += center_y;
      y = center_x - y;
      d.draw_circle(y, x, r);
      d.draw_line(y, x, y + dy, x - dx);
    }

    if(__head_yaw != 12345) {
      int hx = static_cast<int>(r * cosf(__own_position.ori + __head_yaw));
      int hy = static_cast<int>(r * sinf(__own_position.ori + __head_yaw));
      int hdx = static_cast<int>((r + 4) * cosf(__own_position.ori + __head_yaw));
      int hdy = static_cast<int>((r + 4) * sinf(__own_position.ori + __head_yaw));

      if (draw_landscape) d.draw_line(x + hx, y - hy, x + hdx, y - hdy);
      else d.draw_line(y + hy, x - hx, y + hdy, x - hdx);
    }
  }

  draw_line_points(draw_landscape, scale);
  clear_own_pos();
}

/**
 * Draws the line points
 * @param draw_landscape true if the field should be drawn landscape
 * @param scale the pre calculated scale (conversion factor between image size and field size - if 0 the value gets calculated)
 */
void
FieldDrawer::draw_line_points(bool draw_landscape, float scale) const
{
  if (!scale) {
    if (draw_landscape) scale = std::min(_img_width / __lines.get_field_length(), _img_height / __lines.get_field_width());
    else scale = std::min(_img_width / __lines.get_field_width(), _img_height / __lines.get_field_length());
  }

  cart_coord_2d_t f_offs = __lines.get_field_offsets();
  unsigned int center_x = std::max(0, static_cast<int>(_img_width / 2) + static_cast<int>(f_offs.x * scale));
  unsigned int center_y = std::max(0, static_cast<int>(_img_height / 2) + static_cast<int>(f_offs.y * scale));

  Drawer d;
  d.set_buffer(_img_buffer, _img_width, _img_height);

  if (__points_est) {
    d.set_color(__c_line_points_est);
    for (fld_line_points_t::const_iterator it = __points_est->begin(); it != __points_est->end(); ++it) {
      unsigned int y = static_cast<unsigned int>(center_y - (draw_landscape ? it->y : it->x) * scale);
      unsigned int x =static_cast<unsigned int>((draw_landscape ? it->x : it->y) * scale + center_x);

      d.draw_cross(x, y, 4);
    }
  }

  if (__points) {
    d.set_color(__c_line_points);
    for (fld_line_points_t::const_iterator it = __points->begin(); it != __points->end(); ++it) {
      unsigned int y = static_cast<unsigned int>(center_y - (draw_landscape ? it->y : it->x) * scale);
      unsigned int x = static_cast<unsigned int>((draw_landscape ? it->x : it->y) * scale + center_x);

      d.draw_cross(x, y, 4);
    }
  }
}


/**
 * Draws the field lines to a SharedMemoryImageBuffer
 *
 * @param color of the lines
 * @param draw_landscape if true (default) the field is supposed to be landscape
 * @param scale the conversation factor between [m] and [px] (if 0 this value gets calculated)
 */
void
FieldDrawer::draw_lines(YUV_t color, bool draw_landscape, float scale) const
{
  if (!scale) {
    if (draw_landscape) scale = std::min(_img_width / __lines.get_field_length(), _img_height / __lines.get_field_width());
    else scale = std::min(_img_width / __lines.get_field_width(), _img_height / __lines.get_field_length());
  }

  cart_coord_2d_t f_offs = __lines.get_field_offsets();
  int f_off_x = static_cast<int>(f_offs.x * scale);
  int f_off_y = static_cast<int>(f_offs.y * scale);

  unsigned int off_x = std::max(0, static_cast<int>(_img_width / 2) + f_off_x);
  unsigned int off_y = std::max(0, static_cast<int>(_img_height / 2) + f_off_y);

  Drawer d;
  d.set_buffer(_img_buffer, _img_width, _img_height);
  d.set_color(color);

  for (FieldLines::const_iterator it = __lines.begin(); it != __lines.end(); ++it) {
    unsigned int sx = static_cast<unsigned int>((draw_landscape ? (*it).start.x : (*it).start.y) * scale);
    unsigned int sy = static_cast<unsigned int>((draw_landscape ? (*it).start.y : (*it).start.x) * scale);
    unsigned int ex = static_cast<unsigned int>((draw_landscape ? (*it).end.x : (*it).end.y) * scale);
    unsigned int ey = static_cast<unsigned int>((draw_landscape ? (*it).end.y : (*it).end.x) * scale);

    d.draw_line(off_x + sx, off_y + sy, off_x + ex, off_y + ey);
  }

  for (field_circles_t::const_iterator it = __lines.get_circles().begin(); it != __lines.get_circles().end(); ++it) {
    unsigned int cx = static_cast<unsigned int>((draw_landscape ? it->center.x : it->center.y) * scale);
    unsigned int cy = static_cast<unsigned int>((draw_landscape ? it->center.y : it->center.x) * scale);
    unsigned int r  = static_cast<unsigned int>(it->radius * scale);
    //TODO: Draw only arcs for corner circle, etc.
    d.draw_circle(off_x + cx, off_y + cy, r);
  }
}

} // end namespace firevision
