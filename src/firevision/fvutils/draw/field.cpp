/***************************************************************************
 *  field.cpp - Encapsulates a soccer field
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

#include "field.h"

#include <core/exceptions/software.h>
#include <fvutils/base/roi.h>
#include <fvutils/draw/drawer.h>

#include <cmath>
#include <cstring>
#include <stdio.h>

using namespace fawkes;

/** @class Field field.h </nao_fawkes/src/firevision/apps/nao_loc/field.cpp/field.h>
 * This class is used to describe a soccer field.
 *
 * @author Christof Rath
 */

/**
 * Created a new field object
 *
 * @param lines the field lines container
 * @param destroy_on_delete if true the field lines container gets destroid with the object
 */
Field::Field(FieldLines *lines, bool destroy_on_delete)
{
  if (!lines) throw fawkes::NullPointerException("lines may not be null!");

  __lines             = lines;
  __destroy_on_delete = destroy_on_delete;

  clear_own_pos();

  set_color_background(YUV_t::black());
  set_color_field(YUV_t::green());
  set_color_lines(YUV_t::white());

  set_color_own_pos(YUV_t::cyan());
  set_color_line_points(YUV_t::cyan());

  set_color_own_pos_est(YUV_t(180, 128, 0)); //yellowish
  set_color_line_points_est(YUV_t(180, 128, 0));
}

/**
 * Destructor.
 */
Field::~Field()
{
  if (__destroy_on_delete) delete __lines;
}


/**
 * Field lines getter
 * @return the field lines object
 */
FieldLines*
Field::get_lines() const
{
  return __lines;
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
 * Sets the angular offset between body and head (along the body axis)
 * @param head_yaw angular offset
 */
void
Field::set_head_yaw(float head_yaw)
{
  __head_yaw = head_yaw;
}

/**
 * Own position setter.
 * Sets the (calculated) own position on the field
 * @param own_position as calculated by the localization
 */
void
Field::set_own_pos(field_pos_t own_position)
{
  __own_position = own_position;
}

/**
 * Own position estimate setter.
 * Sets the position estimate (e.g. by triangulation, odometry, ...)
 * @param own_position_estimate as estimated
 */
void
Field::set_own_pos_est(field_pos_t own_position_estimate)
{
  __own_pos_est     = own_position_estimate;
}

/**
 * Clears the own position.
 * Used (e.g.) if the own position couldn't be calculated
 */
void
Field::clear_own_pos()
{
  __own_position.ori  = 12345;
  __own_pos_est.ori   = 12345;
  set_head_yaw(12345);
}

/**
 * Setter for detected line points
 *
 * @param points a list of line points (relative to the center of the field!)
 */
void
Field::set_line_points(const fld_line_points *points)
{
  __points = fld_line_points(*points);
}

/**
 * Setter for detected line points
 *
 * @param points_est a list of line points (relative to the center of the field!)
 */
void
Field::set_line_points_est(const fld_line_points *points_est)
{
  __points_est = fld_line_points(*points_est);
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
Field::get_scale(unsigned int img_width, unsigned int img_height, bool draw_landscape) const
{
  float f_width  = (draw_landscape ? get_field_length() : get_field_width());
  float f_height = (draw_landscape ? get_field_width() : get_field_length());
  return std::min(img_width / f_width, img_height / f_height);

}

/**
 * Sets the background color (outside the field)
 * @param color to be used
 */
void
Field::set_color_background(YUV_t color)
{
  __c_background = color;
}

/**
 * Sets the field color
 * @param color to be used
 */
void
Field::set_color_field(YUV_t color)
{
  __c_field = color;
}

/**
 * Sets the lines color
 * @param color to be used
 */
void
Field::set_color_lines(YUV_t color)
{
  __c_lines = color;
}

/**
 * Sets the line points color
 * @param color to be used
 */
void
Field::set_color_line_points(YUV_t color)
{
  __c_line_points = color;
}

/**
 * Sets the line points color
 * @param color to be used
 */
void
Field::set_color_line_points_est(YUV_t color)
{
  __c_line_points_est = color;
}

/**
 * Sets the own position color
 * @param color to be used
 */
void
Field::set_color_own_pos(YUV_t color)
{
  __c_own_pos = color;
}

/**
 * Sets the own position estimates color
 * @param color to be used
 */
void
Field::set_color_own_pos_est(YUV_t color)
{
  __c_own_pos_est = color;
}



/**
 * Draws the field (including the own position [est])
 *
 * @param target the image buffer
 * @param draw_background true if the background (field and border) should be drawn
 * @param draw_landscape true if the field should be drawn landscape
 */
void
Field::draw_field(SharedMemoryImageBuffer *target, bool draw_background, bool draw_landscape) const
{
  float f_width  = (draw_landscape ? get_field_length() : get_field_width());
  float f_height = (draw_landscape ? get_field_width() : get_field_length());
  float scale = std::min(target->width() / f_width, target->height() / f_height);

  if (draw_background) {
    unsigned int draw_width  = static_cast<unsigned int>(f_width * scale);
    unsigned int draw_height = static_cast<unsigned int>(f_height * scale);
    unsigned int u_offset = target->width() * target->height();
    unsigned int v_offset = u_offset + u_offset / 2;

    if (target->width() == draw_width) {//use memcpy
      unsigned int offset = (target->height() - draw_height) / 2;
      memset(target->buffer(), __c_background.Y, offset * target->width());
      memset(target->buffer() + offset * target->width(), __c_field.Y, draw_height * target->width());
      memset(target->buffer() + (offset + draw_height) * target->width(), __c_background.Y, offset * target->width());

      offset /= 2;
      draw_height /= 2;

      memset(target->buffer() + u_offset, __c_background.U, offset * target->width());
      memset(target->buffer() + u_offset + offset * target->width(), __c_field.U, draw_height * target->width());
      memset(target->buffer() + u_offset + (offset + draw_height) * target->width(), __c_background.U, offset * target->width());

      memset(target->buffer() + v_offset, __c_background.V, offset * target->width());
      memset(target->buffer() + v_offset + offset * target->width(), __c_field.V, draw_height * target->width());
      memset(target->buffer() + v_offset + (offset + draw_height) * target->width(), __c_background.V, offset * target->width());
    } else {
      //center the field
      unsigned int sx = (target->width() - draw_width) / 2;
      unsigned int sy = (target->height() - draw_height) / 2;

      ROI f_roi(sx,sy, draw_width,draw_height, target->width(),target->height());
      for (unsigned int x = 0; x < target->width(); ++x) {
        for (unsigned int y = 0; y < target->height(); ++y) {
          if (f_roi.contains(x, y)) {
            target->buffer()[y * target->width() + x] = __c_field.Y;
            target->buffer()[(y * target->width() + x) / 2 + u_offset] = __c_field.U;
            target->buffer()[(y * target->width() + x) / 2 + v_offset] = __c_field.V;
          } else {
            target->buffer()[y * target->width() + x] = __c_background.Y;
            target->buffer()[(y * target->width() + x) / 2 + u_offset] = __c_background.U;
            target->buffer()[(y * target->width() + x) / 2 + v_offset] = __c_background.V;
          }
        }
      }
    }
  } else {
    unsigned int size = target->width() * target->height();
    memset(target->buffer(), 0, size);
    memset(target->buffer() + size, 128, size);
  } //END: if (draw_background)


  __lines->draw_lines(target, __c_lines, draw_landscape, scale);

  cart_coord_2d_t f_offs = __lines->get_field_offsets();
  unsigned int center_x = std::max(0, static_cast<int>(target->width() / 2) + static_cast<int>(f_offs.x * scale));
  unsigned int center_y = std::max(0, static_cast<int>(target->height() / 2) + static_cast<int>(f_offs.y * scale));

  if (__own_position.ori != 12345) {
    Drawer d;
    d.set_buffer(target->buffer(), target->width(), target->height());
    d.set_color(__c_own_pos);
    unsigned int r = target->width() / 40;
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

  if (__own_pos_est.ori != 12345) {
    Drawer d;
    d.set_buffer(target->buffer(), target->width(), target->height());
    d.set_color(__c_own_pos_est);
    unsigned int r = target->width() / 40;
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

  draw_line_points(target, draw_landscape, scale);
}

/**
 * Draws the line points
 * @param target the image buffer
 * @param draw_landscape true if the field should be drawn landscape
 * @param scale the pre calculated scale (conversion factor between image size and field size - if 0 the value gets calculated)
 */
void
Field::draw_line_points(SharedMemoryImageBuffer *target, bool draw_landscape, float scale) const
{
  if (!scale) scale = std::min(target->width() / get_field_length(), target->height() / get_field_width());

  cart_coord_2d_t f_offs = __lines->get_field_offsets();
  unsigned int center_x = std::max(0, static_cast<int>(target->width() / 2) + static_cast<int>(f_offs.x * scale));
  unsigned int center_y = std::max(0, static_cast<int>(target->height() / 2) + static_cast<int>(f_offs.y * scale));

  Drawer d;
  d.set_buffer(target->buffer(), target->width(), target->height());

  d.set_color(__c_line_points);
  for (fld_line_points::const_iterator it = __points.begin(); it != __points.end(); ++it)
  {
    unsigned int y = static_cast<unsigned int>(center_y - (draw_landscape ? it->y : it->x) * scale);
    unsigned int x =static_cast<unsigned int>((draw_landscape ? it->x : it->y) * scale + center_x);

    d.draw_cross(x, y, 4);
  }

  d.set_color(__c_line_points_est);
  for (fld_line_points::const_iterator it = __points_est.begin(); it != __points_est.end(); ++it)
  {
    unsigned int y = static_cast<unsigned int>(center_y - (draw_landscape ? it->y : it->x) * scale);
    unsigned int x =static_cast<unsigned int>((draw_landscape ? it->x : it->y) * scale + center_x);

    d.draw_cross(x, y, 4);
  }
}

/**
 * Prints the information to the console
 * @param in_mm if true all units that have been [m] are now [mm]
 */
void
Field::print(bool in_mm) const
{
  printf("Position estimate (x y ori): ");
  if (in_mm) printf("%d %d %0.03f\n\n", static_cast<int>(__own_pos_est.x * 1000), static_cast<int>(__own_pos_est.y * 1000), __own_position.ori);
  else       printf("%0.03f %0.03f %0.03f\n\n", __own_pos_est.x, __own_pos_est.y, __own_pos_est.ori);

  printf("Field lines (start-x -y end-x -y):\n==================================\n");
  for (FieldLines::iterator it = __lines->begin(); it != __lines->end(); ++it) {
    if (in_mm) printf("%d %d %d %d\n", static_cast<int>(it->start.x * 1000), static_cast<int>(it->start.y * 1000), static_cast<int>(it->end.x * 1000), static_cast<int>(it->end.y * 1000));
    else       printf("%0.03f %0.03f %0.03f %0.03f\n", it->start.x, it->start.y, it->end.x, it->end.y);
  }
  printf("\n\n");

  printf("Line points (x y):\n======================\n");
  for (fld_line_points::const_iterator it = __points.begin(); it != __points.end(); ++it)
  {
    if (in_mm) printf("%d %d\n", static_cast<int>(it->x * 1000), static_cast<int>(it->y * 1000));
    else       printf("%0.03f %0.03f\n", it->x, it->y);
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
  else throw fawkes::IllegalArgumentException("Unknown field name! Please set field_name to a valid value (see field.h)");
}

