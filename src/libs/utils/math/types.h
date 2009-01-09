
/***************************************************************************
 *  types.h - Simple math related types
 *
 *  Created: Thu Oct 30 14:32:38 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_MATH_TYPES_H_
#define __UTILS_MATH_TYPES_H_

namespace fawkes {

/** Point with cartesian coordinates as unsigned integers. */
typedef struct {
  unsigned int x;       /**< x coordinate */
  unsigned int y;       /**< y coordinate */
} point_t;

/** Cartesian coordinates. */
typedef struct {
  float x;      /**< x coordinate */
  float y;      /**< y coordinate */
} cart_coord_2d_t;

/** Polar coordinates. */
typedef struct {
  float r;      /**< distance */
  float phi;    /**< angle */
} polar_coord_2d_t;

/** Rectangular extent with unsigne integers. */
typedef struct {
  unsigned int w;       /**< width */
  unsigned int h;       /**< height */
} extent_2d_t;

/** Rectangle (unsigned integers) */
typedef struct {
  point_t      start;      /**< start point */
  extent_2d_t  extent;     /**< extent */
} rectangle_t;

/** Position on the field. */
typedef struct {
  float x;      /**< x coordinate in meters */
  float y;      /**< y coordinate in meters */
  float ori;    /**< orientation */
} field_pos_t;

/** Describes a field line */
typedef struct field_line_struct{
  cart_coord_2d_t start;   /**< start of the line [m] */
  cart_coord_2d_t end;     /**< end of the line [m] */

  /** 
   * Constructor
   * @param start of the line
   * @param end of the line
   */
  field_line_struct(fawkes::cart_coord_2d_t start, fawkes::cart_coord_2d_t end)
  {
    this->start = start;
    this->end   = end;
  }

  /** 
   * Constructor
   * @param start_x of the line
   * @param start_y of the line
   * @param end_x of the line
   * @param end_y of the line
   */
  field_line_struct(float start_x, float start_y, float end_x, float end_y)
  {
    this->start.x = start_x;
    this->start.y = start_y;
    this->end.x   = end_x;
    this->end.y   = end_y;
  }
} field_line_t;


} // end namespace fawkes

#endif
