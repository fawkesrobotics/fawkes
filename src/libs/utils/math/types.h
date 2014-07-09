
/***************************************************************************
 *  types.h - Simple math related types
 *
 *  Created: Thu Oct 30 14:32:38 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef M_TWO_PI
#define M_TWO_PI 6.28318530717959
#endif

namespace fawkes {

/** Point with cartesian coordinates as unsigned integers. */
typedef struct {
  unsigned int x;       /**< x coordinate */
  unsigned int y;       /**< y coordinate */
} upoint_t;

/** Point with cartesian coordinates as signed integers. */
typedef struct point_struct {
  int x;       /**< x coordinate */
  int y;       /**< y coordinate */

  /** Default constructor */
  point_struct() {}

  /** Constructor.
   * @param x The x coordinate
   * @param y The y coordinate
   */
  point_struct(int x, int y) {
    this->x = x;
    this->y = y;
  }

} point_t;

/** Cartesian coordinates (2D). */
typedef struct cart_coord_2d_struct {
  float x;      /**< x coordinate */
  float y;      /**< y coordinate */

  /** Default constructor */
  cart_coord_2d_struct() {}

  /** Constructor.
   * @param x The x coordinate.
   * @param y The y coordinate.
   */
  cart_coord_2d_struct(float x, float y) {
    this->x = x;
    this->y = y;
  }

} cart_coord_2d_t;

/** Cartesian coordinates (3D). */
typedef struct {
  float x;      /**< x coordinate */
  float y;      /**< y coordinate */
  float z;      /**< z coordinate */
} cart_coord_3d_t;

/** Polar coordinates. */
typedef struct {
  float r;      /**< distance */
  float phi;    /**< angle */
} polar_coord_2d_t;

/** Polar coordinates. */
typedef struct {
  float r;      /**< distance */
  float phi;    /**< x-y : plane */
  float theta;  /**< plane-z : space */
} polar_coord_3d_t;

/** Rectangular extent with unsigne integers. */
typedef struct {
  unsigned int w;       /**< width */
  unsigned int h;       /**< height */
} extent_2d_t;

/** Rectangle (unsigned integers) */
typedef struct {
  upoint_t      start;      /**< start point */
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

/** Defines an arc (or circle) */
typedef struct arc_struct {
  /** Constructor.
   * @param radius The radius of the arc or circle
   * @param center_x The x-coordinate of the center of the arc or circle
   * @param center_y The y-coordinate of the center of the arc or circle
   * @param start_phi The start angle of the arc
   * @param end_phi The end angle of the arc
   */
  arc_struct(float radius, float center_x, float center_y, float start_phi = 0, float end_phi = M_TWO_PI) {
    this->radius    = radius;
    this->center.x  = center_x;
    this->center.y  = center_y;
    this->start_phi = start_phi;
    this->end_phi   = end_phi;
  }

  float radius;           /**< The radius of the arc or circle */
  cart_coord_2d_t center; /**< The center of the arc or circle */
  float start_phi;        /**< The start angle of the arc */
  float end_phi;          /**< The end angle of the arc */
} arc_t;

/** Defines an ellipse */
typedef struct ellipse_struct {
  cart_coord_2d_t center; /**< The center point of the ellipse */
  float width;            /**< The total width of the ellipse */
  float height;           /**< The total height of the ellipse */

  /** Constructur.
   * @param x The x-coordinate of the center of the ellipse
   * @param y The y-coordinate of the center of the ellipse
   * @param w The total width of the ellipse
   * @param h The total height of the ellipse
   */
  ellipse_struct(float x, float y, float w, float h) {
    this->center.x = x;
    this->center.y = y;
    this->width = w;
    this->height = h;
  }

} ellipse_t;

/** Defines a point with 6-degrees of freedom */
typedef struct point_6D_struct {
  float x;      /**< The x-coordinate of the point */
  float y;      /**< The y-coordinate of the point */
  float z;      /**< The z-coordinate of the point */
  float roll;   /**< The angle around the x-axis */
  float pitch;  /**< The angle around the y-axis */
  float yaw;    /**< The angle around the z-axis */
} point_6D_t;

} // end namespace fawkes

#endif
