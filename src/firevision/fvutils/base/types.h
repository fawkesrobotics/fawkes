
/***************************************************************************
 *  types.h - Definition of simple types
 *
 *  Generated: Sun May 08 22:29:34 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_UTILS_TYPE_H_
#define __FIREVISION_UTILS_TYPE_H_

/** Cartesian coordinates. */
typedef struct {
  unsigned int x;	/**< x coordinate */
  unsigned int y;	/**< y coordinate */
} cart_coord_t;

typedef cart_coord_t point_t;

/** World points, for instance, need to be float-valued. */
typedef struct {
  float x;	/**< x coordinate */
  float y;	/**< y coordinate */
} f_point_t;

/** Rectangular extension of some kind. */
typedef struct {
  unsigned int w;	/**< width */
  unsigned int h;	/**< height */
} extent_t;

/** Rectangle */
typedef struct {
  point_t   start;	/**< start point */
  extent_t  extent;	/**< extent */
} rectangle_t;

/** Center in ROI.
 * Must be signed since the center of a ball may be out of the ROI.
 */
typedef struct {
  float x;	/**< x in pixels */
  float y;	/**< y in pixels */
} center_in_roi_t;

/** Position on the field. */
typedef struct {
  float x;	/**< x coordinate in meters */
  float y;	/**< y coordinate in meters */
  float ori;	/**< orientation */
} field_pos_t;

/** Polar coordinates. */
typedef struct {
  float r;	/**< distance */
  float phi;	/**< angle */
} polar_coord_t;

/** Orientations. */
typedef enum {
  ORI_HORIZONTAL = 1,	/**< horizontal */
  ORI_VERTICAL,		/**< vertical */
  ORI_CROSS,		/**< cross */
  ORI_DEG_0,		/**< 0 degrees */
  ORI_DEG_45,		/**< 45 degrees */
  ORI_DEG_90,		/**< 90 degrees */
  ORI_DEG_135,		/**< 135 degrees */
  ORI_DEG_180,		/**< 180 degrees */
  ORI_DEG_225,		/**< 225 degrees */
  ORI_DEG_270,		/**< 270 degrees */
  ORI_DEG_315,		/**< 315 degrees */
  ORI_DEG_360		/**< 360 degrees */
} orientation_t;

/** The type "color_t" enumerates all colors that are
   of interest in the RoboCup-domain */
typedef enum {
  C_ORANGE = 0,		/**< Orange. */
  C_BACKGROUND = 1,	/**< Background of whatever color. */
  C_MAGENTA = 2,  	/**< Magenta */
  C_CYAN = 3,      	/**< Cyan */
  C_BLUE = 4,   	/**< Blue */
  C_YELLOW = 5,		/**< Yellow */
  C_GREEN = 6,		/**< Green */
  C_WHITE = 7,		/**< White */
  C_RED = 8,		/**< Red */
  C_BLACK = 9,		/**< Black */
  C_OTHER = 10  	/**< Other */
} color_t;


/** datatype to determine the type of the used coordinate system
 * Not that if the robot is positioned at (X=0,Y=0,Ori=0) the robot and world cartesian
 * coordinate systems are the same. This can help to remember the robot coord sys.
 */
typedef enum {
  COORDSYS_UNKNOWN      = 0,	/**< Unknown */
  COORDSYS_ROBOT_CART   = 1,	/**< robot-centric cartesian coordinate system. From
				 * robot forward is positive X, backward is negative X,
				 * right is positive Y, left negative Y */
  COORDSYS_WORLD_CART   = 2,	/**< World cartesian coordinate system, center is at the
				 * middle of the field, from center to opponent goal is
				 * positive X, from center to own goal negative X, from
				 * own goal to opponent goal right wing is positive Y,
				 * left wing is negative Y. */
  COORDSYS_ROBOT_POLAR  = 3,	/**< robot-centric polar coordinate system. Front is zero
				 * rad. */
  COORDSYS_WORLD_POLAR  = 4	/**< world polar coordinate system. Center is zero.
				 * Center to opponent goal is zero rad. */
} coordsys_type_t;


/** Result source. */
typedef enum {
  SRC_UNKNOWN = 0,	/**< unknown */
  SRC_FRONT   = 1,	/**< from front vision */
  SRC_OMNI    = 2	/**< from omni vision */
} source_t;


/** Fieldinfo as sent by Fountain. */
typedef struct {
  source_t     source;			/**< info source */
  long int     time_sec;		/**< timestamp seconds */
  long int     time_usec;		/**< timestamp microseconds */
  color_t      own_goal_color;		/**< own goal color */
  color_t      opp_goal_color;		/**< opponent goal color */
  color_t      own_head_color;		/**< own head color */
  float        pose_x;			/**< pose x */
  float        pose_y;			/**< pose y */
  float        pose_ori;		/**< pose ori */
  float        pan;			/**< pan */
  float        tilt;			/**< tilt */
  int          tracking_mode;		/**< tracking mode */
  float        ball_rel_x;		/**< ball relative x position */
  float        ball_rel_y;		/**< ball relative y position */
  float        ball_glob_x;		/**< ball global x position */
  float        ball_glob_y;		/**< ball global y position */
  float        ball_rel_vel_x;		/**< ball relative velocity in x direction */
  float        ball_rel_vel_y;		/**< ball relative velocity in y direction */
  float        ball_glob_vel_x;		/**< ball global velocity in x direction */
  float        ball_glob_vel_y;		/**< ball global velocity in y direction */
  unsigned int rel_vel_available   :1;	/**< 1 if relative velocity available */
  unsigned int glob_vel_available  :1;	/**< 1 if global velocity available */
  unsigned int ball_visible        :1;	/**< 1 if ball visible */
  unsigned int reserved            :29;	/**< reserved for future use */
} fieldinfo_t;


#endif
