
/***************************************************************************
 *  types.h - Definition of simple types
 *
 *  Generated: Sun May 08 22:29:34 2005
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2005       Martin Heracles
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

#ifndef __FIREVISION_UTILS_TYPE_H_
#define __FIREVISION_UTILS_TYPE_H_

#include <utils/math/types.h>
#include <stdint.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Center in ROI.
 * Must be signed since the center of a ball may be out of the ROI.
 */
typedef struct {
  float x;	/**< x in pixels */
  float y;	/**< y in pixels */
} center_in_roi_t;

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


#pragma pack(push,4)
/** Structure defining a point in a CARTESIAN_3D_FLOAT buffer */
typedef struct {
  float x;	/**< X value */
  float y;	/**< Y value */
  float z;	/**< Z value */
} pcl_point_t;

/** Structure defining a point in a CARTESIAN_3D_FLOAT_RGB buffer */
typedef struct {
  float x;	/**< X value */
  float y;	/**< Y value */
  float z;	/**< Z value */
  union {
    struct {
      uint8_t b;	/**< B color component value */
      uint8_t g;	/**< G color component value */
      uint8_t r;	/**< R color component value */
      uint8_t _unused;	/**< unused */
    }; /**< Color as RGB struct */
    float rgb;		/**< Color value as float */
  }; /**< Union representing color as separate pairs or float */
} pcl_point_xyzrgb_t;
#pragma pack(pop)

} // end namespace firevision

#endif
