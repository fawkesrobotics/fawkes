
/***************************************************************************
 *  types.h - Types that are used throughout the colli
 *
 *  Created: Tue Mar 25 21:51:11 2014
 *  Copyright  2014  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_TYPES_H_
#define __PLUGINS_COLLI_TYPES_H_

#include <utils/math/types.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Colli States */
typedef enum {
  NothingToDo,          /**< Indicating that nothing is to do */
  OrientAtTarget,       /**< Indicating that the robot is at target and has to orient */
  DriveToOrientPoint,   /**< Drive to the orientation point */
  DriveToTarget,        /**< Drive to the target */
} colli_state_t;

/** Colli data, refering to current movement */
typedef struct {
  bool final;                   /**< final-status */
  cart_coord_2d_t local_target; /**< local target */
  cart_coord_2d_t local_trajec; /**< local trajectory */
} colli_data_t;

/** Costs of occupancy-grid cells */
typedef struct {
  unsigned int occ;  /**< The cost for an occupied cell */
  unsigned int near; /**< The cost for a cell near an obstacle (distance="near")*/
  unsigned int mid;  /**< The cost for a cell near an obstacle (distance="near")*/
  unsigned int far;  /**< The cost for a cell near an obstacle (distance="near")*/
  unsigned int free; /**< The cost for a free cell */
} colli_cell_cost_t;

/** Storing Translation and rotation */
typedef struct {
  float x;          /**< Translation in x-direction */
  float y;          /**< Translation in y-direction */
  float rot;         /**< Rotation around z-axis */
} colli_trans_rot_t;

/** Colli Escape modes */
typedef enum {
  potential_field,
  basic
} colli_escape_mode_t;

/** Colli motor_instuct modes */
typedef enum {
  linear,               // The enum for the linear motor instruct
  quadratic             // The enum for the quadratic motor instruct
} colli_motor_instruct_mode_t;

/** Colli drive restrictions */
typedef enum {
  differential,               // The enum for the differential drive restriction
  omnidirectional             // The enum for the omnidirectional drive restriction
} colli_drive_restriction_t;


} // end namespace firevision

#endif
