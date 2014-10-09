
/***************************************************************************
 *  quadratic_motor_instruct.h - Motor instructor with quadratic approximation
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_DRIVE_REALIZATION_QUADRATIC_MOTORINSTRUCT_H_
#define __PLUGINS_COLLI_DRIVE_REALIZATION_QUADRATIC_MOTORINSTRUCT_H_

#include "base_motor_instruct.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class QuadraticMotorInstruct: public BaseMotorInstruct
{
 public:
  QuadraticMotorInstruct( MotorInterface* motor,
                          float frequency,
                          Logger* logger,
                          Configuration* config );
  virtual ~QuadraticMotorInstruct();

 private:
  ///\brief linear implementation of velocity constraints
  float calculate_rotation( float current, float desired, float time_factor );

  ///\brief linear implementation of velocity constraints
  float calculate_translation( float current, float desired, float time_factor );
};

} // namespace fawkes
#endif
