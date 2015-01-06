
/***************************************************************************
 *  emergency_motor_instruct.h - Motor instructor with quadratic approximation
 *
 *  Created: Thu Jul 10:35:23 2014
 *  Copyright  2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_DRIVE_REALIZATION_EMERGENCY_MOTORINSTRUCT_H_
#define __PLUGINS_COLLI_DRIVE_REALIZATION_EMERGENCY_MOTORINSTRUCT_H_

#include "base_motor_instruct.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class EmergencyMotorInstruct: public BaseMotorInstruct
{
 public:
  EmergencyMotorInstruct( MotorInterface* motor,
                          float frequency,
                          Logger* logger,
                          Configuration* config );
  virtual ~EmergencyMotorInstruct();

 private:
  ///\brief linear implementation of velocity constraints
  float calculate_rotation( float current, float desired,float time_factor );

  ///\brief linear implementation of velocity constraints
  float calculate_translation( float current, float desired,float time_factor );
};

} // namespace fawkes

#endif
