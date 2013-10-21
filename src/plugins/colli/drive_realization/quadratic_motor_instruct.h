
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

class MotorInterface;
class Logger;
class Configuration;

/** This module is a class for validity checks of drive
 * commands and sets those things with respect to the physical
 * borders of the robot.
 * For this purpose the two functions CalculateRotation and
 * CalculateTranslation are implemented quadratically ;-)
 */
class CQuadraticMotorInstruct: public CBaseMotorInstruct
{
public:

  ///
  CQuadraticMotorInstruct( fawkes::MotorInterface* motor,
                           fawkes::MotorInterface* motor_des,
                           float frequency,
                           fawkes::Logger* logger,
                           fawkes::Configuration* config );

  ///
  virtual ~CQuadraticMotorInstruct();


private:

  // implementation of the virtual functions of the base class


  /// linear implementation of velocity constraints
  float CalculateRotation( float currentRotation, float desiredRotation,
         float time_factor );

  /// linear implementation of velocity constraints
  float CalculateTranslation( float currentTranslation, float desiredTranslation,
            float time_factor );


  /// maximum acceleration and deceleration values for translation and rotation
  float basic_trans_acc;
  float basic_trans_dec;
  float basic_rot_acc;
  float basic_rot_dec;

  fawkes::Configuration* config_;

};

} // namespace fawkes

#endif

