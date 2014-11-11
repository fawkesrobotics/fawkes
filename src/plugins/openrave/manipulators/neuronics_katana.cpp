
/***************************************************************************
 *  neuronics_katana.cpp - Fawkes to OpenRAVE Neuronics Katana6M180 Manipulator Data
 *
 *  Created: Thu Sep 08 15:34:52 2011
 *  Copyright  2011  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "neuronics_katana.h"
#include "../manipulator.h"

#include <cmath>
#include <cstdio>

 namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRaveManipulatorNeuronicsKatana <plugins/openrave/manipulators/neuronics_katana.h>
 * Class containing information about all neuronics-katana motors.
 * It is the Katana6M180 type. Model is provided by OpenRAVE.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param count number of motors of OpenRAVE model
 * @param countDevice number of motors of real device
 */
OpenRaveManipulatorNeuronicsKatana::OpenRaveManipulatorNeuronicsKatana(unsigned int count, unsigned int countDevice) :
  OpenRaveManipulator( count, countDevice )
{
}

/** Destructor. */
OpenRaveManipulatorNeuronicsKatana::~OpenRaveManipulatorNeuronicsKatana()
{
}

/** Create a new copy of this OpenRaveManipulator instance.
 * @return A pointer to the copied instance
 */
OpenRaveManipulatorPtr
OpenRaveManipulatorNeuronicsKatana::copy()
{
  return RefPtr<OpenRaveManipulatorNeuronicsKatana>( new OpenRaveManipulatorNeuronicsKatana(*this) );
}


/* ########## various ######### */
float
OpenRaveManipulatorNeuronicsKatana::angle_OR_to_device(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = M_PI - angle;
      break;
    case 1:
      _angle = M_PI/2 - angle;
      break;
    case 2:
      _angle = M_PI - angle;
      break;
    case 3:
      _angle = M_PI - angle;
      break;
    case 4:
      _angle = M_PI - angle;
      break;
    default:
      _angle = angle;
      break;
  }

  return _angle;
}

float
OpenRaveManipulatorNeuronicsKatana::angle_device_to_OR(unsigned int number, float angle) const
{
  float _angle;

  switch( number ) {
    case 0:
      _angle = M_PI - angle;
      break;
    case 1:
      _angle = M_PI/2 - angle;
      break;
    case 2:
      _angle = M_PI - angle;
      break;
    case 3:
      _angle = M_PI - angle;
      break;
    case 4:
      _angle = M_PI - angle;
      break;
    default:
      _angle = angle;
      break;
  }

  return _angle;
}
} // end namespace fawkes
