
/***************************************************************************
 *  katana6M180.cpp - Fawkes to OpenRAVE Katana6M180 Manipulator Data
 *
 *  Created: Thu Sep 16 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#include "katana6M180.h"

#include "../manipulator.h"

#include <cmath>
#include <cstdio>

namespace fawkes {

/** @class OpenRaveManipulatorKatana6M180 <plugins/openrave/manipulators/katana6M180.h>
 * Class containing information about all katana6M180 motors.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param count number of motors of OpenRAVE model
 * @param countDevice number of motors of real device
 */
OpenRaveManipulatorKatana6M180::OpenRaveManipulatorKatana6M180(unsigned int count,
                                                               unsigned int countDevice)
: OpenRaveManipulator(count, countDevice)
{
}

/** Destructor. */
OpenRaveManipulatorKatana6M180::~OpenRaveManipulatorKatana6M180()
{
}

/** Create a new copy of this OpenRaveManipulator instance.
 * @return A pointer to the copied instance
 */
OpenRaveManipulatorPtr
OpenRaveManipulatorKatana6M180::copy()
{
	return RefPtr<OpenRaveManipulatorKatana6M180>(new OpenRaveManipulatorKatana6M180(*this));
}

/* ########## various ######### */
float
OpenRaveManipulatorKatana6M180::angle_OR_to_device(unsigned int number, float angle) const
{
	float _angle;

	switch (number) {
	case 0: _angle = angle + M_PI; break;
	case 1: _angle = angle + M_PI / 2; break;
	case 2: _angle = angle + M_PI; break;
	case 3: _angle = M_PI - angle; break;
	case 4: _angle = 1.5 * M_PI - angle; break;
	default: _angle = angle; break;
	}

	return _angle;
}

float
OpenRaveManipulatorKatana6M180::angle_device_to_OR(unsigned int number, float angle) const
{
	float _angle;

	switch (number) {
	case 0: _angle = angle - M_PI; break;
	case 1: _angle = angle - M_PI / 2; break;
	case 2: _angle = angle - M_PI; break;
	case 3: _angle = M_PI - angle; break;
	case 4: _angle = 1.5 * M_PI - angle; break;
	default: _angle = angle; break;
	}

	return _angle;
}
} // end namespace fawkes
