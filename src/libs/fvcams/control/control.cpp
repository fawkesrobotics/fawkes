
/***************************************************************************
 *  control.cpp - Abstract base class for camera controllers
 *
 *  Created: Sun Jan 21 14:53:32 2007
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *             2009       Tobias Kellner
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

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControl <fvcams/control/control.h>
 * Camera control interface base class.
 * Some cameras feature an actuator to allow for panning and tilting the
 * camera or support additional features like setting the focus, using
 * zoom or using effects.
 *
 * This is the base class for different kinds of such camera controls.
 * They can be instantiated through the CameraControlFactory.
 *
 * @see CameraControlFactory
 * @author Tim Niemueller
 * @author Tobias Kellner
 */

/** Virtual empty destructor. */
CameraControl::~CameraControl()
{
}

} // end namespace firevision
