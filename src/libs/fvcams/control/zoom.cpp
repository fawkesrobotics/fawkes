
/***************************************************************************
 *  zoom.cpp - Abstract class defining a camera zoom controller
 *
 *  Created: Wed Apr 22 10:50:53 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#include <fvcams/control/zoom.h>
#include <core/exceptions/software.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlZoom <fvcams/control/zoom.h>
 * Camera zoom control interface.
 * Some cameras feature zooming.
 *
 * This interface shall be implemented by such cameras.
 *
 * @author Tim Niemueller
 * @author Tobias Kellner
 *
 * @fn void CameraControlZoom::reset_zoom() = 0
 * Reset zoom.
 * @throws NotImplementedException Not implemented by this control
 *
 * @fn void CameraControlZoom::set_zoom(unsigned int zoom) = 0
 * Set new camera-specific zoom value.
 * @param zoom zoom value
 *
 * @fn unsigned int CameraControlZoom::zoom() = 0
 * Get current zoom value.
 * @return current zoom value.
 *
 * @fn unsigned int CameraControlZoom::zoom_max() = 0
 * Maximum zoom value.
 * @return maximum zoom value
 *
 * @fn unsigned int CameraControlZoom::zoom_min() = 0
 * Minimum zoom value.
 * @return Minimum zoom value
 */

using fawkes::NotImplementedException;

/** Empty virtual destructor. */
CameraControlZoom::~CameraControlZoom()
{
}


/** Set speed in tele range.
 * @param speed camera-specific speed value
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlZoom::set_zoom_speed_tele(unsigned int speed)
{
  throw NotImplementedException("Not implemented");
}


/** Set speed in wide range.
 * @param speed camera-specific speed value.
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlZoom::set_zoom_speed_wide(unsigned int speed)
{
  throw NotImplementedException("Not implemented");
}


/** Set if digital zoom may be used.
 * @param enabled true, to enable digital zoom, false otherwise
 * @throws NotImplementedException Not implemented by this control
 */
void
CameraControlZoom::set_zoom_digital_enabled(bool enabled)
{
  throw NotImplementedException("Not implemented");
}

} // end namespace firevision
