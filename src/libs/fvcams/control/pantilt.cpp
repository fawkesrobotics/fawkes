
/***************************************************************************
 *  pantilt.cpp - Abstract class defining a pan/tilt camera controller
 *
 *  Created: Tue Apr 21 22:39:20 2009
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

#include <fvcams/control/pantilt.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlPanTilt <fvcams/control/pantilt.h>
 * Camera pan/tilt control interface.
 * Some cameras feature an actuator to allow for panning and tilting the
 * camera.
 *
 * This interface shall be implemented by such cameras or for other external
 * units for panning and tilting.
 *
 * @author Tim Niemueller
 * @author Tobias Kellner
 *
 * @fn void CameraControlPanTilt::process_pantilt() = 0
 * Process pan/tilt information.
 * Some operations allow for asynchronous usage (like fetching pan/tilt data).
 * This is because some cameras need some time to retrieve the information and
 * thus it is a good idea to let that run besides the image processing loop. With
 * process_control the incoming information is processed.
 *
 * @fn bool CameraControlPanTilt::supports_pan() = 0
 * Check whether this controller supports panning.
 * @return true if panning is supported
 *
 * @fn bool CameraControlPanTilt::supports_tilt() = 0
 * Check whether this controller supports tilting.
 * @return true if tilting is supported
 *
 * @fn void CameraControlPanTilt::set_pan(int pan) = 0
 * Set pan value.
 * The pan value is dependent on the camera control. See the implementations
 * documentation for details.
 * @param pan new pan value
 *
 * @fn void CameraControlPanTilt::set_tilt(int tilt) = 0
 * Set tilt value.
 * The tilt value is dependent on the camera control. See the implementations
 * documentation for details.
 * @param tilt new tilt value
 *
 * @fn void CameraControlPanTilt::set_pan_tilt(int pan, int tilt) = 0
 * Set pan and tilt in one go.
 * Sometimes camera controls have a command for setting pan and tilt at the
 * same time. If possible this should be preferred since is minimizes the
 * number of required operations and communication acts. See the
 * implementations documentation for details.
 * @param pan new pan value
 * @param tilt new tilt value
 *
 * @fn void CameraControlPanTilt::set_pan_tilt_rad(float pan, float tilt) = 0
 * Set pan and tilt as float value.
 * You give a radiant value where the camera should head relative to the basic
 * camera position. Implementations shall look forward (center the camera) for
 * if pan equals zero, look right if the pan is positive and left is the pan is
 * negative, they shall look forward (vertically centered) if tilt is zero,
 * upwards if tilt is negative and downwards if tilt is positive.
 * @param pan new pan value in radiant
 * @param tilt new tilt value in radiant
 *
 * @fn int CameraControlPanTilt::pan() = 0
 * Get pan value
 * @return camera control specific pan value
 *
 * @fn int CameraControlPanTilt::tilt() = 0
 * Get tilt value
 * @return camera control specific tilt value
 *
 * @fn void CameraControlPanTilt::start_get_pan_tilt() = 0
 * Start asynchronous fetch operation for pan and tilt values.
 * This will initiate fetching the pan and tilt values but will not wait until
 * the values have been received but will return immediately (non-blocking).
 *
 * @fn void CameraControlPanTilt::pan_tilt(int &pan, int &tilt) = 0
 * Get pan and tilt at the same time.
 * This will store the current pan and tilt values in the given arguments.
 * @param pan contains current pan after call
 * @param tilt contains current tilt after call
 *
 * @fn void CameraControlPanTilt::pan_tilt_rad(float &pan, float &tilt) = 0
 * Get pan and tilt at the same time in radiant.
 * This will store the current pan and tilt values in the given arguments.
 * @param pan contains current pan after call
 * @param tilt contains current tilt after call
 * @see set_pan_tilt_rad()
 *
 * @fn int CameraControlPanTilt::min_pan()
 * Get minimum pan value.
 * @return minimum camera-specific pan value
 *
 * @fn int CameraControlPanTilt::max_pan()
 * Get maximum pan value.
 * @return maximum camera-specific pan value
 *
 * @fn int CameraControlPanTilt::min_tilt()
 * Get minimum tilt value.
 * @return minimum camera-specific tilt value
 *
 * @fn int CameraControlPanTilt::max_tilt()
 * Get maximum tilt value.
 * @return maximum camera-specific tilt value
 *
 * @fn void CameraControlPanTilt::reset_pan_tilt()
 * Bring camera into home position.
 * After the reset the camera shall look forward (horizontally and
 * vertically centered "home" position).
 *
 * @fn void CameraControlPanTilt::set_pan_tilt_limit(int pan_left, int pan_right, int tilt_up, int tilt_down) = 0
 * Set pan/tilt limits.
 * Some camera controls allow for extra constraints to the min and max pan/tilt
 * values.
 * @param pan_left new minimum pan limit
 * @param pan_right new maximum pan limit
 * @param tilt_up new minimum tilt limit
 * @param tilt_down new maximum tilt limit
 *
 * @fn void CameraControlPanTilt::reset_pan_tilt_limit() = 0
 * Reset pan/tilt limits.
 * This removes all limits from the pan/tilt methods thus the only constraints
 * are hardware induced.
 */

/** Empty virtual destructor. */
CameraControlPanTilt::~CameraControlPanTilt()
{
}


} // end namespace firevision
