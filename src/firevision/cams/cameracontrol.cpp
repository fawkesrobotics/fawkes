
/***************************************************************************
 *  cameracontrol.cpp - Abstract class defining a camera controller
 *
 *  Generated: Sun Jan 21 14:53:32 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <cams/cameracontrol.h>

/** @class CameraControl cams/cameracontrol.h
 * Camera control interface.
 * Some cameras feature an actuator to allow for panning and tilting the
 * camera or support additional features like setting the focus, using
 * zoom  or using effects.
 *
 * This interface shall be implemented by such cameras or for other external
 * units for instance for panning and tilting. Since not all cameras support
 * all features (especially not if the control is just for a pan/tilt-unit)
 * there are query methods to check for a certain feature.
 *
 * @author Tim Niemueller
 */


/** No effect constant.
 * This is the only effect constant defined in the interface. All others that
 * may exist are specific for each camera control implementation.
 */
const unsigned int CameraControl::EFFECT_NONE = 0;

/** Virtual empty destructor. */
CameraControl::~CameraControl()
{
}


/** Process information.
 * Some operations allow for asynchronous usage (like fetching pan/tilt data).
 * This is because some cameras need some time to retrieve the information and
 * thus it is a good idea to let that run beside the image processin loop. With
 * process the incoming information is processed.
 */
void
CameraControl::process_control()
{
}


/** Check if camera control supports focus operations.
 * @return true, if the camera supports focus operations, false otherwise
 */
bool
CameraControl::supports_focus()
{
  return false;
}


/** Check if auto focus is enabled.
 * @return true, if the camera is in auto focus mode, false otherwise
 */
bool CameraControl::auto_focus()
{
  return true;
}


/** Enable or disable auto focus.
 * @param enabled if true, enable auto focus, otherwise disable
 */
void
CameraControl::set_auto_focus(bool enabled)
{
}


/** Get current focus value.
 * @return current focus value.
 */
unsigned int
CameraControl::focus()
{
  return 0;
}


/** Set new focus value.
 * @param focus new focus value
 */
void
CameraControl::set_focus(unsigned int focus)
{
}


/** Get minimum focus value.
 * @return minimum focus value.
 */
unsigned int
CameraControl::focus_min()
{
  return 0;
}


/** Get maximum focus value.
 * @return maximum focus value.
 */
unsigned int
CameraControl::focus_max()
{
  return 0;
}


/** Check if camera control supports panning.
 * @return true, if panning is supported, false otherwise
 */
bool
CameraControl::supports_pan()
{
  return false;
}


/** Check if camera control supports tilting.
 * @return true, if tilting is supported, false otherwise
 */
bool
CameraControl::supports_tilt()
{
  return false;
}


/** Set pan value.
 * The pan value is dependent on the camera control. See the implementations
 * documentation for details.
 * @param pan new pan value
 */
void
CameraControl::set_pan(int pan)
{
}


/** Set tilt value.
 * The tilt value is dependent on the camera control. See the implementations
 * documentation for details.
 * @param tilt new tilt value
 */
void
CameraControl::set_tilt(int tilt)
{
}


/** Set pan and tilt in one go.
 * Sometimes camera controls have a command for setting pan and tilt at the
 * same time. If possible this should be preferred since is minimizes the
 * number of required operations and communication acts. See the
 * implementations documentation for details.
 * @param pan new pan value
 * @param tilt new tilt value
 */
void
CameraControl::set_pan_tilt(int pan, int tilt)
{
}



/** Set pan and tilt as float value.
 * You give a radiant value where the camera should head relative to the basic
 * camera position. Implementations shall look forward (center the camera) for
 * if pan equals zero, look right if the pan is positive and left is the pan is
 * negative, they shall look forward (vertically centered) if tilt is zero,
 * upwards if tilt is negative and downwards if tilt is positive.
 * @param pan new pan value in radiant
 * @param tilt new tilt value in radiant
 */
void
CameraControl::set_pan_tilt_rad(float pan, float tilt)
{
}


/** Get pan value
 * @return camera control specific pan value
 */
int
CameraControl::pan()
{
  return 0;
}


/** Get tilt value
 * @return camera control specific tilt value
 */
int
CameraControl::tilt()
{
  return 0;
}


/** Start asynchronous fetch operation for pan and tilt values.
 * This will initiate fetching the pan and tilt values but will not wait until
 * the values have been received but will return immediately (non-blocking).
 */
void
CameraControl::start_get_pan_tilt()
{
}


/** Get pan and tilt at the same time.
 * This will store the current pan and tilt values in the given arguments.
 * @param pan contains current pan after call
 * @param tilt contains current tilt after call
 */
void
CameraControl::pan_tilt(int *pan, int *tilt)
{
  *pan  = 0;
  *tilt = 0;
}


/** Get pan and tilt at the same time in radiant.
 * This will store the current pan and tilt values in the given arguments.
 * @param pan contains current pan after call
 * @param tilt contains current tilt after call
 * @see set_pan_tilt_rad()
 */
void
CameraControl::pan_tilt_rad(float *pan, float *tilt)
{
  *pan  = 0.;
  *tilt = 0.;
}


/** Get minimum pan value.
 * @return minimum camera-specific pan value
 */
int
CameraControl::min_pan()
{
  return 0;
}


/** Get maximum pan value.
 * @return maximum camera-specific pan value
 */
int
CameraControl::max_pan()
{
  return 0;
}


/** Get minimum tilt value.
 * @return minimum camera-specific tilt value
 */
int
CameraControl::min_tilt()
{
  return 0;
}

/** Get maximum tilt value.
 * @return maximum camera-specific tilt value
 */
int
CameraControl::max_tilt()
{
  return 0;
}


/** Bring camera into home position.
 * After the reset the camera shall look forward (horizontally and
 * vertically centered "home" position).
 */
void
CameraControl::reset_pan_tilt()
{
}


/** Set pan/tilt limits.
 * Some camera controls allow for extra constraints to the min and max pan/tilt
 * values.
 * @param pan_left new minimum pan limit
 * @param pan_right new maximum pan limit
 * @param tilt_up new minimum tilt limit
 * @param tilt_down new maximum tilt limit
 */
void
CameraControl::set_pan_tilt_limit(int pan_left, int pan_right, int tilt_up, int tilt_down)
{
}


/** Reset pan/tilt limits.
 * This removes all limits from the pan/tilt methods thus the only constraints
 * are hardware induced.
 */
void
CameraControl::reset_pan_tilt_limit()
{
}


/** Check for support of zoom feature.
 * @return true if zoom is supported, false otherwise
 */
bool
CameraControl::supports_zoom()
{
  return false;
}


/** Reset zoom. */
void
CameraControl::reset_zoom()
{
}


/** Set new camera-specific zoom value.
 * @param zoom zoom value
 */
void
CameraControl::set_zoom(unsigned int zoom)
{
}


/** Get current zoom value.
 * @return current zoom value.
 */
unsigned int
CameraControl::zoom()
{
  return 0;
}


/** Maximum zoom value.
 * @return maximum zoom value
 */
unsigned int
CameraControl::zoom_max()
{
  return 0;
}


/** Minimum zoom value.
 * @return Minimum zoom value
 */
unsigned int
CameraControl::zoom_min()
{
  return 0;
}


/** Set speed in tele range.
 * @param speed camera-specific speed value
 */
void
CameraControl::set_zoom_speed_tele(unsigned int speed)
{
}


/** Set speed in wide range.
 * @param speed camera-specific speed value.
 */
void
CameraControl::set_zoom_speed_wide(unsigned int speed)
{
}


/** Set if digital zoom may be used.
 * @param enabled true, to enable digital zoom, false otherwise
 */
void
CameraControl::set_zoom_digital_enabled(bool enabled)
{
}



/** Check if camera control supports effects.
 * @return true, if effects are supported, false otherwise
 */
bool
CameraControl::supports_effects()
{
  return false;
}


/** Check if camera control supports desired effect.
 * Use camera-specific constants.
 * @param effect supported effect
 * @return true, if effect is supported, false otherwise
 */
bool
CameraControl::supports_effect(unsigned int effect)
{
  return false;
}


/** Enable effect.
 * @param effect camera-specific effect.
 */
void
CameraControl::set_effect(unsigned int effect)
{
}


/** Current effect.
 * @return current effect.
 */
unsigned int
CameraControl::effect()
{
  return EFFECT_NONE;
}


/** Reset effect.
 * Disable all effects.
 */
void
CameraControl::reset_effect()
{
}


/** Get current white balance mode.
 * @return white balance mode
 */
unsigned int
CameraControl::white_balance_mode()
{
  return 0;
}
