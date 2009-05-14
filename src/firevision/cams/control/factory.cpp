
/***************************************************************************
 *  factory.cpp - Camera control factory
 *
 *  Created: Fri Jun 15 13:11:28 2007
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

#include <cams/control/factory.h>
#include <fvutils/system/camargp.h>
#include <core/exceptions/software.h>

#include <cams/control/color.h>
#include <cams/control/image.h>
#include <cams/control/effect.h>
#include <cams/control/focus.h>
#include <cams/control/pantilt.h>
#include <cams/control/zoom.h>
#include <cams/control/dummy.h>
#include <cams/cam_exceptions.h>

#ifdef HAVE_VISCA_CTRL
#include <cams/visca.h>
#endif
#ifdef HAVE_EVID100P_CTRL
#include <cams/sony_evid100p_control.h>
#endif
#ifdef HAVE_DPPTU_CTRL
#include <cams/dpptu.h>
#endif

using namespace std;

/** @class CameraControlFactory <cams/control/factory.h>
 * Camera control factory.
 * This camera control factory provides access to all camera controls in a unified way.
 * You just supply a camera argument string and depending on the camera ID and compile-time
 * support of camera control types an instance of the desired camera control is
 * returned or otherwise an exception is thrown. See instance() for a list of
 * supported camera control types.
 *
 * @author Tim Niemueller
 */

/** Get camera control instance with parameters from given camera argument parser.
 * This is a convenience method and works like instace(const char *as).
 * @param cap camera argument parser
 * @return camera instance
 * @exception UnknownCameraControlTypeException thrown if camera type is not known or
 * was not available at build time.
 */
CameraControl *
CameraControlFactory::instance(const CameraArgumentParser *cap)
{
  CameraControl *c = NULL;

  // ######
  if ( cap->cam_type() == "evid100p" ) {
#ifdef HAVE_EVID100P_CTRL
    c = new SonyEviD100PControl(cap);
#else
    throw UnknownCameraControlTypeException("No EviD100P/Visca support at compile time");
#endif
  }

  // ######
  if ( cap->cam_type() == "dpptu" ) {
#ifdef HAVE_DPPTU_CTRL
    c = new DPPTUControl();
#else
    throw UnknownCameraControlTypeException("No DPPTU support at compile time");
#endif
  }

  // ######
  if ( cap->cam_type() == "dummy" ) {
    c = new DummyCameraControl();
  }

  if ( c == NULL ) {
    throw UnknownCameraControlTypeException();
  }

  return c;
}


/** Get camera control instance.
 * Get an instance of a camera of the given type. The argument string determines
 * the type of camera to open.
 * Supported camera types:
 * - evid100p, SonyEviD100PControl, compiled if HAVE_EVID100P_CTRL is defined in fvconf.mk
 * - dpptu, DPPTUControl, compiled if HAVE_DPPTU_CTRL is defined in fvconf.mk
 * @param as camera argument string
 * @return camera control instance of requested type
 * @exception UnknownCameraControlTypeException thrown, if the desired camera control could
 * not be instantiated. This could be either to a misspelled camera ID, generally
 * missing support or unset definition due to configuration in fvconf.mk or missing
 * libraries and camera support compile-time autodetection.
 */
CameraControl *
CameraControlFactory::instance(const char *as)
{
  CameraArgumentParser *cap = new CameraArgumentParser(as);
  try {
    return instance(cap);
  } catch (UnknownCameraControlTypeException &e) {
    throw;
  }
}


/** Get camera control instance.
 * Get an instance of a camera of the given type based on the given camera.
 * It is tried to cast the camera to the appropriate camera control type. If that
 * succeeds the camera control is returned, otherwise an exception is thrown.
 * @param type_id type ID of the camera control
 * @param camera camera to cast
 * @return camera control instance of requested type
 * @exception UnknownCameraControlTypeException thrown, if the desired camera control could
 * not be instantiated. This could be either to a misspelled camera ID, generally
 * missing support or unset definition due to configuration in fvconf.mk or missing
 * libraries and camera support compile-time autodetection.
 */
CameraControl *
CameraControlFactory::instance(CameraControl::TypeID type_id, Camera *camera)
{
  CameraControl *c = NULL;
  switch (type_id) {
  case CameraControl::CTRL_TYPE_COLOR:
    c = dynamic_cast<CameraControlColor *>(camera);   break;

  case CameraControl::CTRL_TYPE_IMAGE:
    c = dynamic_cast<CameraControlImage *>(camera);   break;

  case CameraControl::CTRL_TYPE_PANTILT:
    c = dynamic_cast<CameraControlPanTilt *>(camera); break;

  case CameraControl::CTRL_TYPE_FOCUS:
    c = dynamic_cast<CameraControlFocus *>(camera);   break;

  case CameraControl::CTRL_TYPE_ZOOM:
    c = dynamic_cast<CameraControlZoom *>(camera);    break;

  case CameraControl::CTRL_TYPE_EFFECT:
    c = dynamic_cast<CameraControlEffect *>(camera);  break;
    
  default:
    throw UnknownCameraControlTypeException();
  }

  if (c) {
    return c;
  } else {
    throw fawkes::TypeMismatchException("Camera does not provide requested camera control");
  }
}
