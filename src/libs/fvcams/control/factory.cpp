
/***************************************************************************
 *  factory.cpp - Camera control factory
 *
 *  Created: Fri Jun 15 13:11:28 2007
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/control/factory.h>
#include <fvutils/system/camargp.h>
#include <core/exceptions/software.h>

#include <fvcams/control/color.h>
#include <fvcams/control/image.h>
#include <fvcams/control/effect.h>
#include <fvcams/control/focus.h>
#include <fvcams/control/pantilt.h>
#include <fvcams/control/zoom.h>
#include <fvcams/control/source.h>
#include <fvcams/control/dummy.h>
#include <fvcams/cam_exceptions.h>

#ifdef HAVE_VISCA_CTRL
#include <fvcams/control/visca.h>
#endif
#ifdef HAVE_EVID100P_CTRL
#include <fvcams/control/sony_evid100p.h>
#endif
#ifdef HAVE_DPPTU_CTRL
#include <fvcams/control/dp_ptu.h>
#endif

#include <typeinfo>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraControlFactory <fvcams/control/factory.h>
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
    c = new DPPTUControl(cap);
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
 * Get an instance of a camera control from the passed camera.
 * It is tried to cast the camera to the appropriate camera control type. If that
 * succeeds the camera control is returned, otherwise an exception is thrown.
 * @param camera camera to cast
 * @return camera control instance.
 * @exception UnknownCameraControlTypeException thrown, if the desired camera control could
 * not be instantiated. This could be either to a misspelled camera ID, generally
 * missing support or unset definition due to configuration in fvconf.mk or missing
 * libraries and camera support compile-time autodetection.
 */
CameraControl *
CameraControlFactory::instance(Camera *camera)
{
  CameraControl *c = dynamic_cast<CameraControl *>(camera);
  if (c) {
    return c;
  } else {
    throw fawkes::TypeMismatchException("Camera does not provide requested camera control");
  }
}


/** Get camera control instance.
 * Get an instance of a camera of the given type based on the given camera.
 * It is tried to cast the camera to the appropriate camera control type. If that
 * succeeds the camera control is returned, otherwise an exception is thrown.
 * @param typeinf type info for the intended type of the camera control
 * @param camera camera to cast
 * @return camera control instance of requested type
 * @exception UnknownCameraControlTypeException thrown, if the desired camera control could
 * not be instantiated. This could be either to a misspelled camera ID, generally
 * missing support or unset definition due to configuration in fvconf.mk or missing
 * libraries and camera support compile-time autodetection.
 */
CameraControl *
CameraControlFactory::instance(const std::type_info &typeinf, Camera *camera)
{
  CameraControl *c = NULL;

  if (typeid(CameraControlColor) == typeinf) {
    c = dynamic_cast<CameraControlColor *>(camera);

  } else if (typeid(CameraControlImage) == typeinf) {
    c = dynamic_cast<CameraControlImage *>(camera);

  } else if (typeid(CameraControlPanTilt) == typeinf) {
    c = dynamic_cast<CameraControlPanTilt *>(camera);

  } else if (typeid(CameraControlFocus) == typeinf) {
    c = dynamic_cast<CameraControlFocus *>(camera);

  } else if (typeid(CameraControlZoom) == typeinf) {
    c = dynamic_cast<CameraControlZoom *>(camera);

  } else if (typeid(CameraControlEffect) == typeinf) {
    c = dynamic_cast<CameraControlEffect *>(camera);

  } else if (typeid(CameraControlSource) == typeinf) {
    c = dynamic_cast<CameraControlSource *>(camera);

  } else {
    throw UnknownCameraControlTypeException();
  }

  if (c) {
    return c;
  } else {
    throw fawkes::TypeMismatchException("Camera does not provide requested camera control");
  }
}

} // end namespace firevision
