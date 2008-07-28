
/***************************************************************************
 *  controlfactory.cpp - Camera control factory
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

#include <cams/controlfactory.h>
#include <fvutils/system/camargp.h>

#include <cams/dummy_control.h>

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

/** @class UnknownCameraControlTypeException factory.h <cams/factory.h>
 * Unknown camera control exception.
 * Thrown if the requested camera control has not been recognized or the needed
 * libraries were not available at compile time.
 */

/** Constructor.
 * @param msg optional extra message
 */
UnknownCameraControlTypeException::UnknownCameraControlTypeException(const char *msg)
  : Exception("Unknown camera control type")
{
  append(msg);
}


/** @class CameraControlFactory <cams/controlfactory.h>
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
    c = new DummyControl();
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
