
/***************************************************************************
 *  cam_exceptions.cpp - Camera-related exceptions
 *
 *  Created: Sat Apr 14 23:07:12 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/cam_exceptions.h>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraNotOpenedException <fvcams/cam_exceptions.h>
 * Camera not opened exception.
 * Throw this exception if an operations was requested on a camera that is
 * not possible if the camera has not been properly opened before.
 */

/** Constructor. */
CameraNotOpenedException::CameraNotOpenedException()
  : Exception("Camera not opened")
{
}

/** @class CameraNotStartedException <fvcams/cam_exceptions.h>
 * Camera not started exception.
 * Throw this exception if an operations was requested on a camera that is
 * not possible if the camera has not been properly started before.
 */

/** Constructor. */
CameraNotStartedException::CameraNotStartedException()
  : Exception("Camera not started")
{
}


/** @class CaptureException <fvcams/cam_exceptions.h>
 * Capturing a frame failed.
 * This exception is thrown if a camera failed to retrieve a new image from
 * the camera.
 */

/** Constructor.
 * @param format format of the descriptive message
 */
CaptureException::CaptureException(const char *format, ...)
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}


/** @class UnknownCameraTypeException <fvcams/cam_exceptions.h>
 * Unknown camera type exception.
 * Thrown if the requested camera has not been recognized or the needed
 * libraries were not available at compile time.
 */

/** Constructor.
 * @param msg optional extra message
 */
UnknownCameraTypeException::UnknownCameraTypeException(const char *msg)
  : Exception("Unknown camera type")
{
  append(msg);
}


/** @class UnknownCameraException <fvcams/cam_exceptions.h>
 * Unknown camera exception.
 * Thrown if the requested camera is not available.
 */

/** Constructor.
 * @param msg optional extra message
 */
UnknownCameraException::UnknownCameraException(const char *msg)
  : Exception("Unknown camera")
{
  append(msg);
}


/** @class UnknownCameraControlTypeException <fvcams/cam_exceptions.h>
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

} // end namespace firevision
