
/***************************************************************************
 *  cam_exceptions.cpp - Camera-related exceptions
 *
 *  Created: Sat Apr 14 23:07:12 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <cams/cam_exceptions.h>

/** @class CameraNotOpenedException <cams/cam_exceptions.h>
 * Camera not opened exception.
 * Throw this exception if an operations was requested on a camera that is
 * not possible if the camera has not been properly opened before.
 */

/** Constructor. */
CameraNotOpenedException::CameraNotOpenedException()
  : Exception("Camera not opened")
{
}

/** @class CameraNotStartedException <cams/cam_exceptions.h>
 * Camera not started exception.
 * Throw this exception if an operations was requested on a camera that is
 * not possible if the camera has not been properly started before.
 */

/** Constructor. */
CameraNotStartedException::CameraNotStartedException()
  : Exception("Camera not started")
{
}


/** @class CaptureException <cams/cam_exceptions.h>
 * Capturing a frame failed.
 * This exception is thrown if a camera failed to retrieve a new image from
 * the camera.
 */

/** Constructor.
 * @param msg optional message explaining why capturing failed
 */
CaptureException::CaptureException(const char *msg)
  : Exception("Failed to capture a frame: %s", msg)
{
}
