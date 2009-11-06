
/***************************************************************************
 *  cam_exceptions.h - Camera-related exceptions
 *
 *  Created: Sat Apr 14 23:05:52 2007
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

#ifndef __FIREVISION_CAMS_CAM_EXCEPTIONS_H_
#define __FIREVISION_CAMS_CAM_EXCEPTIONS_H_

#include <core/exception.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraNotOpenedException : public fawkes::Exception
{
 public:
  CameraNotOpenedException();
};

class CameraNotStartedException : public fawkes::Exception
{
 public:
  CameraNotStartedException();
};


class CaptureException : public fawkes::Exception
{
 public:
  CaptureException(const char *format, ...);
};

class UnknownCameraTypeException : public fawkes::Exception
{
 public:
  UnknownCameraTypeException(const char *msg = 0);
};

class UnknownCameraException : public fawkes::Exception
{
 public:
  UnknownCameraException(const char *msg = 0);
};

class UnknownCameraControlTypeException : public fawkes::Exception
{
 public:
  UnknownCameraControlTypeException(const char *msg = 0);
};

} // end namespace firevision

#endif
