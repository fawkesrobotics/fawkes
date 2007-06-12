
/***************************************************************************
 *  cam_exceptions.h - Camera-related exceptions
 *
 *  Created: Sat Apr 14 23:05:52 2007
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

#ifndef __FIREVISION_CAMS_CAM_EXCEPTIONS_H_
#define __FIREVISION_CAMS_CAM_EXCEPTIONS_H_

#include <core/exception.h>

class CameraNotOpenedException : public Exception
{
 public:
  CameraNotOpenedException();
};

class CameraNotStartedException : public Exception
{
 public:
  CameraNotStartedException();
};


class CaptureException : public Exception
{
 public:
  CaptureException(const char *msg);
};
#endif
