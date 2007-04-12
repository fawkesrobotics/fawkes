
/***************************************************************************
 *  factory.h - Camera factory
 *
 *  Created: Wed Apr 11 14:40:22 2007
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_CAMS_FACTORY_H_
#define __FIREVISION_CAMS_FACTORY_H_

#include <core/exception.h>
#include <cams/camera.h>

class UnknownCameraTypeException : public Exception
{
 public:
  UnknownCameraTypeException() : Exception("Unknown camera type") {}
};

class CameraFactory
{
 public:
  static Camera * instance(const char *idents);

  template <class C>
    static C* instance(const char *idents);
};


template <class C>
C *
CameraFactory::instance(const char *idents)
{
  Camera *c = CameraFactory::instance(idents);
  C *tc = dynamic_cast<C *>(c);
  return tc;
}

#endif
