
/***************************************************************************
 *  controlfactory.h - Camera control factory
 *
 *  Created: Fri Jun 15 13:11:11 2007
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

#ifndef __FIREVISION_CAMS_CONTROLFACTORY_H_
#define __FIREVISION_CAMS_CONTROLFACTORY_H_

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <cams/cameracontrol.h>

#include <cstddef>

class CameraArgumentParser;

class UnknownCameraControlTypeException : public fawkes::Exception
{
 public:
  UnknownCameraControlTypeException(const char *msg = NULL);
};

class CameraControlFactory
{
 public:
  static CameraControl * instance(const char *as);
  static CameraControl * instance(const CameraArgumentParser *cap);

  /** Get typed instance of camera.
   * Creates a new instance and converts it to the requested type. If the type
   * does not match the requested camera an exception is thrown.
   * @param as camera argument string
   * @return typed camera instance
   * @exception TypeMismatchException thrown, if requested camera does not match
   * requested type.
   */
  template <class C>
    static C* instance(const char *as);
};


template <class C>
C *
CameraControlFactory::instance(const char *as)
{
  CameraControl *c = CameraControlFactory::instance(as);
  C *tc = dynamic_cast<C *>(c);
  if ( tc == NULL ) {
    throw fawkes::TypeMismatchException();
  }
  return tc;
}

#endif
