
/***************************************************************************
 *  factory.h - Camera factory
 *
 *  Created: Wed Apr 11 14:40:22 2007
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

#ifndef __FIREVISION_CAMS_FACTORY_H_
#define __FIREVISION_CAMS_FACTORY_H_

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvcams/camera.h>
#include <fvcams/cam_exceptions.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class CameraFactory
{
 public:
  static Camera * instance(const char *as);
  static Camera * instance(const CameraArgumentParser *cap);

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
CameraFactory::instance(const char *as)
{
  Camera *c = CameraFactory::instance(as);
  C *tc = dynamic_cast<C *>(c);
  if ( tc == NULL ) {
    throw fawkes::TypeMismatchException("Returned camera is not of expected type");
  }
  return tc;
}

} // end namespace firevision

#endif
