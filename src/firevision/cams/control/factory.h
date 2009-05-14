
/***************************************************************************
 *  factory.h - Camera control factory
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

#include <cams/control/control.h>

#include <cstddef>

class Camera;
class CameraArgumentParser;

class CameraControlFactory
{
 public:
  static CameraControl * instance(const char *as);
  static CameraControl * instance(CameraControl::TypeID type_id, Camera *camera);
  static CameraControl * instance(const CameraArgumentParser *cap);

  /** Get typed instance of camera control.
   * Creates a new instance and converts it to the requested type. If the type
   * does not match the requested camera control an exception is thrown.
   * @param as camera argument string
   * @return typed camera control instance
   * @exception TypeMismatchException thrown if requested camera control does not match
   * requested type.
   */
  template <class CC>
    static CC* instance(const char *as);

  /** Get typed instance of camera control.
   * This checks if the given camera provides the desired camera control. If it
   * the properly casted camera is returned, otherwise an exception is thrown.
   * @param camera camera
   * @return typed camera control instance
   * @exception TypeMismatchException thrown if requested camera control does not match
   * requested type.
   */
  template <class CC>
    static CC* instance(Camera *camera);
};


template <class CC>
CC *
CameraControlFactory::instance(const char *as)
{
  CameraControl *cc = CameraControlFactory::instance(as);
  CC *tcc = dynamic_cast<CC *>(cc);
  if (tcc == NULL) throw fawkes::TypeMismatchException();
  return tcc;
}

template <class CC>
CC *
CameraControlFactory::instance(Camera *camera)
{
  CC *tcc = dynamic_cast<CC *>(camera);
  if (tcc == NULL) throw fawkes::TypeMismatchException();
  return tcc;
}

#endif
