
/***************************************************************************
 *  factory.h - Camera control factory
 *
 *  Created: Fri Jun 15 13:11:11 2007
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

#ifndef __FIREVISION_CAMS_CONTROLFACTORY_H_
#define __FIREVISION_CAMS_CONTROLFACTORY_H_

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvcams/control/control.h>

#include <typeinfo>
#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Camera;
class CameraArgumentParser;

class CameraControlFactory
{
 public:
  static CameraControl * instance(const char *as);
  static CameraControl * instance(Camera *camera);
  static CameraControl * instance(const CameraArgumentParser *cap);

  static CameraControl * instance(const std::type_info &typeinf, Camera *camera);
};

} // end namespace firevision

#endif
