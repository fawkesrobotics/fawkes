
/***************************************************************************
 *  source.h - Abstract class defining a camera source controller
 *
 *  Created: Mon Jun 29 15:47:11 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_CONTROL_SOURCE_H_
#define __FIREVISION_CAMS_CONTROL_SOURCE_H_

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraControlSource : virtual public CameraControl
{
 public:
  virtual ~CameraControlSource();

  virtual unsigned char source()                         = 0;
  virtual void          set_source(unsigned char source) = 0;
};

} // end namespace firevision

#endif // __FIREVISION_CAMS_CONTROL_SOURCE_H_
