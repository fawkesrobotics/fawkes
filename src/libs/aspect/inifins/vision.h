
/***************************************************************************
 *  vision.h - Fawkes VisionMasterAspect initializer/finalizer
 *
 *  Created: Wed Nov 24 00:09:35 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_INIFINS_VISION_H_
#define __ASPECT_INIFINS_VISION_H_

#include <aspect/inifins/inifin.h>
#include <aspect/vision.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VisionMasterAspectIniFin;

class VisionAspectIniFin : public AspectIniFin
{
 public:
  VisionAspectIniFin(VisionMasterAspectIniFin *master_inifin);
  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
  virtual bool prepare_finalize(Thread *thread);

 private:
  VisionMasterAspectIniFin *__master_inifin;
};

} // end namespace fawkes

#endif
