
/***************************************************************************
 *  vision_master.h - Fawkes VisionMasterAspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:53:50 2010
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

#ifndef __ASPECT_INIFINS_VISION_MASTER_H_
#define __ASPECT_INIFINS_VISION_MASTER_H_

#include <aspect/inifins/inifin.h>
#include <aspect/vision_master.h>
#include <aspect/vision.h>
#include <utils/constraints/dependency_onetomany.h>

namespace firevision {
  class VisionMaster;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VisionMasterAspectIniFin : public AspectIniFin
{
 public:
  VisionMasterAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
  virtual bool prepare_finalize(Thread *thread);
  
  firevision::VisionMaster *  vision_master();
  void add_vision_thread(VisionAspect *thread);
  void remove_vision_thread(VisionAspect *thread);
  bool can_remove_vision_thread(VisionAspect *thread);

 private:
  OneToManyDependency<VisionMasterAspect, VisionAspect> __vision_dependency;
};

} // end namespace fawkes

#endif
