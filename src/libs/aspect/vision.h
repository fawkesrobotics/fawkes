
/***************************************************************************
 *  vision.h - Vision aspect for Fawkes
 *
 *  Created: Tue May 29 14:47:03 2007
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

#ifndef __ASPECT_VISION_H_
#define __ASPECT_VISION_H_

#include <aspect/aspect.h>
#include <fvutils/base/vision_master.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class VisionAspect : public virtual Aspect
{
 public:
  /** The operation mode of this vision thread.
   * @see Thread
   */
  typedef enum {
    CYCLIC,	/**< cyclic mode */
    CONTINUOUS	/**< continuous mode */
  } VisionThreadMode;

  VisionAspect(VisionThreadMode mode);
  virtual ~VisionAspect();

  void              init_VisionAspect(firevision::VisionMaster *vision_master);
  VisionThreadMode  vision_thread_mode();
 protected:
  /** Vision master */
  firevision::VisionMaster *vision_master;
 private:
  VisionThreadMode __vision_thread_mode;
};

} // end namespace fawkes

#endif
