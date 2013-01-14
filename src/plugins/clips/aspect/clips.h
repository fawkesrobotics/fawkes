
/***************************************************************************
 *  clips.h - CLIPS aspect for Fawkes
 *
 *  Created: Sat Jun 16 14:28:31 2012
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_ASPECT_CLIPS_H_
#define __PLUGINS_CLIPS_ASPECT_CLIPS_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLIPSAspect : public virtual Aspect
{
  friend class CLIPSAspectIniFin;

 public:
  CLIPSAspect(const char *log_component_name = 0);
  virtual ~CLIPSAspect();

 protected:
  LockPtr<CLIPS::Environment> clips;

 private:
  const char *  get_CLIPSAspect_log_component_name() const;
  void init_CLIPSAspect(LockPtr<CLIPS::Environment> clips);
  void finalize_CLIPSAspect();

 private:
  const char * CLIPSAspect_log_component_name_;
};

} // end namespace fawkes

#endif
