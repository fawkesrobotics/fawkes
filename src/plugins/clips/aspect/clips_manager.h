
/***************************************************************************
 *  clips_manager.h - CLIPS manager aspect for Fawkes
 *
 *  Created: Thu Aug 15 18:50:07 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_ASPECT_CLIPS_MANAGER_H_
#define __PLUGINS_CLIPS_ASPECT_CLIPS_MANAGER_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>
#include <plugins/clips/aspect/clips_env_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLIPSManagerAspectIniFin;

class CLIPSManagerAspect : public virtual Aspect
{
  friend CLIPSManagerAspectIniFin;

 public:
  CLIPSManagerAspect();
  virtual ~CLIPSManagerAspect();

 protected:
  LockPtr<CLIPSEnvManager> clips_env_mgr;

};

} // end namespace fawkes

#endif
