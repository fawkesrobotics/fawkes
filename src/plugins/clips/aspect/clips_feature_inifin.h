
/***************************************************************************
 *  clips_feature_inifin.h - Fawkes CLIPSFeatureAspect initializer/finalizer
 *
 *  Created: Fri Aug 16 13:14:44 2013
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

#ifndef __PLUGINS_CLIPS_ASPECT_CLIPS_FEATURE_INIFIN_H_
#define __PLUGINS_CLIPS_ASPECT_CLIPS_FEATURE_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLIPSEnvManager;

class CLIPSFeatureAspectIniFin : public AspectIniFin
{
 public:
  CLIPSFeatureAspectIniFin();
  ~CLIPSFeatureAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
  virtual bool prepare_finalize(Thread *thread);

  void set_manager(LockPtr<CLIPSEnvManager> &clips_env_mgr);

 private:
  LockPtr<CLIPSEnvManager> clips_env_mgr_;
};

} // end namespace fawkes

#endif
