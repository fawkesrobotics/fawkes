
/***************************************************************************
 *  clips_feature.h - CLIPS feature aspect for Fawkes
 *
 *  Created: Thu Jul 25 17:37:58 2013
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

#ifndef __PLUGINS_CLIPS_ASPECT_CLIPS_FEATURE_H_
#define __PLUGINS_CLIPS_ASPECT_CLIPS_FEATURE_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

#include <string>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLIPSEnvManager;
class CLIPSFeatureAspectIniFin;

class CLIPSFeature
{
 friend CLIPSEnvManager;

 public:
  CLIPSFeature(const char *feature_name);
  virtual ~CLIPSFeature();

  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips) = 0;
  virtual void clips_context_destroyed(const std::string &env_name) = 0;

 protected:
  const std::string clips_feature_name;	///< CLIPS feature name

};

class CLIPSFeatureAspect : public virtual Aspect
{
  friend CLIPSFeatureAspectIniFin;

 public:
  CLIPSFeatureAspect(CLIPSFeature *feature);
  CLIPSFeatureAspect(const std::list<CLIPSFeature *> features);
  virtual ~CLIPSFeatureAspect();

 private:
  std::list<CLIPSFeature *> clips_features_;
};

} // end namespace fawkes

#endif
