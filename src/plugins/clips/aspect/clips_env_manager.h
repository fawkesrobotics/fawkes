
/***************************************************************************
 *  clips_env_manager.h - CLIPS environment manager
 *
 *  Created: Thu Aug 15 18:55:32 2013
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

#ifndef __PLUGINS_CLIPS_ASPECT_CLIPS_ENV_MANAGER_H_
#define __PLUGINS_CLIPS_ASPECT_CLIPS_ENV_MANAGER_H_

#include <core/utils/lockptr.h>
#include <string>
#include <map>
#include <list>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class CLIPSEnvManager
{
 public:
  CLIPSEnvManager(Logger *logger);
  virtual ~CLIPSEnvManager();

  LockPtr<CLIPS::Environment>
    create_env(const std::string &env_name, const std::string &log_component_name);
  void destroy_env(const std::string &env_name);

 public:

 private:
  LockPtr<CLIPS::Environment> new_env(const std::string &log_component_name);

 private:
  Logger *logger_;
  /// @cond INTERNAL
  typedef struct {
    LockPtr<CLIPS::Environment> env;
    std::list<std::string>      required_features;
  } ClipsEnvData;
  /// @endcond

  std::map<std::string, ClipsEnvData > envs_;
  //std::map<std::string, CLIPSFeatureAspect * > features_;

};

} // end namespace fawkes

#endif
