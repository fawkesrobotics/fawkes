
/***************************************************************************
 *  clips_env_manager.h - CLIPS environment manager
 *
 *  Created: Thu Aug 15 18:55:32 2013
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Clock;
class CLIPSFeature;

class CLIPSEnvManager
{
 public:
  CLIPSEnvManager(Logger *logger, Clock *clock, std::string &clips_dir);
  virtual ~CLIPSEnvManager();

  LockPtr<CLIPS::Environment>
    create_env(const std::string &env_name, const std::string &log_component_name);
  void destroy_env(const std::string &env_name);

  void add_features(const std::list<CLIPSFeature *> &features);
  void remove_features(const std::list<CLIPSFeature *> &features);
  void assert_can_remove_features(const std::list<CLIPSFeature *> &features);

  std::map<std::string, LockPtr<CLIPS::Environment>> environments() const;

 private:
  LockPtr<CLIPS::Environment> new_env(const std::string &log_component_name);
  void assert_features(LockPtr<CLIPS::Environment> &clips, bool immediate_assert);
  void add_functions(const std::string &env_name, LockPtr<CLIPS::Environment> &clips);
  CLIPS::Value clips_request_feature(std::string env_name, std::string feature_name);
  CLIPS::Values clips_now();
  void guarded_load(const std::string &env_name, const std::string &filename);


 private:
  Logger *logger_;
  Clock  *clock_;

  std::string clips_dir_;

  /// @cond INTERNAL
  typedef struct {
    LockPtr<CLIPS::Environment> env;
    std::list<std::string>      req_feat;
  } ClipsEnvData;
  /// @endcond

  std::map<std::string, ClipsEnvData > envs_;
  std::map<std::string, CLIPSFeature * > features_;
};

} // end namespace fawkes

#endif
