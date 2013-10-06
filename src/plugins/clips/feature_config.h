
/***************************************************************************
 *  feature_config.h - CLIPS config feature
 *
 *  Created: Sun Oct 06 13:06:14 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_CLIPS_FEATURE_CONFIG_H_
#define __PLUGINS_CLIPS_FEATURE_CONFIG_H_

#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
  class Configuration;
  class Logger;
}

class ConfigCLIPSFeature : public fawkes::CLIPSFeature
{
 public:
  ConfigCLIPSFeature(fawkes::Logger *logger, fawkes::Configuration *config);
  virtual ~ConfigCLIPSFeature();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 private: // members
  fawkes::Logger        *logger_;
  fawkes::Configuration *config_;

  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

 private: // methods
  void clips_config_load(std::string env_name, std::string cfg_prefix);
  void clips_config_cleanup(std::string env_name);

};

#endif
