
/***************************************************************************
 *  feature_blackboard.h - CLIPS blackboard feature
 *
 *  Created: Thu Oct 03 11:46:20 2013
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

#ifndef __PLUGINS_CLIPS_FEATURE_BLACKBOARD_H_
#define __PLUGINS_CLIPS_FEATURE_BLACKBOARD_H_

#include <plugins/clips/aspect/clips_feature.h>

#include <map>
#include <string>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
  class BlackBoard;
  class Logger;
  class Interface;
}

class BlackboardCLIPSFeature : public fawkes::CLIPSFeature
{
 public:
  BlackboardCLIPSFeature(fawkes::Logger *logger, fawkes::BlackBoard *blackboard);
  virtual ~BlackboardCLIPSFeature();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 private: // members
  fawkes::Logger     *logger_;
  fawkes::BlackBoard *blackboard_;

  typedef std::multimap<std::string, fawkes::Interface *> InterfaceMap;
  std::map<std::string, InterfaceMap >  interfaces_;
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

 private: // methods
  void clips_blackboard_open_interface(std::string env_name, std::string type, std::string id);
  void clips_blackboard_read(std::string env_name);

};

#endif
