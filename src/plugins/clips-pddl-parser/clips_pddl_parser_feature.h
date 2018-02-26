/***************************************************************************
 *  clips_pddl_parser_feature.h - CLIPS PDDL Parser feature
 *
 *  Created: Mon 16 Oct 2017 10:20:07 CEST 10:20
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef __PLUGINS_CLIPS_PDDL_PARSER_FEATURE_PDDL_H_
#define __PLUGINS_CLIPS_PDDL_PARSER_FEATURE_PDDL_H_

#include <plugins/clips/aspect/clips_feature.h>

#include <string>
#include <map>

namespace CLIPS {
  class Environment;
}

namespace fawkes {
  class Logger;
}

class PDDLCLIPSFeature : public fawkes::CLIPSFeature
{
 public:
  PDDLCLIPSFeature(fawkes::Logger *logger);
  //virtual ~PDDLCLIPSFeature();
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 private:
  void parse_domain(std::string env_name, std::string domain_file);

 private:
  fawkes::Logger     *logger_;
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;
};

#endif /* !__PLUGINS_CLIPS_PDDL_PARSER_FEATURE_PDDL_H_ */
