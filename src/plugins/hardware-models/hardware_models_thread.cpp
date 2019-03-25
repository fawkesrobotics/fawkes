
/***************************************************************************
 *  hardware_models_thread.cpp -  Hardware Models
 *
 *  Created: Sun Mar 24 12:00:06 2019
 *  Copyright  2019  Daniel Habering (daniel@habering.de)
 *  ****************************************************************************/

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

#include "hardware_models_thread.h"

#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <utils/misc/map_skill.h>
#include <interfaces/SwitchInterface.h>
#include <core/threading/mutex_locker.h>
#include <config/yaml.h>

using namespace fawkes;

/** @class HardwareModelsThread "hardware_models_thread.h"
 * Main thread of Hardware Models Plugin.
 *
 * @author Daniel Habering
 */

/** Constructor. */
HardwareModelsThread::HardwareModelsThread()
	: Thread("HardwareModelsThread", Thread::OPMODE_WAITFORWAKEUP),
	  CLIPSFeature("hardware-models"), CLIPSFeatureAspect(this)
{
}


void
HardwareModelsThread::init()
{

}


void
HardwareModelsThread::clips_context_init(const std::string &env_name,
          LockPtr<CLIPS::Environment> &clips)
{
	std::string models_dir;

  models_dir = config->get_string("/hardware-models/models-dir");
  if (models_dir[models_dir.size()-1] != '/') {
      models_dir += "/";
  }

  clips->batch_evaluate(SRCDIR"/hardware_models.clp");

  std::vector<std::string> components  = config->get_strings("/hardware-models/components");
  for (const auto c : components) {
    logger->log_info(name(),c.c_str());
    if (!config->exists(std::string(c + "/states").c_str())) {
      logger->log_warn(name(),"Component config file is missing in %s : %s", models_dir.c_str(),c.c_str());
      continue;
    }
    std::vector<std::string> states = config->get_strings(std::string(c + "/states").c_str());
    if (!states.empty()) {
      for (const auto state : states) {
        if (config->is_list(std::string(c + "/" + state + "/edges").c_str())) {
          std::vector<std::string> edges = config->get_strings(std::string(c + "/" + state + "/edges").c_str());
          for (const auto edge : edges) {
              std::string transition = config->get_string(std::string(c + "/" + state + "/" + edge).c_str());
              clips_add_edge(clips,c,state,edge,transition);
              logger->log_debug(name(),"Edge from %s to %s via %s",state.c_str(),edge.c_str(),transition.c_str());
          }
        }
        logger->log_debug(name(),state.c_str());
      }
      clips_add_component(clips,c,states[0]);
    } else {
      logger->log_warn(name(),"No states for component %s in %s",c.c_str(),models_dir.c_str());
    }
  }
}

void
HardwareModelsThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_debug(name(), "Removing environment %s", env_name.c_str());
}



void
HardwareModelsThread::clips_add_component(LockPtr<CLIPS::Environment> &clips, const std::string& component,const std::string& init_state)
{
  CLIPS::Template::pointer temp = clips->get_template("hm-component");
  if (temp) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("name",component.c_str());
    fact->set_slot("state",init_state.c_str());

    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn(name(), "Asserting component %s failed", component.c_str());
    }

  } else {
    logger->log_warn(name(),"Did not get component template, did you load hardware_models.clp?");
  }
}

void
HardwareModelsThread::clips_add_edge(LockPtr<CLIPS::Environment> &clips, const std::string& component, const std::string& from, const std::string& to, const std::string& trans)
{
  CLIPS::Template::pointer temp = clips->get_template("hm-edge");
  if (temp) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("component",component.c_str());
    fact->set_slot("from",from.c_str());
    fact->set_slot("to",to.c_str());
    fact->set_slot("transition",trans.c_str());

    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn(name(), "Asserting edge from %s to %s failed", from.c_str(),to.c_str());
    }

  } else {
    logger->log_warn(name(),"Did not get edge template, did you load hardware_models.clp?");
  }


}

void
HardwareModelsThread::finalize()
{
}


void
HardwareModelsThread::loop()
{
}



