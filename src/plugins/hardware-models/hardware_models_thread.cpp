
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
#include <utils/time/wait.h>
#include <unistd.h>
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
	CLIPSFeature("hardware-models"), CLIPSFeatureAspect(this), BlackBoardInterfaceListener("HardwareModelsThread")

{
}


void
HardwareModelsThread::init()
{

  std::string cfg_interface_ = "HardwareModels";
  if (config->exists("/hardware-models/interface")) {
    cfg_interface_ = config->get_string("/hardware-models/interface");
  }

  hm_if_ = blackboard->open_for_writing<HardwareModelsInterface>(cfg_interface_.c_str());
  blackboard->register_listener(this);
  logger->log_error(name(),"Init done");
  wakeup();
  bbil_add_message_interface(hm_if_);
}


void
HardwareModelsThread::clips_context_init(const std::string &env_name,
          LockPtr<CLIPS::Environment> &clips)
{
	std::string models_dir_;

  models_dir_ = config->get_string("/hardware-models/models-dir");
  if (models_dir_[models_dir_.size()-1] != '/') {
      models_dir_ += "/";
  }

  envs_[env_name] = clips;

  clips->batch_evaluate(SRCDIR"/hardware_models.clp");

  components_  = config->get_strings("/hardware-models/components");
  for (const auto c : components_) {
    logger->log_info(name(),c.c_str());
    if (!config->exists(std::string(c + "/states").c_str())) {
      logger->log_warn(name(),"Component config file is missing in %s : %s", models_dir_.c_str(),c.c_str());
      continue;
    }
    std::vector<std::string> states = config->get_strings(std::string(c + "/states").c_str());
    if (!states.empty()) {
      for (const auto state : states) {
        if (config->is_list(std::string(c + "/" + state + "/edges").c_str())) {
          std::vector<std::string> edges = config->get_strings(std::string(c + "/" + state + "/edges").c_str());
          for (const auto edge : edges) {

              std::string transition = "";
              try{
                transition = config->get_string(std::string(c + "/" + state + "/" + edge + "/transition").c_str());
              } catch (...){
                logger->log_error(name(),"Cant find transition value in %s",std::string(c+"/"+state+"/"+edge).c_str());
              }

              double prob = 0.0;
              if (config->exists(std::string(c + "/" + state + "/" + edge + "/probability").c_str())) {
                prob = config->get_float(std::string(c + "/" + state + "/" + edge + "/probability").c_str());
              }

              clips_add_edge(clips,c,state,edge,transition,prob);
              logger->log_debug(name(),"Edge from %s to %s via %s",state.c_str(),edge.c_str(),transition.c_str());
          }
        }
        logger->log_debug(name(),state.c_str());
      }
      clips_add_component(clips,c,states[0]);
    } else {
      logger->log_warn(name(),"No states for component %s in %s",c.c_str(),models_dir_.c_str());
    }
  }
  hm_if_->set_result("Ready");
  hm_if_->write();
}

void
HardwareModelsThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_error(name(), "Removing environment %s", env_name.c_str());
}



void
HardwareModelsThread::clips_add_component(LockPtr<CLIPS::Environment> &clips,const std::string& component,const std::string& init_state)
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
HardwareModelsThread::clips_add_edge(LockPtr<CLIPS::Environment> &clips,const std::string& component, const std::string& from, const std::string& to, const std::string& trans, const double prob)
{
  CLIPS::Template::pointer temp = clips->get_template("hm-edge");
  if (temp) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("component",component.c_str());
    fact->set_slot("from",from.c_str());
    fact->set_slot("to",to.c_str());
    fact->set_slot("transition",trans.c_str());
    fact->set_slot("probability",prob);

    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn(name(), "Asserting edge from %s to %s failed", from.c_str(),to.c_str());
    }

  } else {
    logger->log_warn(name(),"Did not get edge template, did you load hardware_models.clp?");
  }


}

void
HardwareModelsThread::clips_add_transaction(const std::string& component, const std::string& transaction) throw()
{
  for (const auto e : envs_) {
    fawkes::LockPtr<CLIPS::Environment> clips = e.second;
    clips.lock();
    CLIPS::Template::pointer temp = clips->get_template("hm-transition");
    if (temp) {
      CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
      fact->set_slot("component",component.c_str());
      fact->set_slot("transition",transaction.c_str());

      CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

      if (!new_fact) {
        logger->log_warn(name(), "Asserting transaction of %s: %s failed", component.c_str(),transaction.c_str());
      }

    } else {
      logger->log_warn(name(),"Did not get edge template, did you load hardware_models.clp?");
    }
    clips.unlock();
    logger->log_error(name(),"Added transaction in env: %s",e.first.c_str());
  }
  logger->log_error(name(),"Done");
}

void
HardwareModelsThread::finalize()
{
}


void
HardwareModelsThread::loop()
{

while ( hm_if_->msgq_empty() ) {
     usleep(100);
}

  hm_if_->read();
  while (!hm_if_->msgq_empty())
  {
    if (hm_if_->msgq_first_is<HardwareModelsInterface::StateChangeMessage>())
    {
      HardwareModelsInterface::StateChangeMessage *msg = hm_if_->msgq_first(msg);

      std::string comp = std::string(msg->component());
      std::string trans = std::string(msg->transaction());

      logger->log_error(name(),"Component: %s changed state by executing transaction: %s",comp.c_str(),trans.c_str());

      clips_add_transaction(comp.c_str(),trans.c_str());
    } else {
      logger->log_error(name(),"Recieved unknown message type");
    }
    hm_if_->msgq_pop();
  }
}


