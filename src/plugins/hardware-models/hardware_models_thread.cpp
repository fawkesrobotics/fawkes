
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
 * Parses yaml files that are assumend to have the following format:
 * COMPONENT-NAME:
 *  states:
 *      <List of states, the first state is assumed to be the initial state>
 * 
 *  STATE_1:
 *   edges:
 *    <List of states the state has an edge to>
 *   edge_1:
 *    transition: <name of the action that causes the transition>
 *    (optional)probability: <probability of the transition if it is an exogenous action>
 *     
 *  Any component plugin can inform the HardwareModel Plugin about state changes
 *  by sending a HardwareModelInterfaceMessage.
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
  wakeup();
  bbil_add_message_interface(hm_if_);
}

/**
 * @brief Initializes hardware components from yaml files for a clips environment
 * 
 * @param env_name Name of clips environment
 * @param clips Pointer to clips environment
 */
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

    //First state in the config file is assumend to be the initial state
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
                logger->log_warn(name(),"Cant find transition value in %s",std::string(c+"/"+state+"/"+edge).c_str());
                continue;
              }

              double prob = 0.0;
              if (config->exists(std::string(c + "/" + state + "/" + edge + "/probability").c_str())) {
                prob = config->get_float(std::string(c + "/" + state + "/" + edge + "/probability").c_str());
              }

              clips_add_edge(clips,c,state,edge,transition,prob);
              logger->log_debug(name(),"Edge from %s to %s via %s",state.c_str(),edge.c_str(),transition.c_str());
          }
        } else {
          logger->log_warn(name(),"State %s of %s needs edges field",state.c_str(),c.c_str());
          continue;
        }

        logger->log_debug(name(),state.c_str());
      }
      clips_add_component(clips,c,states[0]);
    } else {
      logger->log_warn(name(),"No states for component %s in %s",c.c_str(),models_dir_.c_str());
    }
  }
  hm_if_->set_busy(false);
  hm_if_->write();
}

void
HardwareModelsThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_error(name(), "Removing environment %s", env_name.c_str());
}

/**
 * @brief Adds a hardware component fact to the given clips environment
 * 
 * @param clips Pointer to clips environment
 * @param component Name of the hardware component
 * @param init_state Name of the initial state of the component
 */
void
HardwareModelsThread::clips_add_component(LockPtr<CLIPS::Environment> &clips,const std::string& component,const std::string& init_state)
{
  CLIPS::Template::pointer temp = clips->get_template("hm-component");
  if (temp) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("name",CLIPS::Value(component.c_str(),CLIPS::TYPE_SYMBOL));
    fact->set_slot("state",CLIPS::Value(init_state.c_str(),CLIPS::TYPE_SYMBOL));

    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn(name(), "Asserting component %s failed", component.c_str());
    }

  } else {
    logger->log_warn(name(),"Did not get component template, did you load hardware_models.clp?");
  }
}

/**
 * @brief Adds a hardware component edge fact to the given clips environment
 * 
 * @param clips Pointer to the clips environment
 * @param component Name of the component the edge belongs to
 * @param from Name of the origin state
 * @param to Name of the destination state
 * @param trans Label/Transition action that triggers the edge
 * @param prob Probability of the action if it is an exogenous action
 */
void
HardwareModelsThread::clips_add_edge(LockPtr<CLIPS::Environment> &clips,const std::string& component, 
                                                  const std::string& from, 
                                                  const std::string& to, 
                                                  const std::string& trans, 
                                                  const double prob)
{
  CLIPS::Template::pointer temp = clips->get_template("hm-edge");
  if (temp) {
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
    fact->set_slot("component",CLIPS::Value(component.c_str(),CLIPS::TYPE_SYMBOL));
    fact->set_slot("from",CLIPS::Value(from.c_str(),CLIPS::TYPE_SYMBOL));
    fact->set_slot("to",CLIPS::Value(to.c_str(),CLIPS::TYPE_SYMBOL));
    fact->set_slot("transition",CLIPS::Value(trans.c_str(),CLIPS::TYPE_SYMBOL));
    fact->set_slot("probability",prob);

    CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

    if (!new_fact) {
      logger->log_warn(name(), "Asserting edge from %s to %s failed", from.c_str(),to.c_str());
    }

  } else {
    logger->log_warn(name(),"Did not get edge template, did you load hardware_models.clp?");
  }


}

/**
 * @brief Adds a transition fact to all registered clips environments. This represents that
 *        the given component changed its state by executing the transition.
 * 
 * @param component Name of the component that executed the transition
 * @param transition Name of the transition action
 */
void
HardwareModelsThread::clips_add_transition(const std::string& component, const std::string& transition) throw()
{
  for (const auto e : envs_) {
    fawkes::LockPtr<CLIPS::Environment> clips = e.second;
    clips.lock();
    CLIPS::Template::pointer temp = clips->get_template("hm-transition");
    if (temp) {
      CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
      fact->set_slot("component",component.c_str());
      fact->set_slot("transition",transition.c_str());

      CLIPS::Fact::pointer new_fact = clips->assert_fact(fact);

      if (!new_fact) {
        logger->log_warn(name(), "Asserting transition of %s: %s failed", component.c_str(),transition.c_str());
      }

    } else {
      logger->log_warn(name(),"Did not get edge template, did you load hardware_models.clp?");
    }
    clips.unlock();
    logger->log_error(name(),"Added transition in env: %s",e.first.c_str());
  }
  logger->log_error(name(),"Done");
}

void
HardwareModelsThread::finalize()
{
}

/**
 * @brief Loop function to be executed when a new HardwareInterfaceMessage is recieved
 * 
 */
void
HardwareModelsThread::loop()
{
  hm_if_->read();
  while (!hm_if_->msgq_empty())
  {
    if (hm_if_->msgq_first_is<HardwareModelsInterface::StateChangeMessage>())
    {
      HardwareModelsInterface::StateChangeMessage *msg = hm_if_->msgq_first(msg);

      std::string comp = std::string(msg->component());
      std::string trans = std::string(msg->transition());

      logger->log_error(name(),"Component: %s changed state by executing transition: %s",comp.c_str(),trans.c_str());

      clips_add_transition(comp.c_str(),trans.c_str());
    } else {
      logger->log_error(name(),"Recieved unknown message type");
    }
    hm_if_->msgq_pop();
  }
}


