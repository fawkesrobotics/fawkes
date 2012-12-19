
/***************************************************************************
 *  clips_agent_thread.cpp -  CLIPS-based agent main thread
 *
 *  Created: Sat Jun 16 14:40:56 2012 (Mexico City)
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips_agent_thread.h"

#include <interfaces/SkillerInterface.h>
#include <interfaces/SwitchInterface.h>

using namespace fawkes;

/** @class ClipsAgentThread "clips_agent_thread.h"
 * Main thread of CLIPS-based agent.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsAgentThread::ClipsAgentThread()
  : Thread("ClipsAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
}


/** Destructor. */
ClipsAgentThread::~ClipsAgentThread()
{
}


void
ClipsAgentThread::init()
{
  cfg_clips_debug_ = false;
  cfg_skill_sim_time_ = 2.0;
  cfg_use_sim_ = false;

  try {
    cfg_clips_debug_ = config->get_bool("/clips-agent/clips-debug");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_use_sim_ = config->get_bool("/clips-agent/use-sim");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_skill_sim_time_ =
      config->get_float("/clips-agent/skill-sim-time");
  } catch (Exception &e) {} // ignore, use default

  cfg_clips_dir_ = std::string(SRCDIR) + "/clips/";

  skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

  if (! skiller_if_->has_writer()) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller has no writer, aborting");
    
  } else if (skiller_if_->exclusive_controller() != 0) {
    blackboard->close(skiller_if_);
    throw Exception("Skiller already has a different exclusive controller");
  }

  switch_if_ = blackboard->open_for_reading<SwitchInterface>("Clips Agent Start");

  clips->add_function("get-clips-dirs", sigc::slot<CLIPS::Values>(sigc::mem_fun(*this, &ClipsAgentThread::clips_get_clips_dirs)));
  clips->add_function("now", sigc::slot<CLIPS::Values>(sigc::mem_fun( *this, &ClipsAgentThread::clips_now)));
  clips->add_function("call-skill", sigc::slot<void, std::string, CLIPS::Values>(sigc::mem_fun( *this, &ClipsAgentThread::clips_call_skill)));
  clips->add_function("load-config", sigc::slot<void, std::string>(sigc::mem_fun( *this, &ClipsAgentThread::clips_load_config)));

  if (!clips->batch_evaluate(cfg_clips_dir_ + "init.clp")) {
    logger->log_error(name(), "Failed to initialize CLIPS environment, "
                      "batch file failed.");
    blackboard->close(skiller_if_);
    throw Exception("Failed to initialize CLIPS environment, batch file failed.");
  }

  if (cfg_clips_debug_) {
    clips->assert_fact("(enable-debug)");
  }

  if (cfg_use_sim_) {
    clips->assert_fact("(enable-sim)");
  } else {
    clips->assert_fact("(enable-skills)");
  }

  clips->assert_fact("(init)");
  clips->refresh_agenda();
  clips->run();

  ctrl_recheck_ = true;

  started_ = false;
  skill_started_ = false;
  skill_start_time_ = new Time(clock);
}


void
ClipsAgentThread::finalize()
{
  delete skill_start_time_;

  if (skiller_if_->has_writer()) {
    SkillerInterface::ReleaseControlMessage *msg =
      new SkillerInterface::ReleaseControlMessage();
    skiller_if_->msgq_enqueue(msg);
  }

  blackboard->close(skiller_if_);
  blackboard->close(switch_if_);
}


void
ClipsAgentThread::loop()
{
  skiller_if_->read();

  if ((skiller_if_->exclusive_controller() == 0) && skiller_if_->has_writer())
  {
    if (ctrl_recheck_) {
      logger->log_info(name(), "Acquiring exclusive skiller control");
      SkillerInterface::AcquireControlMessage *msg =
        new SkillerInterface::AcquireControlMessage();
      skiller_if_->msgq_enqueue(msg);
      ctrl_recheck_ = false;
    } else {
      ctrl_recheck_ = true;
    }
    return;
  }

  if (! started_) {
    switch_if_->read();
    if (switch_if_->is_enabled()) {
      clips->assert_fact("(start)");
      started_ = true;
    }
  }

  // might be used to trigger loop events
  // must be cleaned up each loop from within the CLIPS code
  //clips->assert_fact("(time (now))");


  Time now(clock);
  if (! active_skills_.empty()) {
    std::list<std::string> finished_skills;
    std::map<std::string, SkillExecInfo>::iterator as;
    for (as = active_skills_.begin(); as != active_skills_.end(); ++as) {
      const std::string   &as_name = as->first;
      const SkillExecInfo &as_info = as->second;

      if ((now - as_info.start_time) >= cfg_skill_sim_time_) {
	logger->log_warn(name(), "Skill '%s' is final", as_name.c_str());
	clips->assert_fact_f("(skill-finished (name %s) (status FINAL))", as_name.c_str());
	finished_skills.push_back(as_name);
      }
    }

    std::list<std::string>::iterator fs;
    for (fs = finished_skills.begin(); fs != finished_skills.end(); ++fs) {
      active_skills_.erase(*fs);
    }
  }

  clips->refresh_agenda();
  clips->run();
}


CLIPS::Values
ClipsAgentThread::clips_now()
{
  CLIPS::Values rv;
  fawkes::Time now(clock);
  rv.push_back(now.get_sec());
  rv.push_back(now.get_usec());
  return rv;
}


CLIPS::Values
ClipsAgentThread::clips_get_clips_dirs()
{
  CLIPS::Values rv;
  rv.push_back(cfg_clips_dir_);
  return rv;
}


void
ClipsAgentThread::clips_call_skill(std::string skill_name, CLIPS::Values args)
{
  logger->log_info(name(), "Call skill %s", skill_name.c_str());
}


void
ClipsAgentThread::clips_load_config(std::string cfg_prefix)
{
  std::auto_ptr<Configuration::ValueIterator> v(config->search(cfg_prefix.c_str()));
  while (v->next()) {
    std::string type = "";
    std::string value = v->get_as_string();

    if (v->is_float())       type = "FLOAT";
    else if (v->is_uint())   type = "UINT";
    else if (v->is_int())    type = "INT";
    else if (v->is_bool())   type = "BOOL";
    else if (v->is_string()) {
      type = "STRING";
      value = std::string("\"") + value + "\"";
    } else {
      logger->log_warn(name(), "Config value at '%s' of unknown type '%s'",
		       v->path(), v->type());
    }

    //logger->log_info(name(), "ASSERT (confval (path \"%s\") (type %s) (value %s)",
    //		     v->path(), type.c_str(), v->get_as_string().c_str());
    clips->assert_fact_f("(confval (path \"%s\") (type %s) (value %s))",
			 v->path(), type.c_str(), value.c_str());
  }
}
