
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

#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <utils/graph/topological_map_graph.h>
#include <utils/graph/yaml_navgraph.h>
#include <interfaces/SwitchInterface.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ClipsAgentThread "clips_agent_thread.h"
 * Main thread of CLIPS-based agent.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsAgentThread::ClipsAgentThread()
  : Thread("ClipsAgentThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    CLIPSAspect("agent", "CLIPS (agent)")
{
}


/** Destructor. */
ClipsAgentThread::~ClipsAgentThread()
{
}


void
ClipsAgentThread::init()
{
  skiller_if_ = NULL;

  cfg_auto_start_ = false;
  cfg_assert_time_each_loop_ = false;
  cfg_skill_sim_time_ = 2.0;
  cfg_skill_sim_ = false;
  cfg_steal_skiller_control_ = true;

  try {
    cfg_auto_start_ = config->get_bool("/clips-agent/auto-start");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_assert_time_each_loop_ = config->get_bool("/clips-agent/assert-time-each-loop");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_skill_sim_ = config->get_bool("/clips-agent/skill-sim");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_skill_sim_time_ =
      config->get_float("/clips-agent/skill-sim-time");
  } catch (Exception &e) {} // ignore, use default
  try {
    cfg_steal_skiller_control_ = config->get_bool("/clips-agent/steal-skiller-control");
  } catch (Exception &e) {} // ignore, use default

  cfg_clips_dirs_.clear();
  try {
    cfg_clips_dirs_ = config->get_strings("/clips-agent/clips-dirs");
    for (size_t i = 0; i < cfg_clips_dirs_.size(); ++i) {
      std::string::size_type pos;
      if ((pos = cfg_clips_dirs_[i].find("@BASEDIR@")) != std::string::npos) {
	cfg_clips_dirs_[i].replace(pos, 9, BASEDIR);
      }
      if ((pos = cfg_clips_dirs_[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
	cfg_clips_dirs_[i].replace(pos, 16, FAWKES_BASEDIR);
      }
      if ((pos = cfg_clips_dirs_[i].find("@RESDIR@")) != std::string::npos) {
	cfg_clips_dirs_[i].replace(pos, 8, RESDIR);
      }
      if ((pos = cfg_clips_dirs_[i].find("@CONFDIR@")) != std::string::npos) {
	cfg_clips_dirs_[i].replace(pos, 9, CONFDIR);
      }
      if (cfg_clips_dirs_[i][cfg_clips_dirs_.size()-1] != '/') {
	cfg_clips_dirs_[i] += "/";
      }
      logger->log_warn(name(), "DIR: %s", cfg_clips_dirs_[i].c_str());
    }
  } catch (Exception &e) {} // ignore, use default

  cfg_clips_dirs_.insert(cfg_clips_dirs_.begin(), std::string(SRCDIR) + "/clips/");

  if (! cfg_skill_sim_) {
    skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

    if (! skiller_if_->has_writer()) {
      blackboard->close(skiller_if_);
      throw Exception("Skiller has no writer, aborting");
      
    } else if (skiller_if_->exclusive_controller() != 0) {
      blackboard->close(skiller_if_);
      throw Exception("Skiller already has a different exclusive controller");
    }
  }

  switch_if_ = blackboard->open_for_reading<SwitchInterface>("Clips Agent Start");

  clips->add_function("get-clips-dirs", sigc::slot<CLIPS::Values>(sigc::mem_fun(*this, &ClipsAgentThread::clips_get_clips_dirs)));
  clips->add_function("skill-call-ext", sigc::slot<void, std::string, std::string>(sigc::mem_fun( *this, &ClipsAgentThread::clips_skill_call_ext)));
  clips->add_function("load-config", sigc::slot<void, std::string>(sigc::mem_fun( *this, &ClipsAgentThread::clips_load_config)));
  clips->add_function("navgraph-load", sigc::slot<CLIPS::Value, std::string>(sigc::mem_fun( *this, &ClipsAgentThread::clips_navgraph_load)));

  clips->load(SRCDIR"/clips/navgraph.clp");

  if (!clips->batch_evaluate(SRCDIR"/clips/init.clp")) {
    logger->log_error(name(), "Failed to initialize CLIPS environment, "
                      "batch file failed.");
    blackboard->close(skiller_if_);
    throw Exception("Failed to initialize CLIPS environment, batch file failed.");
  }

  clips->assert_fact("(agent-init)");
  clips->refresh_agenda();
  clips->run();

  ctrl_recheck_ = true;
  started_ = false;
}


void
ClipsAgentThread::finalize()
{
  MutexLocker lock(clips.objmutex_ptr());

  clips->remove_function("get-clips-dirs");
  clips->remove_function("now");
  clips->remove_function("skill-call-ext");
  clips->remove_function("load-config");
  clips->remove_function("navgraph-load");

  if ( ! cfg_skill_sim_ && skiller_if_->has_writer()) {
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
  MutexLocker lock(clips.objmutex_ptr());

  if (! started_ && cfg_auto_start_) {
    clips->assert_fact("(start)");
    started_ = true;
  }

  if (! cfg_skill_sim_) {
    skiller_if_->read();

    if ((skiller_if_->exclusive_controller() == 0) && skiller_if_->has_writer())
    {
      if (ctrl_recheck_) {
	logger->log_info(name(), "Acquiring exclusive skiller control");
	SkillerInterface::AcquireControlMessage *msg =
	  new SkillerInterface::AcquireControlMessage(cfg_steal_skiller_control_);
	skiller_if_->msgq_enqueue(msg);
	ctrl_recheck_ = false;
      } else {
	ctrl_recheck_ = true;
      }
      return;
    }
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
  if (cfg_assert_time_each_loop_) {
    clips->assert_fact("(time (now))");
  }


  Time now(clock);
  if (! active_skills_.empty()) {
    if (! cfg_skill_sim_)  skiller_if_->read();

    std::list<std::string> finished_skills;
    std::map<std::string, SkillExecInfo>::iterator as;
    for (as = active_skills_.begin(); as != active_skills_.end(); ++as) {
      const std::string   &as_name = as->first;
      const SkillExecInfo &as_info = as->second;

      if (cfg_skill_sim_) {
	if ((now - as_info.start_time) >= cfg_skill_sim_time_) {
	  logger->log_warn(name(), "Simulated skill '%s' final", as_name.c_str());
	  clips->assert_fact_f("(skill-update (name \"%s\") (status FINAL))",
			       as_name.c_str());
	  finished_skills.push_back(as_name);
	} else {
	  clips->assert_fact_f("(skill-update (name \"%s\") (status RUNNING))",
			       as_name.c_str());
	}
      } else {
	if (as_info.skill_string == skiller_if_->skill_string()) {
	  clips->assert_fact_f("(skill-update (name \"%s\") (status %s))", as_name.c_str(),
			       status_string(skiller_if_->status()));
	}
	if (skiller_if_->status() == SkillerInterface::S_FINAL ||
	    skiller_if_->status() == SkillerInterface::S_FAILED)
	{
	  finished_skills.push_back(as_name);
	}
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


const char *
ClipsAgentThread::status_string(SkillerInterface::SkillStatusEnum status)
{
  switch (status) {
  case SkillerInterface::S_FINAL:   return "FINAL";
  case SkillerInterface::S_FAILED:  return "FAILED";
  case SkillerInterface::S_RUNNING: return "RUNNING";
  default: return "IDLE";
  }
}


CLIPS::Values
ClipsAgentThread::clips_get_clips_dirs()
{
  CLIPS::Values rv(cfg_clips_dirs_.size(), CLIPS::Value(std::string("")));
  for (size_t i = 0; i < cfg_clips_dirs_.size(); ++i) {
    rv[i] = cfg_clips_dirs_[i];
  }
  return rv;
}


void
ClipsAgentThread::clips_skill_call_ext(std::string skill_name, std::string skill_string)
{
  if (active_skills_.find(skill_name) != active_skills_.end()) {
    logger->log_warn(name(), "Skill %s called again while already active",
		     skill_name.c_str());
  }

  if (cfg_skill_sim_) {
    logger->log_info(name(), "Simulating skill %s", skill_string.c_str());

  } else {
    logger->log_info(name(), "Calling skill %s", skill_string.c_str());

    SkillerInterface::ExecSkillContinuousMessage *msg =
      new SkillerInterface::ExecSkillContinuousMessage(skill_string.c_str());
      
    skiller_if_->msgq_enqueue(msg);
  }

  SkillExecInfo sei;
  sei.start_time   = clock->now();
  sei.skill_string = skill_string;
  active_skills_[skill_name] = sei;
}


void
ClipsAgentThread::clips_load_config(std::string cfg_prefix)
{
  std::auto_ptr<Configuration::ValueIterator> v(config->search(cfg_prefix.c_str()));
  while (v->next()) {
    std::string type = "";
    std::string value = v->get_as_string();

    if      (v->is_uint())   type = "UINT";
    else if (v->is_int())    type = "INT";
    else if (v->is_float())  type = "FLOAT";
    else if (v->is_bool())   type = "BOOL";
    else if (v->is_string()) {
      type = "STRING";
      if (! v->is_list()) {
	value = std::string("\"") + value + "\"";
      }
    } else {
      logger->log_warn(name(), "Config value at '%s' of unknown type '%s'",
		       v->path(), v->type());
    }

    if (v->is_list()) {
      logger->log_info(name(), "(confval (path \"%s\") (type %s) (is-list TRUE) (list-value %s))",
		       v->path(), type.c_str(), value.c_str());
      clips->assert_fact_f("(confval (path \"%s\") (type %s) (is-list TRUE) (list-value %s))",
			    v->path(), type.c_str(), value.c_str());
    } else {
      //logger_->log_info(name(), "(confval (path \"%s\") (type %s) (value %s))",
      //       v->path(), type.c_str(), value.c_str());
      clips->assert_fact_f("(confval (path \"%s\") (type %s) (value %s))",
			    v->path(), type.c_str(), value.c_str());
    }

  }
}


CLIPS::Value
ClipsAgentThread::clips_navgraph_load(std::string filename)
{
  if (filename.empty()) {
    logger->log_warn(name(), "Cannot load navgraph (file name is empty)");
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
  if (filename[0] != '/') {
    filename = std::string(CONFDIR) + "/" + filename;
  }

  try {
    TopologicalMapGraph *graph = load_yaml_navgraph(filename);
    TopologicalMapNode root_node                 = graph->root_node();
    const std::vector<TopologicalMapNode> &nodes = graph->nodes();
    const std::vector<TopologicalMapEdge> &edges = graph->edges();

    clips->assert_fact_f("(navgraph (name \"%s\") (root \"%s\"))",
			 graph->name().c_str(), root_node.name().c_str());

    for (auto n : nodes) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = n.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-node (name \"%s\") (pos %f %f) (properties %s))",
			   n.name().c_str(), n.x(), n.y(), props_string.c_str());
    }

    for (auto e : edges) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = e.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-edge (from \"%s\") (to \"%s\") (directed %s) "
			   "(properties \"%s\"))",
			   e.from().c_str(), e.to().c_str(),
			   e.is_directed() ? "TRUE" : "FALSE", props_string.c_str());
    }

    return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);

  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to load navgraph %s, exception follows", filename.c_str());
    logger->log_warn(name(), e);
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
}
