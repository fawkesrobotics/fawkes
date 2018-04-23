
/***************************************************************************
 *  clips_executive_thread.cpp -  CLIPS executive
 *
 *  Created: Tue Sep 19 12:00:06 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include "clips_executive_thread.h"
#include "map_skill.h"

#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <interfaces/SwitchInterface.h>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ClipsExecutiveThread "clips_executive_thread.h"
 * Main thread of CLIPS-based executive.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsExecutiveThread::ClipsExecutiveThread()
	: Thread("ClipsExecutiveThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
	  CLIPSAspect("executive", "CLIPS (executive)")
{
}


/** Destructor. */
ClipsExecutiveThread::~ClipsExecutiveThread()
{
}


void
ClipsExecutiveThread::init()
{
	cfg_assert_time_each_loop_ = false;
	try {
		cfg_assert_time_each_loop_ = config->get_bool("/clips-executive/assert-time-each-loop");
	} catch (Exception &e) {} // ignore, use default

	std::vector<std::string> clips_dirs;
	try {
		clips_dirs = config->get_strings("/clips-executive/clips-dirs");
		for (size_t i = 0; i < clips_dirs.size(); ++i) {
			if (clips_dirs[i][clips_dirs[i].size()-1] != '/') {
				clips_dirs[i] += "/";
			}
			logger->log_debug(name(), "DIR: %s", clips_dirs[i].c_str());
		}
	} catch (Exception &e) {} // ignore, use default
	clips_dirs.insert(clips_dirs.begin(), std::string(SRCDIR) + "/clips/");

	try {
		std::string cfg_spec = config->get_string("/clips-executive/spec");

		std::string action_mapping_cfgpath = std::string("/clips-executive/specs/") + cfg_spec + "/action-mapping/";
#if __cplusplus >= 201103L
		std::unique_ptr<Configuration::ValueIterator> v(config->search(action_mapping_cfgpath.c_str()));
#else
		std::auto_ptr<Configuration::ValueIterator> v(config_->search(action_mapping_cfgpath.c_str()));
#endif
		std::map<std::string, std::string> mapping;
		while (v->next()) {
			std::string action_name = std::string(v->path()).substr(action_mapping_cfgpath.length());
			mapping[action_name] = v->get_as_string();
			logger->log_info(name(), "Adding action mapping '%s' -> '%s'",
			                 action_name.c_str(), v->get_as_string().c_str());
		}

		action_skill_mapping_ = std::make_shared<fawkes::ActionSkillMapping>(mapping);
	} catch (Exception &e) {} // ignore
	
	MutexLocker lock(clips.objmutex_ptr());

	clips->evaluate(std::string("(path-add-subst \"@BASEDIR@\" \"") + BASEDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@FAWKES_BASEDIR@\" \"") +
	                FAWKES_BASEDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@RESDIR@\" \"") + RESDIR + "\")");
	clips->evaluate(std::string("(path-add-subst \"@CONFDIR@\" \"") + CONFDIR + "\")");

	for (size_t i = 0; i < clips_dirs.size(); ++i) {
		clips->evaluate("(path-add \"" + clips_dirs[i] + "\")");
	}

	clips->evaluate("(ff-feature-request \"config\")");

	clips->add_function("map-action-skill",
	                    sigc::slot<std::string, std::string, CLIPS::Values, CLIPS::Values>
	                    (sigc::mem_fun(*this, &ClipsExecutiveThread::clips_map_skill)));

	bool cfg_req_redefwarn_feature = true;
	try {
		cfg_req_redefwarn_feature =
			config->get_bool("/clips-executive/request-redefine-warning-feature");
	} catch (Exception &e) {} // ignored, use default
	if (cfg_req_redefwarn_feature) {
		logger->log_debug(name(), "Enabling warnings for redefinitions");
		clips->evaluate("(ff-feature-request \"redefine-warning\")");
	}

  std::vector<std::string> files{SRCDIR "/clips/saliences.clp", SRCDIR "/clips/init.clp"};
	for (const auto f : files) {
		if (!clips->batch_evaluate(f)) {
		  logger->log_error(name(), "Failed to initialize CLIPS environment, "
			                  "batch file '%s' failed.", f.c_str());
			throw Exception("Failed to initialize CLIPS environment, batch file '%s' failed.",
			                f.c_str());
	  }
  }

	clips->assert_fact("(executive-init)");
	clips->refresh_agenda();
	clips->run();

	// Verify that initialization did not fail (yet)
	{
		CLIPS::Fact::pointer fact = clips->get_facts();
		while (fact) {
			CLIPS::Template::pointer tmpl = fact->get_template();
			if (tmpl->name() == "executive-init-stage") {
				CLIPS::Values v = fact->slot_value("");
				if (v.size() > 0 && v[0].as_string() == "FAILED") {
					throw Exception("CLIPS Executive initialization failed");
				}
			}

			fact = fact->next();
		}
	}
}


void
ClipsExecutiveThread::finalize()
{
	clips->assert_fact("(executive-finalize)");
	clips->refresh_agenda();
	clips->run();
}


void
ClipsExecutiveThread::loop()
{
	MutexLocker lock(clips.objmutex_ptr());

	// might be used to trigger loop events
	// must be cleaned up each loop from within the CLIPS code
	if (cfg_assert_time_each_loop_) {
		clips->assert_fact("(time (now))");
	}

	clips->refresh_agenda();
	clips->run();
}


std::string
ClipsExecutiveThread::clips_map_skill(std::string action_name, CLIPS::Values param_names, CLIPS::Values param_values)
{
	if (! action_skill_mapping_) {
		logger->log_error(name(), "No action mapping has been loaded");
		return "";
	}
	if (action_name == "") {
		logger->log_warn(name(), "Failed to map, action name is empty");
		return "";
	}
	if (! action_skill_mapping_->has_mapping(action_name)) {
		logger->log_warn(name(), "No mapping for action '%s' known", action_name.c_str());
		return "";
	}
	if (param_names.size() != param_values.size()) {
		logger->log_warn(name(), "Number of parameter names and values "
		                 "do not match for action '%s'", action_name.c_str());
		return "";
	}
	std::map<std::string, std::string> param_map;
	for (size_t i = 0; i < param_names.size(); ++i) {
		if (param_names[i].type() != CLIPS::TYPE_SYMBOL && param_names[i].type() != CLIPS::TYPE_STRING) {
			logger->log_error(name(), "Param for '%s' is not a string or symbol", action_name.c_str());
			return "";
		}
		switch (param_values[i].type()) {
		case CLIPS::TYPE_FLOAT:
			param_map[param_names[i].as_string()] = std::to_string(param_values[i].as_float());
			break;
		case CLIPS::TYPE_INTEGER:
			param_map[param_names[i].as_string()] = std::to_string(param_values[i].as_integer());
			break;
		case CLIPS::TYPE_SYMBOL:
		case CLIPS::TYPE_STRING:
			param_map[param_names[i].as_string()] = param_values[i].as_string();
			break;
		default:
			logger->log_error(name(), "Param '%s' for action '%s' of invalid type",
			                  param_names[i].as_string().c_str(), action_name.c_str());
			break;
		}
	}

	std::multimap<std::string, std::string> messages;
	std::string rv = action_skill_mapping_->map_skill(action_name, param_map, messages);
	for (auto &m : messages) {
		if (m.first == "WARN") {
			logger->log_warn(name(), "%s", m.second.c_str());
		} else if (m.first == "ERROR") {
			logger->log_error(name(), "%s", m.second.c_str());
		} else if (m.first == "DEBUG") {
			logger->log_debug(name(), "%s", m.second.c_str());
		} else {
			logger->log_info(name(), "%s", m.second.c_str());
		}
	}
	return rv;
}
