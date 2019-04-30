
/***************************************************************************
 *  stn-generator_thread.cpp - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#include "stn-generator_thread.h"

#include <utils/misc/string_conversions.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <chrono>
#include <fstream>
#include <mongocxx/client.hpp>
#include <streambuf>
#include <thread>

using namespace fawkes;
using namespace mongocxx;
using namespace bsoncxx;

/** @class StnGeneratorThread 'stn-generator_thread.h' 
 * Generates an STN representation of a sequential task plan
 * @author Matthias Loebach
 */

/** Constructor. */
StnGeneratorThread::StnGeneratorThread()
: Thread("StnGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("StnGeneratorThread")
{
}

void
StnGeneratorThread::init()
{
	logger->log_info(name(), "reading config");
	std::string cfg_prefix       = "plugins/stn-generator/";
	cfg_plan_collection_         = config->get_string(cfg_prefix + "plan/collection");
	cfg_output_collection_       = config->get_string(cfg_prefix + "output/collection");
	cfg_publish_to_robot_memory_ = config->get_bool(cfg_prefix + "output/publish-to-rm");
	cfg_draw_graph_              = config->get_bool(cfg_prefix + "output/draw-graph");

	std::string pddl_domain_path =
	  StringConversions::resolve_path(config->get_string(cfg_prefix + "domain-file"));
	cfg_pddl_problem_path_ =
	  StringConversions::resolve_path(config->get_string(cfg_prefix + "problem-file"));

	std::ifstream s(pddl_domain_path);
	if (!s.good()) {
		logger->log_error(name(), "Could not open domain-file at %s", pddl_domain_path.c_str());
	}
	std::string pddl_domain;

	s.seekg(0, std::ios::end);
	pddl_domain.reserve(s.tellg());
	s.seekg(0, std::ios::beg);
	pddl_domain.assign((std::istreambuf_iterator<char>(s)), std::istreambuf_iterator<char>());

	if (config->get_bool(cfg_prefix + "generate-classic-domain")) {
		std::string classic_dom_path =
		  StringConversions::resolve_path(config->get_string(cfg_prefix + "classic-domain-file"));
		stn_ = new stn::Stn(logger, classic_dom_path);
	} else {
		stn_ = new stn::Stn(logger);
	}
	stn_->set_pddl_domain(pddl_domain);
	logger->log_info(name(), "Created STN object from domain");

	plan_if_ = blackboard->open_for_reading<PddlPlannerInterface>(
	  config->get_string(cfg_prefix + "plan/interface").c_str());
	bbil_add_data_interface(plan_if_);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
}

void
StnGeneratorThread::loop()
{
	std::ifstream s(cfg_pddl_problem_path_);
	if (!s.good()) {
		logger->log_error(name(), "Could not open problem-file at %s", cfg_pddl_problem_path_.c_str());
	}
	std::string pddl_problem;
	s.seekg(0, std::ios::end);
	pddl_problem.reserve(s.tellg());
	s.seekg(0, std::ios::beg);
	pddl_problem.assign((std::istreambuf_iterator<char>(s)), std::istreambuf_iterator<char>());
	stn_->read_initial_state(pddl_problem);

	auto cursor = robot_memory->query(from_json("{plan:1}"), cfg_plan_collection_);
	for (auto doc : cursor) {
		array::view actions = doc["actions"].get_array();
		for (auto &a : actions) {
			std::string args;
			bool        first      = true;
			array::view args_array = a["args"].get_array();
			for (auto &arg : args_array) {
				if (!first) {
					args += " ";
				}
				first = false;
				args += arg.get_utf8().value.to_string();
			}
			std::string action_name = a["name"].get_utf8().value.to_string();
			stn_->add_plan_action(action_name, args);
			logger->log_debug(name(), "Added Plan action %s to STN", action_name.c_str());
		}
	}
	stn_->generate();
	if (cfg_draw_graph_) {
		try {
			stn_->drawGraph();
		} catch (std::out_of_range &e) {
			logger->log_warn(name(), "Failed to draw graph: %s", e.what());
		}
	}
	logger->log_info(name(), "STN Generation finished.");

	using namespace bsoncxx::builder;
	if (cfg_publish_to_robot_memory_) {
		//TODO reset actions in robot-memory
		for (auto &action : stn_->get_bson()) {
			basic::document rm_action;
			rm_action.append(basic::kvp("relation", "proposed-stn-action"));
			rm_action.append(bsoncxx::builder::concatenate(action.view()));
			robot_memory->insert(rm_action.view(), cfg_output_collection_);
		}
		// ensure all actions are written to RM before acknowledment
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		num_published_actions_ += stn_->get_bson().size();
		basic::document rm_final;
		rm_final.append(basic::kvp("relation", "stn-sync"));
		rm_final.append(basic::kvp("state", "synced"));
		rm_final.append(basic::kvp("count", std::to_string(num_published_actions_)));
		robot_memory->insert(rm_final.view(), cfg_output_collection_);
	}
}

void
StnGeneratorThread::finalize()
{
	delete stn_;
}

void
StnGeneratorThread::bb_interface_data_changed(Interface *interface) throw()
{
	if (interface->uid() == plan_if_->uid()) {
		plan_if_->read();
		if (plan_if_->is_final()) {
			logger->log_info(name(), "Planning is final, starting STN generation");
			wakeup();
		}
	} else {
		logger->log_error(name(), "Received data change for wrong interface");
	}
}
