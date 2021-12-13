
/***************************************************************************
 *  pddl-planner_thread.cpp - pddl-planner
 *
 *  Created: Wed Dec  7 19:09:44 2016
 *  Copyright  2016  Frederik Zwilling
 *             2017  Matthias Loebach
 *             2017  Till Hofmann
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

#include "pddl-planner_thread.h"

#include <utils/misc/string_conversions.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/json.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

using namespace fawkes;
using namespace mongocxx;
using namespace bsoncxx;
using bsoncxx::builder::basic::kvp;

/** @class PddlPlannerThread 'pddl-planner_thread.h'
 * Starts a pddl planner and writes the resulting plan into the robot memory
 * @author Frederik Zwilling
 */

/** Constructor. */
PddlPlannerThread::PddlPlannerThread()
: Thread("PddlPlannerThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("PddlPlannerThread")
{
}

void
PddlPlannerThread::init()
{
	//read config
	std::string cfg_prefix = "plugins/pddl-planner/";
	cfg_descripton_path_ =
	  StringConversions::resolve_path(config->get_string((cfg_prefix + "description-folder")));
	cfg_result_path_  = cfg_descripton_path_ + config->get_string((cfg_prefix + "result-file"));
	cfg_domain_path_  = cfg_descripton_path_ + config->get_string(cfg_prefix + "domain-description");
	cfg_problem_path_ = cfg_descripton_path_ + config->get_string(cfg_prefix + "problem-description");
	cfg_fd_options_   = config->get_string(cfg_prefix + "fd-search-opts");
	cfg_collection_   = config->get_string(cfg_prefix + "collection");

	//set configured planner
	std::string planner_string = config->get_string((cfg_prefix + "planner").c_str());
	if (planner_string == "ff") {
		planner_ = std::bind(&PddlPlannerThread::ff_planner, this);
		logger->log_info(name(), "Fast-Forward planner selected.");
	} else if (planner_string == "fd") {
		planner_ = std::bind(&PddlPlannerThread::fd_planner, this);
		logger->log_info(name(), "Fast-Downward planner selected.");
	} else if (planner_string == "dbmp") {
		planner_ = std::bind(&PddlPlannerThread::dbmp_planner, this);
		logger->log_info(name(), "DBMP selected.");
	} else if (planner_string == "popf") {
		planner_ = std::bind(&PddlPlannerThread::popf_planner, this);
		logger->log_info(name(), "POPF selected.");
	} else {
		planner_ = std::bind(&PddlPlannerThread::ff_planner, this);
		logger->log_warn(name(), "No planner configured.\nDefaulting to ff.");
	}

	//setup interface
	plan_if_ = blackboard->open_for_writing<PddlPlannerInterface>(
	  config->get_string(cfg_prefix + "interface-name").c_str());
	plan_if_->set_active_planner(planner_string.c_str());
	plan_if_->set_msg_id(0);
	plan_if_->set_final(false);
	plan_if_->set_success(false);
	plan_if_->write();

	//setup interface listener
	bbil_add_message_interface(plan_if_);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

	// If we receive multiple wakeup() calls during loop, only call the loop once afterwards
	// We want to only re-plan once after a loop run since multiple runs would plan on the same problem
	this->set_coalesce_wakeups(true);
}

/**
 * Thread is only waked up if there was a new interface message to plan
 */
void
PddlPlannerThread::loop()
{
	logger->log_info(name(), "Starting PDDL Planning...");

	//writes plan into action_list_
	planner_();

	if (!action_list_.empty()) {
		auto plan = BSONFromActionList();
		robot_memory->update(builder::basic::make_document(kvp("plan", "{$exists:true}")),
		                     plan,
		                     cfg_collection_,
		                     true);
		print_action_list();
		plan_if_->set_success(true);
	} else {
		logger->log_error(name(), "Updating plan failed, action list empty!");
		robot_memory->update(builder::basic::make_document(kvp("plan", "{$exists:true}")),
		                     builder::basic::make_document(kvp("plan", 0)),
		                     cfg_collection_,
		                     true);
		plan_if_->set_success(false);
	}

	plan_if_->set_final(true);
	plan_if_->write();
}

void
PddlPlannerThread::finalize()
{
	blackboard->close(plan_if_);
}

void
PddlPlannerThread::ff_planner()
{
	logger->log_info(name(), "Starting PDDL Planning with Fast-Forward...");

	std::string command = "ff -o " + cfg_domain_path_ + " -f " + cfg_problem_path_;
	logger->log_info(name(), "Calling %s", command.c_str());
	std::string result = run_planner(command);

	//Parse Result and write it into the robot memory
	logger->log_info(name(), "Parsing result");

	action_list_.clear();

	size_t cur_pos = 0;
	if (result.find("found legal plan as follows", cur_pos) == std::string::npos) {
		logger->log_error(name(), "Planning Failed: %s", result.c_str());
		robot_memory->update(builder::basic::make_document(kvp("plan", "{$exists:true}")),
		                     builder::basic::make_document(kvp("plan", 1),
		                                                   kvp("fail", 1),
		                                                   kvp("steps", builder::basic::array())),
		                     cfg_collection_,
		                     true);
		return;
	}
	//remove stuff that could confuse us later
	result.erase(result.find("time spent:", cur_pos));

	cur_pos = result.find("step", cur_pos) + 4;
	while (result.find(": ", cur_pos) != std::string::npos) {
		cur_pos         = result.find(": ", cur_pos) + 2;
		size_t line_end = result.find("\n", cur_pos);
		logger->log_info(name(),
		                 "line:%s (%zu-%zu)",
		                 result.substr(cur_pos, line_end - cur_pos).c_str(),
		                 cur_pos,
		                 line_end);
		action a;
		if (line_end < result.find(" ", cur_pos)) {
			a.name = result.substr(cur_pos, line_end - cur_pos);
		} else {
			size_t action_end = result.find(" ", cur_pos);
			a.name            = StringConversions::to_lower(result.substr(cur_pos, action_end - cur_pos));
			cur_pos           = action_end + 1;
			while (cur_pos < line_end) {
				size_t arg_end = result.find(" ", cur_pos);
				if (arg_end > line_end) {
					arg_end = line_end;
				}
				a.args.push_back(result.substr(cur_pos, arg_end - cur_pos));
				cur_pos = arg_end + 1;
			}
		}
		action_list_.push_back(a);
	}
}

void
PddlPlannerThread::dbmp_planner()
{
	logger->log_info(name(), "Starting PDDL Planning with DBMP...");

	std::string command =
	  "dbmp.py -p ff --output plan.pddl " + cfg_domain_path_ + " " + cfg_problem_path_;
	logger->log_info(name(), "Calling %s", command.c_str());
	std::string result = run_planner(command);

	//Parse Result and write it into the robot memory
	logger->log_info(name(), "Parsing result");

	size_t cur_pos = 0;
	if (result.find("Planner failed", cur_pos) != std::string::npos) {
		logger->log_error(name(), "Planning Failed: %s", result.c_str());
		robot_memory->update(builder::basic::make_document(kvp("plan", "{$exists:true}")),
		                     builder::basic::make_document(kvp("plan", 1),
		                                                   kvp("fail", 1),
		                                                   kvp("steps", builder::basic::array())),
		                     cfg_collection_,
		                     true);
		return;
	}
	std::ifstream planfile("plan.pddl");
	std::string   line;
	action_list_.clear();
	while (std::getline(planfile, line)) {
		std::string time_string = "Time";
		if (line.compare(0, time_string.size(), time_string) == 0) {
			// makespan, skip
			continue;
		}
		if (line[0] != '(' || line[line.size() - 1] != ')') {
			logger->log_error(name(), "Expected parantheses in line '%s'!", line.c_str());
			return;
		}
		// remove parantheses
		std::string action_str = line.substr(1, line.size() - 2);
		action      a;
		cur_pos = action_str.find(" ", cur_pos + 1);
		a.name  = StringConversions::to_lower(action_str.substr(0, cur_pos));
		while (cur_pos != std::string::npos) {
			size_t word_start = cur_pos + 1;
			cur_pos           = action_str.find(" ", word_start);
			a.args.push_back(action_str.substr(word_start, cur_pos - word_start));
		}
		action_list_.push_back(a);
	}
}

void
PddlPlannerThread::fd_planner()
{
	logger->log_info(name(), "Starting PDDL Planning with Fast-Downward...");

	std::string command =
	  "fast-downward" + std::string(" ") + cfg_domain_path_ + std::string(" ") + cfg_problem_path_;

	if (!cfg_fd_options_.empty()) {
		command += std::string(" ") + cfg_fd_options_;
	}

	std::string result = run_planner(command);

	logger->log_info(name(), "Removing temporary planner output.");
	std::remove("output");
	std::remove("output.sas");

	size_t cur_pos = 0;
	if (result.find("Solution found!", cur_pos) == std::string::npos) {
		logger->log_error(name(), "Planning Failed: %s", result.c_str());
		throw Exception("No solution found");
	} else {
		cur_pos = result.find("Solution found!", cur_pos);
		cur_pos = result.find("\n", cur_pos);
		cur_pos = result.find("\n", cur_pos + 1);
		logger->log_info(name(), "Planner found solution.");
	}
	result.erase(0, cur_pos);
	size_t end_pos = result.find("Plan length: ");
	result.erase(end_pos, result.size() - 1);

	std::istringstream iss(result);
	std::string        line;
	// remove surplus line
	getline(iss, line);
	while (getline(iss, line)) {
		action a;
		a.name = line.substr(0, find_nth_space(line, 1));
		if (find_nth_space(line, 2) != line.rfind(' ') + 1) {
			std::stringstream ss(
			  line.substr(find_nth_space(line, 2), line.rfind(' ') - find_nth_space(line, 2)));
			std::string item;
			while (getline(ss, item, ' ')) {
				a.args.push_back(item);
			}
		}
		action_list_.push_back(a);
	}
}

void
PddlPlannerThread::popf_planner()
{
	logger->log_info(name(), "Starting PDDL Planning with POPF...");
	const std::string command = "popf -n " + cfg_domain_path_ + std::string(" ") + cfg_problem_path_;

	const std::string result = run_planner(command);

	std::size_t cur_pos = result.find("Solution Found");
	if (cur_pos == std::string::npos) {
		logger->log_error(name(), "Planning failed: %s", result.c_str());
		action_list_.clear();
		return;
	}
	cur_pos                = result.find("\n", cur_pos);
	const std::string plan = result.substr(cur_pos);

	std::istringstream iss(plan);
	logger->log_info(name(), "Planner found solution.");
	logger->log_info(name(), "Result:\n%s", plan.c_str());
	std::string      line;
	const std::regex skip_line("\\s*(;.*)?");
	const std::regex action_regex("\\d*\\.\\d*:\\s\\((.*)\\)\\s*\\[\\d*.\\d*\\]");
	const std::regex action_split_regex("\\S+");

	action_list_.clear();
	while (getline(iss, line)) {
		logger->log_debug(name(), "Parsing line %s", line.c_str());
		std::smatch regex_match;
		if (std::regex_match(line, skip_line)) {
			logger->log_debug(name(), "Skipping line '%s'", line.c_str());
		} else if (std::regex_match(line, regex_match, action_regex)) {
			action            a;
			const std::string action_string = regex_match[1];
			const auto        action_args_begin =
			  std::sregex_iterator(action_string.begin(), action_string.end(), action_split_regex);
			const auto action_args_end = std::sregex_iterator();
			if (action_args_begin == action_args_end) {
				throw Exception("Unexpected action string, could not find action name in '%s'",
				                action_string.c_str());
			}
			a.name = action_args_begin->str();
			for (auto i = std::next(action_args_begin); i != action_args_end; ++i) {
				a.args.push_back(i->str());
			}
			action_list_.push_back(a);
		} else {
			action_list_.clear();
			logger->log_error(name(), "Unexpected planner output line: %s", line.c_str());
			return;
		}
	}
}

document::value
PddlPlannerThread::BSONFromActionList()
{
	using namespace bsoncxx::builder;
	basic::document plan;
	plan.append(basic::kvp("plan", 1));
	plan.append(basic::kvp("msg_id", static_cast<int64_t>(plan_if_->msg_id())));
	plan.append(basic::kvp("actions", [&](basic::sub_array actions) {
		for (action &a : action_list_) {
			basic::document action;
			action.append(basic::kvp("name", a.name));
			action.append(basic::kvp("args", [a](basic::sub_array args) {
				for (std::string arg : a.args) {
					args.append(arg);
				}
			}));
			actions.append(action);
		}
	}));

	return plan.extract();
}

size_t
PddlPlannerThread::find_nth_space(const std::string &s, size_t nth)
{
	size_t   pos        = 0;
	unsigned occurrence = 0;

	while (occurrence != nth && (pos = s.find(' ', pos + 1)) != std::string::npos) {
		++occurrence;
	}

	return pos + 1;
}

void
PddlPlannerThread::print_action_list()
{
	unsigned int count = 0;
	for (action a : action_list_) {
		count++;
		std::string args;
		for (std::string arg : a.args) {
			args += arg + " ";
		}
		logger->log_info(name(), "Action %d %s with args %s", count, a.name.c_str(), args.c_str());
	}
}

std::string
PddlPlannerThread::run_planner(std::string command)
{
	logger->log_info(name(), "Running planner with command: %s", command.c_str());
	std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	char        buffer[128];
	std::string result;
	while (!feof(pipe.get())) {
		if (fgets(buffer, 128, pipe.get()) != NULL)
			result += buffer;
	}
	logger->log_info(name(), "Planner finished run.");

	return result;
}

bool
PddlPlannerThread::bb_interface_message_received(Interface *      interface,
                                                 fawkes::Message *message) noexcept
{
	if (message->is_of_type<PddlPlannerInterface::PlanMessage>()) {
		PddlPlannerInterface::PlanMessage *msg = (PddlPlannerInterface::PlanMessage *)message;
		plan_if_->set_msg_id(msg->id());
		plan_if_->set_success(false);
		plan_if_->set_final(false);
		plan_if_->write();
		wakeup(); //activates loop where the generation is done
	} else {
		logger->log_error(name(), "Received unknown message of type %s, ignoring", message->type());
	}
	return false;
}
