/***************************************************************************
 *  rl_test_thread.cpp -
 *
 *  Created:
 *  Copyright
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

#include "rl-test-thread.h"

#include <chrono>
#include <future>
#include <iostream>
#include <regex>
#include <string>
#include <thread>

//for calling boost python from plugin
//#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <core/threading/mutex_locker.h>

#include <boost/bind/bind.hpp>
#include <clipsmm.h>

using namespace std;

using namespace fawkes;

/** @class RLTestThread
 *  The plugin thread, initializes the aspect.
 *
 *  @author
 */

constexpr char RLTestThread::cfg_prefix_[];

RLTestThread::RLTestThread()
: Thread("RLTestThread", Thread::OPMODE_WAITFORWAKEUP), //Thread::OPMODE_CONTINUOUS),//
  BlackBoardInterfaceListener("RLTestThread"),
  CLIPSFeature("rl-test"),
  CLIPSFeatureAspect(this)
{
}

std::string
getConfigStringReplacedBasedir(Configuration *config, std::string configEntry)
{
	std::string config_value =
	  std::regex_replace(config->get_string(configEntry), std::regex("@BASEDIR@"), BASEDIR);
	return config_value;
}

bool
//RLTestThread::
trainingRlAgent(Configuration *config)
{
	bool        is_done       = false;
	std::string rl_agent_name = config->get_string("/rl-agent/name");
	std::string rl_agent_dir  = getConfigStringReplacedBasedir(config, "/rl-agent/dir");
	std::cout << "RL agent path: " << rl_agent_dir << std::endl;
	std::string training_script = config->get_string("/python/training-script");
	std::string training_dir    = getConfigStringReplacedBasedir(config, "/python/dir");

	std::string env             = config->get_string("/rl-agent/env_name");
	std::string env_dir         = getConfigStringReplacedBasedir(config, "/python/env-dir");
	std::string bin_plugins_dir = getConfigStringReplacedBasedir(config, "/python/plugins-dir");

	std::cout << "Rl-Test-Thread training RL agent before training -laps" << std::endl;
	int training_laps = config->get_int("rl-agent/training-laps");
	std::cout << "Rl-Test-Thread training RL agent after training -laps" << std::endl;

	py::scoped_interpreter guard{};

	try {
		py::object main_namespace = py::module_::import("__main__").attr("__dict__");
		py::exec("import sys", main_namespace);
		py::exec("print(\"Hello From python\")", main_namespace);

		//necessary to include other python scripts - e.g. PDDLExtension
		py::str sysPathAppend = (py::str)("sys.path.append(\"" + training_dir + "\")");
		py::exec(sysPathAppend, main_namespace);

		//necessary to include other python scripts - e.g. ClipsWorld
		py::str sysPathAppend2 = (py::str)("sys.path.append(\"" + env_dir + "\")");
		py::exec(sysPathAppend2, main_namespace);

		py::str sysPathAppend3 = (py::str)("sys.path.append(\"" + bin_plugins_dir + "\")");
		py::exec(sysPathAppend3, main_namespace);

		py::exec("print(sys.path)");

		//Add env name extracted from the config file to python
		py::str env_name = (py::str)("env_name = \"" + env + "\"");
		py::exec(env_name, main_namespace);

		//Add dir of env_name.pddl and problem.pddl extracted from the config file to python
		py::str dir_path = (py::str)("dir_path = \"" + rl_agent_dir + "\"");
		py::exec(dir_path, main_namespace);

		//Adds dir + name, where the rl agent should be saved to python
		std::cout << "RL Agent: " + rl_agent_dir + "/" + rl_agent_name << std::endl;
		py::str file_name = (py::str)("file_name = \"" + rl_agent_dir + "/" + rl_agent_name + "\"");
		py::exec(file_name, main_namespace);

		//Value of training timesteps
		py::str timesteps = (py::str)("timesteps = " + std::to_string(training_laps)); //1000
		py::exec(timesteps, main_namespace);

		//printing python variables with config values
		//main_print =
		py::exec("print(\"Config values for env_name and path: \" + env_name + \" \" + dir_path)",
		         main_namespace);

		std::cout << "Training script: " + rl_agent_dir + "/" + training_script << std::endl;
		py::str training_script_path = (py::str)(rl_agent_dir + "/" + training_script);
		//py::exec_file(training_script_path, main_namespace, main_namespace);
		auto result = py::eval_file(training_script_path, main_namespace);
		std::cout << "\n\n\nDONE EVALUATING TRAINING SCRIPT \n\n\n" << std::endl;
		py::print(result);
		is_done = true;

	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}

	return is_done;
}

std::string
RLTestThread::executeRlAgent(std::string fact_string)
{
	std::string selected_action = "";

	PyThreadState *py_inter = Py_NewInterpreter();
	if (!Py_IsInitialized()) {
		std::cout << "Initialising the Python interpreter" << std::endl;

		//py::initialize_interpreter();
	}

	try {
		py::object py_scope = py::module_::import("__main__").attr("__dict__");
		py::exec("import sys", py_scope);

		py::str sysPathAppend1 = (py::str)("sys.path.append(\"" + cfg_rl_agent_dir + "\")");
		py::str sysPathAppend2 = (py::str)("sys.path.append(\"" + cfg_python_dir + "\")");
		py::str sysPathAppend3 = (py::str)("sys.path.append(\"" + cfg_env_dir + "\")");
		py::str sysPathAppend4 = (py::str)("sys.path.append(\"" + cfg_bin_plugins_dir + "\")");

		py::exec(sysPathAppend1, py_scope);
		py::exec(sysPathAppend2, py_scope);
		py::exec(sysPathAppend3, py_scope);
		py::exec(sysPathAppend4, py_scope);
		py::exec("print(\"added config directories to sys.path\")");
		py::exec("print(sys.path)", py_scope);

		std::cout << "RL Agent file: " + cfg_rl_agent_dir + "/" + cfg_rl_agent_name << std::endl;
		py::str file_name =
		  (py::str)("file_name = \"" + cfg_rl_agent_dir + "/" + cfg_rl_agent_name + "\"");
		py::exec(file_name, py_scope); //main_namespace2);

		py::str obs = (py::str)("obs =" + fact_string);
		py::exec(obs, py_scope);
		py::exec("print(\"Current observation: \", obs)", py_scope);

		std::cout << "Execution script: " + cfg_python_dir + "/" + cfg_execution_script << std::endl;
		py::str    execution_script_path = (py::str)(cfg_python_dir + "/" + cfg_execution_script);
		py::object temp                  = py::eval_file(execution_script_path, py_scope, py_scope);

		py::exec("print(\"Result py: \", result)", py_scope);

		auto result = py_scope["result"].cast<std::string>();
		//auto result     = py_scope["result"].cast<std::string>();
		selected_action = result;

		std::cout << "\nResult: " + selected_action << std::endl;

	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (const std::runtime_error &re) {
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << re.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}
	std::cout << "In executeRlAgent - after pycode run " << selected_action << std::endl;

	Py_EndInterpreter(py_inter);
	//py::finalize_interpreter();
	//std::cout << "pybind 11 interpreter finalized" <<std::endl;

	std::cout << "In executeRlAgent - after interpreter finalized: " << selected_action << std::endl;

	return selected_action;
}

void
RLTestThread::init()
{
	std::cout << "Init RLTestThread start" << std::endl;

	/* Reading config values */
	cfg_rl_agent_name    = config->get_string("/rl-agent/name");
	cfg_rl_agent_dir     = getConfigStringReplacedBasedir(config, "/rl-agent/dir");
	cfg_python_dir       = getConfigStringReplacedBasedir(config, "/python/dir");
	cfg_env_dir          = getConfigStringReplacedBasedir(config, "/python/env-dir");
	cfg_bin_plugins_dir  = getConfigStringReplacedBasedir(config, "/python/plugins-dir");
	cfg_execution_script = config->get_string("/python/execution-script");
	cfg_env_script       = config->get_string("/python/env-script");
	cfg_training_mode    = config->get_bool("/rl-agent/training-mode");

	std::cout << "Reading config values done: "
	          << cfg_rl_agent_dir + cfg_rl_agent_name + " " + std::to_string(cfg_training_mode)
	          << std::endl;

	//setup blackboard interface
	rl_gs_interface = blackboard->open_for_writing<RLAgentGoalSelectionInterface>("goal-selection");
	rl_gs_interface->set_msg_id(0);
	rl_gs_interface->set_final(false);
	rl_gs_interface->write();

	//setup interface listener
	bbil_add_message_interface(rl_gs_interface);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

	startedTraining        = false;
	startedExecution       = false;
	count_startedExecution = 0;

	std::cout << "Finished RLTestThread init" << std::endl;
	//wakeup(); //activates any loop
}

void
RLTestThread::loop()
{
	std::cout << "In RLTestThread Loop " << std::endl;
	rl_gs_interface->set_final(true);
	rl_gs_interface->set_success(true);
	rl_gs_interface->set_next_select_goal("RL TEST GOAL FROM LOOP");
	rl_gs_interface->write();
	std::future<std::string> execution_done;

	std::cout << "cfg_training_mode: " << cfg_training_mode << " started exection count "
	          << count_startedExecution << std::endl;

	if (!cfg_training_mode && count_startedExecution < 3) {
		/* Using the rl agent to predict the next goal */
		std::cout << "RlTestThread: in loop start execution mode" << std::endl;
		//std::string fact_string = create_rl_env_state_from_facts();

		py_guard = PyGuard::getInstance();
		std::cout << "PyGilState: " << PyGILState_Check() << std::endl;
		//fact_string = "{'said(bob#hello)', 'stack(e#b)', 'stack(c#d)', 'stack(e#d)', 'ontable(b)', 'ontable(e)', 'unstack(d)', 'stack(d#a)', 'stack(a#e)', 'stack(b#d)', 'stack(d#b)', 'clear(e)', 'handempty(robo1)', 'stack(d#c)', 'ontable(a)', 'stack(b#e)', 'stack(e#a)', 'stack(c#b)', 'stack(a#c)', 'putdown(e)', 'pickup(a)', 'stack(e#c)', 'pickup(d)', 'unstack(a)', 'pickup(b)', 'clear(b)', 'stack(b#a)', 'stack(c#a)', 'putdown(d)', 'stack(d#e)', 'stack(b#c)', 'unstack(c)', 'stack(a#b)', 'putdown(a)', 'ontable(d)', 'clear(c)', 'putdown(c)', 'ontable(c)', 'clear(d)', 'clear(a)', 'unstack(b)', 'putdown(b)', 'stack(c#e)', 'pickup(c)', 'pickup(e)', 'unstack(e)', 'stack(a#d)'}";
		if (count_startedExecution == 0) {
			PyGuard::getInstance()->loadConfig(config);
			PyGuard::getInstance()->loadEnv();
		}
		py_guard->print();
		auto action = py_guard->predict();
		//auto action = "TOWER-C1#buttom#a#top#c";
		std::cout << "executeRlAgent returned: " << action << std::endl;
		if (action != "") {
			auto goal_id = getGoalId(action);
			std::cout << "GetGoalID: " << goal_id << std::endl;
			assertRlGoalSelectionFact(goal_id);
		}
		//execution_done   = std::async(std::launch::async, executeRlAgent, config, count_startedExecution);

		count_startedExecution++;
		std::cout << "RlTestThread in rl_goal_selection started async execution thread " << std::endl;
	} else if (cfg_training_mode && !startedTraining) {
		/* Training RL agent */
		std::cout << "RlTestThread: rl_goal_selection - executing RL Agent is not active!" << std::endl;
		training_done   = std::async(std::launch::async, trainingRlAgent, config);
		startedTraining = true;
	} else if (cfg_training_mode && startedTraining) {
		/* Checking if training completed */
		std::cout << "Check if training_done future is vailid " << training_done.valid() << std::endl;
		int sec = 10; //00;
		std::cout << "Wait for " << sec << " msec to check future status" << std::endl;
		std::future_status status = training_done.wait_for(std::chrono::milliseconds(sec));
		if (status == std::future_status::ready) {
			std::cout << "Future: " << training_done.get() << std::endl;
			finalize();
		}
	} else {
		std::cout << "Nothing to do!" << std::endl;
		//delete py_guard;
		//finalize();
	}

	std::cout << "End RLTestThread Loop " << std::endl;
}

void
RLTestThread::finalize()
{
	//Py_Finalize();
	py::finalize_interpreter();
	blackboard->close(rl_gs_interface);
}

bool
RLTestThread::bb_interface_message_received(Interface *interface, fawkes::Message *message) noexcept
{
	std::cout << "In bb_interface_message_received " << std::endl;
	if (message->is_of_type<RLAgentGoalSelectionInterface::GSelectionMessage>()) {
		RLAgentGoalSelectionInterface::GSelectionMessage *msg =
		  (RLAgentGoalSelectionInterface::GSelectionMessage *)message;
		rl_gs_interface->set_msg_id(msg->id());
		rl_gs_interface->set_final(false);
		rl_gs_interface->write();
		if (std::string(msg->goal()) != "")
			goal = msg->goal();
		wakeup(); //activates loop where the generation is done
	} else {
		logger->log_error(name(), "Received unknown message of type %s, ignoring", message->type());
	}
	return false;
}

void
RLTestThread::clips_context_init(const std::string &env_name, LockPtr<CLIPS::Environment> &clips)
{
	std::cout << "Start RLTestThread clips_context_init\n" << std::endl;
	clips_env_name  = env_name;
	envs_[env_name] = clips;
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();
	clips->evaluate("(printout t \"Hello from CLIPS aspect in RL test plugin\" crlf crlf)");

	/*clips->add_function("rl-goal-selection-start",
	                    sigc::slot<void, CLIPS::Value, std::string>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &RLTestThread::rl_goal_selection),
	                                    env_name)));*/

	if (!cfg_training_mode) {
		clips->assert_fact("(execution-mode)");
	}

	clips.unlock();
}

void
RLTestThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

fawkes::LockPtr<CLIPS::Environment>
RLTestThread::getClipsEnv()
{
	fawkes::LockPtr<CLIPS::Environment> clips = envs_[clips_env_name];
	return clips;
}

void
RLTestThread::rl_goal_selection(std::string env_name, CLIPS::Value parent_goal_id, std::string to)
{
	std::cout << "RlTestThread: in rl_goal_selection" << std::endl;
	std::cout << "cfg_training_mode: " << cfg_training_mode << " started exection "
	          << startedExecution << std::endl;
	if (cfg_training_mode && !startedTraining) {
		/* Training RL agent */
		std::cout << "RlTestThread: rl_goal_selection - executing RL Agent is not active!" << std::endl;
		training_done   = std::async(std::launch::async, trainingRlAgent, config);
		startedTraining = true;
	} else if (cfg_training_mode && startedTraining) {
		/* Checking if training completed */
		std::cout << "Check if training_done future is vailid " << training_done.valid() << std::endl;
		int sec = 10; //00;
		std::cout << "Wait for " << sec << " msec to check future status" << std::endl;
		std::future_status status = training_done.wait_for(std::chrono::milliseconds(sec));
		if (status == std::future_status::ready) {
			std::cout << "Future: " << training_done.get() << std::endl;
			finalize();
		}
	} else {
		std::cout << "In rl_goal_selection - nothing to do" << std::endl;
	}
}

/*
* Functions from clips-gym
* TODO: extract to utility class or merge rl-test and clips-gym plugin
*/

std::string
RLTestThread::create_rl_env_state_from_facts()
{
	std::cout << "In create rl env state from facts" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//refcount
	std::cout << "Clips pointer refcount: " << clips.refcount() << std::endl;
	std::cout << current_thread_id() << " " << current_thread_name() << std::endl;

	clips.lock();
	std::cout << "Lock clips done" << std::endl;
	std::cout << "Clips pointer refcount: " << clips.refcount() << std::endl;
	CLIPS::Fact::pointer fact             = clips->get_facts();
	std::string          env_state_string = "{";
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();

		std::size_t found = tmpl->name().find("domain-fact");

		if (found != std::string::npos) {
			std::string fact_name         = getClipsSlotValuesAsString(fact->slot_value("name"));
			std::string fact_param_values = getClipsSlotValuesAsString(fact->slot_value("param-values"));
			env_state_string += "\"" + fact_name + "(" + fact_param_values + ")\",";

			//std::cout << fact_value <<std::endl;
		}
		fact = fact->next();
	}
	env_state_string = env_state_string.substr(0, env_state_string.length() - 1) + "}";
	std::cout << env_state_string << std::endl;
	std::cout << "Finished passing all facts " << std::endl;
	clips.unlock();
	std::cout << "Unlock clips done: Clips pointer refcount: " << clips.refcount() << std::endl;

	return env_state_string;
}

std::string
RLTestThread::getGoalId(std::string action)
{
	std::cout << "RLTestThread: getGoalId of " << action << std::endl;

	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact   = clips->get_facts();
	std::string          goalID = "";
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("goal");

		if (found != std::string::npos) {
			/*
			Slot names: id, class,type, sub-type, parent, mode, outcome, warning, error, message,
						priority, params, meta, meta-fact, meta-template, required-resources, acquired-resources,
						committed-to,verbosity,is-executable,
			Class: TOWER-C1, params: buttom,b,top,d
			*/
			std::string current_class  = getClipsSlotValuesAsString(fact->slot_value("class"));
			std::string current_params = getClipsSlotValuesAsString(fact->slot_value("params"));

			std::string current_action = current_class + "#" + current_params;
			std::cout << "current action: " << current_action << std::endl;
			if (current_action == action) {
				goalID = getClipsSlotValuesAsString(fact->slot_value("id"));
				std::cout << "correct class and params! GoalID is: " << goalID << std::endl;
				break;
			}
		}
		fact = fact->next();
	}
	std::cout << "Finished passing all goals" << std::endl;
	clips.unlock();
	return goalID;
}

std::string
RLTestThread::getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values)
{
	std::string value = "";
	for (std::size_t i = 0; i < slot_values.size(); i++) {
		auto v = slot_values[i];
		switch (v.type()) {
		case CLIPS::TYPE_FLOAT:
			// std::cout << v.as_float() << std::endl;
			value += std::to_string(v.as_float());
			break;

		case CLIPS::TYPE_INTEGER:
			//std::cout << v.as_integer() << std::endl;
			value += std::to_string(v.as_integer());
			break;

		default:
			//std::cout << v.as_string() <<std::endl;
			value += v.as_string();
		}
		if (slot_values.size() > 1 && i != (slot_values.size() - 1)) {
			value += "#";
		}
	}
	return value;
}

void
RLTestThread::assertRlGoalSelectionFact(std::string goalID)
{
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();

	clips->evaluate("(printout t \"In RLTestThread assertRlGoalSelectionFact: next goal " + goalID
	                + "\" crlf)");
	CLIPS::Value             v    = CLIPS::Value(goalID, CLIPS::TYPE_SYMBOL);
	CLIPS::Template::pointer temp = clips->get_template("rl-goal-selection");

	CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
	fact->set_slot("next-goal-id", v);
	clips->assert_fact(fact);

	clips.unlock();
	std::cout << "assertRlGoalSelectionFact done" << std::endl;
}