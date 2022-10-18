/***************************************************************************
 *  clips-gym-thread.cpp -
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

#include "clips-gym-thread.h"
//#include "/home/sonja/MA-Testproject/rlblocksworld/src-cpp/testBoostPython.h"
#include <iostream>
#include <regex>
#include <string>

//for calling boost python from plugin
//#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <core/threading/mutex_locker.h>
#include <pybind11/embed.h>

#include <boost/bind/bind.hpp>
#include <clipsmm.h>

using namespace std;
//using namespace boost::python;
namespace py = pybind11;

using namespace fawkes;

/** @class ClipsGymThread
 *  The plugin thread, initializes the aspect.
 *
 *  @author
 */

constexpr char ClipsGymThread::cfg_prefix_[];

//AspectProviderAspect(&provider_inifin_)
//provider_inifin_(&rl_test_manager_)

ClipsGymThread::ClipsGymThread()
: Thread("ClipsGymThread", Thread::OPMODE_WAITFORWAKEUP), //OPMODE_CONTINUOUS),//
  BlackBoardInterfaceListener("ClipsGymThread"),
  CLIPSFeature("clips-gym"),
  CLIPSFeatureAspect(this)
{
}

ClipsGymThread *ClipsGymThread::thread_instance{nullptr};
std::mutex      ClipsGymThread::mutex;

ClipsGymThread *
ClipsGymThread::getInstance()
{
	std::lock_guard<std::mutex> lock(mutex);
	if (thread_instance == nullptr) {
		thread_instance = new ClipsGymThread();
	}
	return thread_instance;
}

int
add(int i, int j)
{
	return i + j + 5;
}

PYBIND11_MODULE(clips_gym, m)
{
	m.doc() = "pybind11 example plugin"; // optional module docstring

	m.def("add", &add, "A function that adds two numbers");

	py::class_<ClipsGymThread>(m, "ClipsGymThread")
	  .def(py::init<>())
	  .def("getInstance", &ClipsGymThread::getInstance, py::return_value_policy::reference)
	  .def("step", &ClipsGymThread::step)
	  .def("resetCX", &ClipsGymThread::resetCX)
	  .def("create_rl_env_state_from_facts", &ClipsGymThread::create_rl_env_state_from_facts)
	  .def("getAllFormulatedGoals", &ClipsGymThread::getAllFormulatedGoals)
	  .def("generateActionSpace", &ClipsGymThread::generateActionSpace)
	  .def("generateObservationSpace", &ClipsGymThread::generateObservationSpace);

	//std::string step(std::string next_goal);

	//.def("getClipsEnv", &ClipsGymThread::getClipsEnv);
}

void
ClipsGymThread::init()
{
	std::cout << "Hello World!-from ClipsGymThread" << std::endl;
	//std::string clips_gym_name = config->get_string("/clips-gym/name");

	//setup interface
	/*rl_gs_interface = blackboard->open_for_writing<RLAgentGoalSelectionInterface>(
	  "goal-selection");
	  //config->get_string("plugins/pddl-robot-memory/interface-name").c_str());
	rl_gs_interface->set_msg_id(0);
	rl_gs_interface->set_final(false);
	rl_gs_interface->write();

	//setup interface listener
	bbil_add_message_interface(rl_gs_interface);
	blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
	*/

	std::cout << "Hello 2 From ClipsGymThread Thread" << std::endl;

	//wakeup(); //activates any loop
}

void
ClipsGymThread::loop()
{
	std::cout << "In Loop ClipsGymThread" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact       = clips->get_facts();
	bool                 found_fact = false;

	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		//std::cout << "Template: " + tmpl->name() << std::endl;

		std::size_t found_pos = tmpl->name().find("rl-waiting");
		//std::size_t found2 = tmpl->name().find("wm-fact");//"predicate");
		//std::size_t found3 = tmpl->name().find("goal");

		if (found_pos
		    != std::string::npos) //|| found2 !=std::string::npos ) //|| found3 !=std::string::npos)
		{
			std::cout << "RL WAITING fact found in loop clips gym" << std::endl;
			found_fact = true;
		}
		fact = fact->next();
	}

	if (!found_fact) {
		std::cout << "RL WAITING fact  was NOT found in loop clips gym" << std::endl;
	}

	clips.unlock();

	/*rl_gs_interface->set_final(true);
	rl_gs_interface->set_success(true);
	rl_gs_interface->set_next_select_goal("RL TEST GOAL FROM LOOP");
	rl_gs_interface->write();
	*/

	/*try{
		//py::object main_module = py::import("__main__");
		//py::object
		//main_namespace = main_module.attr("__dict__");
		//py::object main_namespace = py::module_::import("__main__").attr("__dict__");
		//py::object main_sys =
		py::exec("import sys", main_namespace);
		py::exec("import example", main_namespace);
		py::exec("result = example.add(i=1, j=2)", main_namespace);
		py::exec("print(\"Result py: \", result)", main_namespace);
	}
	catch(...)
	{
		PyErr_Print();
		PyErr_Clear();
	}*/

	std::cout << "End Loop " << std::endl;
	sleep(200);
}

void
ClipsGymThread::finalize()
{
	//blackboard->close(rl_gs_interface);
}

std::string
ClipsGymThread::step(std::string next_goal)
{
	std::cout << "In ClipsGymThread step function" << std::endl;
	std::cout << "next_goal from python: " << next_goal << std::endl;
	//Transform string to goal
	//std::string n_goal = "TOWER-C1#b#d#";

	std::string                         goalID = getGoalId(next_goal);
	fawkes::LockPtr<CLIPS::Environment> clips  = getClipsEnv();
	clips.lock();

	clips->evaluate("(printout t \"In ClipsGymThread step: asserting fact with next action\" crlf)");
	CLIPS::Value             v    = CLIPS::Value(goalID, CLIPS::TYPE_SYMBOL); //CLIPS::TYPE_STRING);
	CLIPS::Template::pointer temp = clips->get_template("rl-action-selection");

	CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
	fact->set_slot("next-action", v);
	clips->assert_fact(fact);

	clips.unlock();
	std::cout << "fact asserted, start running clips" << std::endl;

	bool env_feedback = false;
	while (!env_feedback) {
		std::this_thread::sleep_for(2000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping Step Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl  = fact->get_template();
			std::size_t              found = tmpl->name().find("rl-finished-goal");
			if (found != std::string::npos) {
				std::string goalID  = getClipsSlotValuesAsString(fact->slot_value("goal-id"));
				std::string outcome = getClipsSlotValuesAsString(fact->slot_value("outcome"));
				std::cout << "In ClipsGymThread step: Goal: " << goalID
				          << " is evaluated with outcome: " << outcome << std::endl;
				env_feedback = true;
				fact->retract();
				std::cout << "In ClipsGymThread step: after retracting rl-finished-goal fact" << std::endl;
				break;
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
	}
	/*
		for (auto &s: g_splitted) {
			std::cout << s << std::endl;
		}
	*/
	//clips->refresh_agenda();
	//clips->run();
	std::string env_state = create_rl_env_state_from_facts();

	std::cout << "End Clips Gym Thread step function" << std::endl;
	return env_state;
}

py::list
ClipsGymThread::generateActionSpace()
{
	//TODO: implement generation based on clips goals
	std::string space[] = {"TOWER-C1#buttom#a#top#c",
	                       "TOWER-C1#buttom#b#top#d",
	                       "TOWER-C1#buttom#e#top#d"}; //, "TOWER-C1#buttom#a#top#e"};

	py::list action_space;
	for (std::string s : space) {
		action_space.append((py::str)s);
	}
	return action_space;
}

py::list
ClipsGymThread::generateObservationSpace()
{
	//TODO: implement generation based on clips facts
	std::string space[] = {"clear(a)",   "clear(b)",         "clear(c)",        "clear(d)",
	                       "clear(e)",   "handempty(robo1)", "handfull(robo1)", "holding(a)",
	                       "holding(b)", "holding(c)",       "holding(d)",      "holding(e)",
	                       "on(a,b)",    "on(a,c)",          "on(a,d)",         "on(a,e)",
	                       "on(b,a)",    "on(b,c)",          "on(b,d)",         "on(b,e)",
	                       "on(c,a)",    "on(c,b)",          "on(c,d)",         "on(c,e)",
	                       "on(d,a)",    "on(d,b)",          "on(d,c)",         "on(d,e)",
	                       "on(e,a)",    "on(e,b)",          "on(e,c)",         "on(e,d)",
	                       "ontable(a)", "ontable(b)",       "ontable(c)",      "ontable(d)",
	                       "ontable(e)"};
	py::list    obs_space;
	for (std::string s : space) {
		obs_space.append((py::str)s);
	}
	return obs_space;
}

std::vector<std::string>
ClipsGymThread::splitActionToGoalParams(std::string action)
{
	std::vector<std::string> g_splitted;
	std::string::size_type   begin = 0;
	for (std::string::size_type end = 0; (end = action.find("#", end)) != std::string::npos; ++end) {
		g_splitted.push_back(action.substr(begin, end - begin));
		begin = end + 1;
	}
	return g_splitted;
}
//todo umbennennen getactionGoalIdMapping oder so
std::string
ClipsGymThread::getGoalId(std::string action)
{
	std::cout << "ClipsGym: getGoalId of " << action << std::endl;
	std::vector<std::string> action_splitted = splitActionToGoalParams(action);
	std::cout << "ClipsGym: splitted action " << std::endl;
	for (std::string s : action_splitted) {
		std::cout << s << std::endl;
	}
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact   = clips->get_facts();
	std::string          goalID = "";
	//std::string          allGoals = "{ ";
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
			std::vector<std::string> slot_names         = fact->slot_names();
			bool                     correct_goal_class = false;
			bool                     correct_params     = true;
			std::string              temp_id            = "";
			for (std::string s : slot_names) {
				//allGoals += getClipsSlotValuesAsString(fact->slot_value(s));
				if (s == "class") {
					std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
					std::cout << "Class: " + slot_values << std::endl;
					if (action_splitted[0] == slot_values) {
						correct_goal_class = true;
					}
				}
				//TODO &&corret_goal_class
				if (s == "params") {
					std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
					std::cout << "params: " << slot_values << std::endl;
					for (size_t i = 1; i < action_splitted.size(); i++) {
						std::cout << "search for: " << action_splitted[i] << std::endl;
						std::size_t found_param = slot_values.find(action_splitted[i]); //(p);
						if (found_param != std::string::npos) {
							correct_params = correct_params && true;
						} else {
							correct_params = correct_params && false;
						}
					}
				}
				if (s == "id") {
					temp_id = getClipsSlotValuesAsString(fact->slot_value(s));
					std::cout << "id: " << temp_id << std::endl;
				}
			}
			if (correct_goal_class && correct_params) {
				std::cout << "correct class and params! GoalID is: " << temp_id << std::endl;
				goalID = temp_id;
			}
			//allGoals += "\n ";
		}
		fact = fact->next();
	}
	//std::cout << allGoals << std::endl;
	std::cout << "Finished passing all goals" << std::endl;
	clips.unlock();
	return goalID;
}

std::string
ClipsGymThread::getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values)
{
	std::string value = "";
	for (std::size_t i = 0; i < slot_values.size(); i++) //for(CLIPS::Value v: slot_values)
	{
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
		if (slot_values.size() > 1
		    && i != (slot_values.size() - 1)) //v != slot_values[slot_values.size()-1])
		{
			value += ",";
		}
	}
	return value;
}

std::vector<std::string> *
ClipsGymThread::getClipsSlotValuesAsStringVector(std::vector<CLIPS::Value> slot_values)
{
	std::vector<std::string> *value = new std::vector<std::string>();
	for (std::size_t i = 0; i < slot_values.size(); i++) //for(CLIPS::Value v: slot_values)
	{
		auto v = slot_values[i];
		switch (v.type()) {
		case CLIPS::TYPE_FLOAT:
			// std::cout << v.as_float() << std::endl;
			value->push_back(std::to_string(v.as_float()));
			break;

		case CLIPS::TYPE_INTEGER:
			//std::cout << v.as_integer() << std::endl;
			value->push_back(std::to_string(v.as_integer()));
			break;

		default:
			//std::cout << v.as_string() <<std::endl;
			value->push_back(v.as_string());
		}
	}
	return value;
}

std::vector<std::string>
ClipsGymThread::getAllFormulatedGoals()
{
	std::cout << "In ClipsGymThread get all formulated goals" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer     fact   = clips->get_facts();
	std::string              goalID = "";
	std::vector<std::string> maskedGoals;
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("goal");

		if (found != std::string::npos) {
			std::string s           = "mode";
			std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
			std::cout << slot_values << std::endl;
			if (slot_values == "FORMULATED") {
				std::string goal_class = getClipsSlotValuesAsString(fact->slot_value("class"));
				std::cout << goal_class << std::endl;

				std::string goal_params = getClipsSlotValuesAsString(fact->slot_value("params"));
				std::cout << goal_params << std::endl;
				maskedGoals.push_back(goal_class + "#" + goal_params);
			}
		}
		fact = fact->next();
	}
	//std::cout<<maskedGoals <<std::endl;
	std::cout << "Finished passing all formulated goals" << std::endl;
	clips.unlock();
	return maskedGoals;
}

void
ClipsGymThread::resetCX()
{
	std::cout << "In ClipsGymThread resetCX" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//clips->clear(), clips->reset() haben zu coredumps geführt
	clips.lock();
	clips->assert_fact("(reset-domain-facts)");
	//clips->refresh_agenda();
	//clips->run();
	clips.unlock();
	std::cout << "Finished resetCX" << std::endl;
	//CLIPS::Fact::pointer fact = clips->get_facts();
}

void
ClipsGymThread::clips_context_init(const std::string &env_name, LockPtr<CLIPS::Environment> &clips)
{
	std::cout << "Hello World!-from ClipsGymThread clips_context_init\n" << std::endl;
	clisp_env_name  = env_name;
	envs_[env_name] = clips;
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();
	clips->evaluate("(printout t \"Hello from CLIPS aspect in ClipsGymThread \" crlf crlf)");
	/*clips->add_function("rl-extract-executable-fact",
						   sigc::slot<void, CLIPS::Value, std::string>(sigc::bind<0>(
						  sigc::mem_fun(*this, &RLTestThread::clips_rl_extract_executable_facts),
						  env_name)));*/
	/*clips->add_function("rl-goal-selection-start",
																	   sigc::slot<void, CLIPS::Value, std::string>(sigc::bind<0>(
																	  sigc::mem_fun(*this, &RLTestThread::rl_goal_selection),
																	  env_name)));*/

	clips->add_function("rl-loop-start",
	                    sigc::slot<void>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsGymThread::rl_loop_start),
	                                    env_name)));

	//clips->refresh_agenda();
	//clips->run();
	clips.unlock();
}

void
ClipsGymThread::rl_loop_start(std::string env_name)
{
	std::cout << "In rl_loop_start\n" << std::endl;

	wakeup();
}

void
ClipsGymThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

fawkes::LockPtr<CLIPS::Environment>
ClipsGymThread::getClipsEnv() //std::string env_name)
{
	fawkes::LockPtr<CLIPS::Environment> clips = envs_[clisp_env_name]; //envs_[env_name];
	return clips;
}

//ToDo return std::vector<string> literals
std::string
ClipsGymThread::create_rl_env_state_from_facts() //std::string env_name)
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
		//std::cout << "Template: " + tmpl->name() << std::endl;

		std::size_t found = tmpl->name().find("domain-fact");
		//std::size_t found2 = tmpl->name().find("wm-fact");//"predicate");
		//std::size_t found3 = tmpl->name().find("goal");

		if (found
		    != std::string::npos) //|| found2 !=std::string::npos ) //|| found3 !=std::string::npos)
		{
			//if (tmpl->name()->contains("domain") || tmpl->name().contains("goal"))
			std::vector<std::string> slot_names = fact->slot_names();
			std::string              fact_value = "";
			for (std::string s : slot_names) {
				fact_value += " Slot " + s + ": ";
				//std::cout << "Slot name: " + s << std::endl;
				std::vector<CLIPS::Value> slot_values = fact->slot_value(s);
				std::string               value       = "";
				for (std::size_t i = 0; i < slot_values.size(); i++) //for(CLIPS::Value v: slot_values)
				{
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
					if (slot_values.size() > 1
					    && i != (slot_values.size() - 1)) //v != slot_values[slot_values.size()-1])
					{
						value += ",";
					}
				}
				//std::cout << value << std::endl;
				if (s == "name") {
					env_state_string += "\"" + value + "(";
				}
				if (s == "param-values") {
					env_state_string += value + ")\","; //value.substr(0, value.length()-2) + "),";
				}
				fact_value += " " + value;
			}
			//std::cout << fact_value <<std::endl;
		}
		fact = fact->next();
	}
	env_state_string = env_state_string.substr(0, env_state_string.length() - 1) + "}";
	std::cout << env_state_string << std::endl;
	std::cout << "Finished passing all facts " << std::endl;
	clips.unlock();
	std::cout << "Unlock clips done: Clips pointer refcount: " << clips.refcount() << std::endl;
	/* }
	else{
		std::cout << "Clips-Gym-Thread: create env state from facts - FAILED to lock clips!";
	} */
	return env_state_string;
}
