/***************************************************************************
 *  clips_gym_thread.cpp -
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

#include "clips_gym_thread.h"

#include <iostream>
#include <regex>
#include <string>

//for calling boost python from plugin
//#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <core/threading/mutex_locker.h>
#include <pybind11/embed.h>

#include <boost/bind/bind.hpp>
#include <clipsmm.h>
//#include "goalAction.h"

//#include "clips-observation-info.h"

using namespace std;
//using namespace boost::python;

using namespace fawkes;

/** @class ClipsGymThread
 *  The plugin thread, initializes the aspect.
 *
 *  @author
 */

constexpr char ClipsGymThread::cfg_prefix_[];

ClipsGymThread::ClipsGymThread(const char* thread_name = "ClipsGymThread", const char* feature_name = "clips_gym")
: Thread(thread_name, Thread::OPMODE_WAITFORWAKEUP), //OPMODE_CONTINUOUS),//
  BlackBoardInterfaceListener(thread_name),
  CLIPSFeature(feature_name),
  CLIPSFeatureAspect(this)
{
}

ClipsGymThread *ClipsGymThread::thread_instance{nullptr};
std::mutex      ClipsGymThread::mutex;

ClipsGymThread *
ClipsGymThread::getInstance()
{
	std::cout << "ClipsGymThread: getInstance start" << std::endl;
	std::lock_guard<std::mutex> lock(mutex);
	if (thread_instance == nullptr) {
		thread_instance = new ClipsGymThread();
	}
	return thread_instance;
}

PYBIND11_MODULE(libfawkes_clips_gym, m)
{
	m.doc() = "pybind11 example plugin"; // optional module docstring

	py::class_<ClipsObservationInfo>(m, "ClipsObservationInfo")
	  .def(py::init<>())
	  .def_readonly("observation", &ClipsObservationInfo::observation)
	  .def_readonly("reward", &ClipsObservationInfo::reward)
	  .def_readonly("done", &ClipsObservationInfo::done)
	  .def_readonly("info", &ClipsObservationInfo::info);

	py::class_<Param>(m, "Param")
	  .def(py::init<const string, const string>())
	  .def("getParamString", &Param::getParamString);

	py::class_<GoalAction>(m, "Goal")
	  .def(py::init<const string>())
	  .def("setParams", &GoalAction::setParams)
	  .def("getParamsString", &GoalAction::getParamsString)
	  .def("getGoalString", &GoalAction::getGoalString);

	py::class_<ClipsGymThread>(m, "ClipsGymThread")
	  .def(py::init<>())
	  .def("getInstance", &ClipsGymThread::getInstance, py::return_value_policy::reference)
	  .def("step", &ClipsGymThread::step)
	  .def("resetCX", &ClipsGymThread::resetCX)
	  .def("create_rl_env_state_from_facts", &ClipsGymThread::create_rl_env_state_from_facts)
	  .def("getAllFormulatedGoals", &ClipsGymThread::getAllFormulatedGoals)
	  .def("unlockRobot", &ClipsGymThread::unlockRobot)
	  .def("waitForFreeRobot", &ClipsGymThread::waitForFreeRobot)
	  .def("getExecutableGoalsForFreeRobot", &ClipsGymThread::getExecutableGoalsForFreeRobot)
	  .def("getAllFormulatedExecutableGoals", &ClipsGymThread::getAllFormulatedExecutableGoals)
	  .def("generateActionSpace", &ClipsGymThread::generateActionSpace)
	  .def("generateObservationSpace", &ClipsGymThread::generateObservationSpace)
	  .def("getParamsNameTypeMapOfGoal", &ClipsGymThread::getParamsNameTypeMapOfGoal)
	  .def("getParamNameDomainObjectsComb", &ClipsGymThread::getParamNameDomainObjectsComb)
	  .def("getGoalClassList", &ClipsGymThread::getGoalClassList)
	  .def("assertRlGoalSelectionFact", &ClipsGymThread::assertRlGoalSelectionFact)
	  .def("getGoalId", &ClipsGymThread::getGoalId)
	  .def("getGoalIdByString", &ClipsGymThread::getGoalIdByString)
	  .def("getDomainPredicates", &ClipsGymThread::getDomainPredicates)
	  .def("getDomainObjects", &ClipsGymThread::getDomainObjects)
	  .def("clipsGymSleep", &ClipsGymThread::clipsGymSleep)
	  .def("log", &ClipsGymThread::log);
}


void
ClipsGymThread::init()
{
	std::cout << "Hello World!-from ClipsGymThread" << std::endl;
	lockRobot = false;
}

/* Checks if rl-waiting fact exists and sleeps */
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

		std::size_t found_pos = tmpl->name().find("rl-waiting");
		//std::size_t found2 = tmpl->name().find("wm-fact");//"predicate");
		//std::size_t found3 = tmpl->name().find("goal");

		if (found_pos != std::string::npos) {
			std::cout << "RL WAITING fact found in loop clips gym" << std::endl;
			found_fact = true;
		}
		fact = fact->next();
	}

	if (!found_fact) {
		std::cout << "RL WAITING fact  was NOT found in loop clips gym" << std::endl;
	}

	clips.unlock();

	std::cout << "End Loop " << std::endl;
	sleep(200);
}

void
ClipsGymThread::finalize()
{
	//clips.lock();
	//clips->assert_fact("(executive-finalize)");
	//clips.unlock();
}

void
ClipsGymThread::assertRlGoalSelectionFact(std::string goalID)
{
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();

	clips->evaluate("(printout t \"In ClipsGymThread assertRlGoalSelectionFact: next goal " + goalID
	                + "\" crlf)");
	CLIPS::Value             v    = CLIPS::Value(goalID, CLIPS::TYPE_SYMBOL); //CLIPS::TYPE_STRING);
	CLIPS::Template::pointer temp = clips->get_template("rl-goal-selection");

	CLIPS::Fact::pointer fact = CLIPS::Fact::create(**clips, temp);
	fact->set_slot("next-goal-id", v);
	clips->assert_fact(fact);

	clips.unlock();
}

void
ClipsGymThread::clipsGymSleep(int milliseconds)
{
	logger->log_info(name(), "ClipsGym Sleeping for %i ms", milliseconds);
	std::this_thread::sleep_for(milliseconds * 1ms);
}

//std::string
ClipsObservationInfo
ClipsGymThread::step(std::string next_goal)
{
	std::cout << "In ClipsGymThread step function" << std::endl;
	std::cout << "next_goal from python: " << next_goal << std::endl;
	ClipsObservationInfo obs_info = ClipsObservationInfo();
	obs_info.reward               = 0;

    logger->log_info(name(), "currentExecutableGoalsCG", currentExecutableGoals.size());
	//Transform string to goal
	//std::string n_goal = "TOWER-C1#b#d#";
	//std::string goalID = getGoalId(next_goal);

	std::string goalID = getGoalIdByString(currentExecutableGoals, next_goal);

	if (goalID == "") {
		std::cout << "Goal id not found!" << std::endl;
		std::string env_state = create_rl_env_state_from_facts();
		std::cout << "End Clips Gym Thread step function" << std::endl;

		obs_info.observation = env_state;
		return obs_info;
	}
	assertRlGoalSelectionFact(goalID);
	std::cout << "fact asserted, start running clips" << std::endl;

	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//Check frequently if the selected goal is evaluated
	bool  env_feedback = false;
	float speed        = config->get_float("/env/speed");
	float max_time     = config->get_float("/env/step/max_time");  //60 sec without speedup
	float wait_time    = config->get_float("/env/step/wait_time"); //5 sec
	if (speed != 0.0 || speed != 1.0) {
		wait_time = wait_time / speed;
	}
	int  elapsed_time        = 0;
	bool check_for_game_over = false;
	while (!env_feedback && elapsed_time < max_time) {
		std::this_thread::sleep_for(wait_time * 1000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping Step Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl    = fact->get_template();
			std::size_t              found   = tmpl->name().find("rl-finished-goal");
			std::size_t              wm_fact = tmpl->name().find("wm-fact");
			if (found != std::string::npos) {
				std::string goalID  = getClipsSlotValuesAsString(fact->slot_value("goal-id"));
				std::string outcome = getClipsSlotValuesAsString(fact->slot_value("outcome"));
				std::string result  = getClipsSlotValuesAsString(fact->slot_value("result"));
				std::cout << "In ClipsGymThread step: Goal: " << goalID
				          << " is evaluated with outcome: " << outcome << " result: " << result
				          << std::endl;

				obs_info.reward = fact->slot_value("result")[0].as_integer();
				//TODO check why its no valid syntax
				obs_info.info = "Goal-id " + goalID + " Outcome " + outcome; //TODO add also goal-id to info
				//TODO: check if goalID is the same
				env_feedback = true;
				fact->retract();
				std::cout << "In ClipsGymThread step: after retracting rl-finished-goal fact" << std::endl;
				break;
			} else if (check_for_game_over && wm_fact != std::string::npos) {
				//(wm-fact (id "/refbox/phase") (key refbox phase) (type UNKNOWN) (is-list FALSE) (value POST_GAME) (values))
				std::string key   = getClipsSlotValuesAsString(fact->slot_value("key"));
				std::string value = getClipsSlotValuesAsString(fact->slot_value("value"));
				//key: refbox#phase id:/refbox/phase value: PRODUCTION
				if (key == "refbox#phase" && (value == "POST_GAME" || value == "SETUP")) {
					obs_info.info = "Game Over";
					logger->log_info(name(), "Step Function: %s %s Game Over", key.c_str(), value.c_str());
					env_feedback = true;
					break;
				}
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
		elapsed_time += wait_time;
		check_for_game_over = true;
	}
	if (!env_feedback) {
		logger->log_error(name(), "Ending step function without finished goal!!!");
	}
	std::string env_state = create_rl_env_state_from_facts();

	std::cout << "End Clips Gym Thread step function" << std::endl;

	obs_info.observation = env_state;
	//ClipsObservationInfo obs_info = ClipsObservationInfo(env_state);
	return obs_info; //env_state;
}

py::list
ClipsGymThread::getGoalClassList()
{
	std::vector<std::string> goalClasses = config->get_strings("/goal-space/classes");
	py::list                 g_classes;
	for (std::string s : goalClasses) {
		g_classes.append((py::str)s);
	}
	return g_classes;
}

py::list
ClipsGymThread::generateActionSpace()
{
	std::cout << "Clips Gym Thread generateActionSpace start" << std::endl;
	//TODO temporär drin
	std::vector<std::string> formulated_goals = getAllFormulatedGoals();
	for (std::string s : formulated_goals) {
		std::cout << s << std::endl;
	}

	//TODO: implement generation based on clips goals
	/*std::string space[] = {"TOWER-C1#buttom#a#top#c",
	                       "TOWER-C1#buttom#b#top#d",
	                       "TOWER-C1#buttom#e#top#d",
	                       "TOWER-C2#buttom#b#middle#d#top#e"}; //, "TOWER-C1#buttom#a#top#e"};
*/
	std::string space[] = {"ENTER-FIELD",
	                       "BUFFER-CAP#cap-color#CAP_BLACK",
	                       "BUFFER-CAP#cap-color#CAP_GREY",
	                       "MOUNT-CAP#wp-loc#C-BS",
	                       "MOUNT-CAP#wp-loc#C-CS1",
	                       "MOUNT-CAP#wp-loc#C-CS2",
	                       "MOUNT-CAP#wp-loc#C-RS1",
	                       "MOUNT-CAP#wp-loc#C-RS2",
	                       "MOUNT-CAP#wp-loc#C-SS",
	                       "DISCARD#wp-loc#C-BS",
	                       "DISCARD#wp-loc#C-CS1",
	                       "DISCARD#wp-loc#C-CS2",
	                       "DISCARD#wp-loc#C-DS",
	                       "DISCARD#wp-loc#C-RS1",
	                       "DISCARD#wp-loc#C-RS2",
	                       "DISCARD#wp-loc#C-SS",
	                       "PAY-FOR-RINGS-WITH-BASE#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-BASE#target-mps#C-RS2",
	                       "PAY-FOR-RINGS-WITH-CAP-CARRIER#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-CAP-CARRIER#target-mps#C-RS2",
	                       "PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF#target-mps#C-RS1",
	                       "PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF#target-mps#C-RS2",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_BLUE#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_GREEN#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_ORANGE#wp-loc#C-RS2",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-BS",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-RS1",
	                       "MOUNT-RING#ring-color#RING_YELLOW#wp-loc#C-RS2",
	                       "DELIVER",
	                       "WAIT-NOTHING-EXECUTABLE",
	                       "MOVE-OUT-OF-WAY"};

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

void
ClipsGymThread::log(std::string log_msg)
{
	if (log_msg != "") {
		logger->log_info(name(), "RL: %s", log_msg.c_str());
	}
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
	/*std::cout << "ClipsGym: splitted action " << std::endl;
	for (std::string s : action_splitted) {
		std::cout << s << std::endl;
	}*/
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact   = clips->get_facts();
	std::string          goalID = "";
	//std::string          allGoals = "{ ";

	logger->log_info(name(), "RL: found goals / actions:");
	while (fact) {
		CLIPS::Template::pointer tmpl = fact->get_template();
		// TODO: this can be improved
		std::size_t found = tmpl->name().find("goal");
		std::size_t meta  = tmpl->name().find("goal-meta");
		if (found != std::string::npos && meta == std::string::npos) {
			/*
			Slot names: id, class,type, sub-type, parent, mode, outcome, warning, error, message,
						priority, params, meta, meta-fact, meta-template, required-resources, acquired-resources,
						committed-to,verbosity,is-executable,
			Class: TOWER-C1, params: buttom,b,top,d
			*/
			std::string current_class = getClipsSlotValuesAsString(fact->slot_value("class"));

			if (current_class == action_splitted[0]) {
				std::cout << "Equal goal class: " << current_class << std::endl;
				std::string current_params =
				  getParamsClipsSlotValuesOfGoalAsString(current_class, fact->slot_value("params"));

				std::string current_action = current_class + "#" + current_params;
				logger->log_info(name(), "RL: %s", current_action.c_str());
				if (current_action == action) {
					goalID = getClipsSlotValuesAsString(fact->slot_value("id"));
					std::cout << "correct class and params! GoalID is: " << goalID << std::endl;
					break;
				}
			}
		}
		fact = fact->next();
	}
	std::cout << "Finished passing all goals" << std::endl;
	logger->log_info(name(), "RL: GoalID %s", goalID.c_str());

	clips.unlock();
	return goalID;
}

std::string
ClipsGymThread::getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values)
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
ClipsGymThread::filterParams(GoalAction *goal)
{
	std::cout << "param string before " << goal->getParamsString() << std::endl;
	py::dict paramNameType = getParamsNameTypeMapOfGoal(goal->getClass());

	auto getType = paramNameType.attr("get");

	auto params = goal->getParams();
	//auto filtered = std::erase_if(params, [](Param p){ return (paramNameType.attr("get")(p) == py::none);} );

	auto iterator = std::remove_if(params->begin(), params->end(), [&](const Param &p) {
		auto n = getType(p.name, "");
		//std::cout << "Filter Params " << n << std::endl;
		//logger->log_info(name(), "RL: filterParams %s", );
		//params->remove_if([&](const Param& p){
		return (getType(p.name).is(py::none()));
	});
	params->erase(iterator, params->end());

	//std::cout << "Filter Params: " << goal->getParamsString() << std::endl;
	//goal.setParams(&params);
}

std::string
ClipsGymThread::getGoalIdByString(std::vector<GoalAction> goals, std::string goal_str)
{
	logger->log_info(name(), "getGoalIdByString %s", goal_str.c_str());
	for (GoalAction g : goals) {
		logger->log_info(name(), "Before filter: %s", g.getGoalString().c_str());
		filterParams(&g);
		logger->log_info(name(), "After filter: %s", g.getGoalString().c_str());
		if (g.getGoalString() == goal_str) {
			return g.getId();
		}
	}
	return "";
}

std::string
ClipsGymThread::getParamsClipsSlotValuesOfGoalAsString(std::string               goalClass,
                                                       std::vector<CLIPS::Value> slot_values)
{
	std::cout << "\nIn getParamsClipsSlotValus \n" << std::endl;
	py::dict paramsNameTyeMap = getParamsNameTypeMapOfGoal(goalClass);
	auto     keysFunc         = paramsNameTyeMap.attr("keys");
	auto     keys             = keysFunc();
	py::print(keys);
	//auto params = keys.cast<std::vector<std::string>>();

	std::string paramsChain = "";

	std::vector<std::string> values_as_string;
	for (std::size_t i = 0; i < slot_values.size(); i++) {
		std::string value = "";
		auto        v     = slot_values[i];
		switch (v.type()) {
		case CLIPS::TYPE_FLOAT: value = std::to_string(v.as_float()); break;
		case CLIPS::TYPE_INTEGER: value = std::to_string(v.as_integer()); break;
		default: value = v.as_string();
		}
		values_as_string.push_back(v);
	}

	for (auto k : keys) //auto p : params)
	{
		std::string p = k.cast<std::string>();
		std::cout << "Param: " << p << std::endl;
		std::vector<std::string>::iterator loc =
		  std::find(values_as_string.begin(), values_as_string.end(), p);
		if (loc != values_as_string.end()) {
			std::advance(loc, 1);
			paramsChain += '#' + p + '#' + (std::string)*loc;
		}
	}

	return paramsChain;
}

static string
getClipsValueString(CLIPS::Value v)
{
	std::string value = "";
	switch (v.type()) {
	case CLIPS::TYPE_FLOAT: value = std::to_string(v.as_float()); break;
	case CLIPS::TYPE_INTEGER: value = std::to_string(v.as_integer()); break;
	default: value = v.as_string();
	}
	return value;
}

std::list<Param>
ClipsGymThread::extractGoalParamsFromClipsValues(std::vector<CLIPS::Value> slot_values)
{
	std::list<Param> params;
	for (std::size_t i = 0; i + 1 < slot_values.size(); i++) {
		//std::string value = "";
		//auto        v     = slot_values[i];
		std::string p_name  = getClipsValueString(slot_values[i]);
		std::string p_value = getClipsValueString(slot_values[i + 1]);
		Param       p       = Param(p_name, p_value);
		i++;
		//values_as_string.push_back(v);
		params.push_back(p);
	}
	return params;
}

//get key value map of param-name and param-type
//std::map<std::string,std::string>
py::dict
ClipsGymThread::getParamsNameTypeMapOfGoal(std::string goalClass)
{
	//TODO check if goalClass not found
	if (goalClassParamsAndParamTypeMap.find(goalClass) != goalClassParamsAndParamTypeMap.end()) {
		return goalClassParamsAndParamTypeMap[goalClass];
	}

	//std::map<std::string, std::string> mapParamsNameType;
	py::dict mapParamsNameType;

	try {
		std::vector<std::string> params = config->get_strings("/param-names/" + goalClass);

		for (std::string param : params) {
			std::string cfg_param = "/param-names/param-types/" + param;
			//std::cout << "is string: " << config->is_string(cfg_param) << std::endl;

			std::string p_type = config->get_string(cfg_param);
			//std::cout << param << ": " << p_type << std::endl;
			//mapParamsNameType.insert(std::pair<std::string, std::string>(param,p_type));

			mapParamsNameType[(py::str)param] = (py::str)p_type;
		}

		//auto sorted_func = py::module_::import("sorted");
		//auto paramsMap = sorted_func(mapParamsNameType);
		//goalClassParamsAndParamTypeMap.insert(
		//	  		std::pair<std::string, std::map<std::string,std::string>(goalClass, paramsMap));
		goalClassParamsAndParamTypeMap.insert(
		  std::pair<std::string, py::dict>(goalClass, mapParamsNameType));
	} catch (Exception &e) {
		logger->log_error(name(), "getParamsNameTypeMapOfGoal %s", e.what());
	}

	return mapParamsNameType;
}

std::vector<std::string>
ClipsGymThread::getAllFormulatedGoals()
{
	std::cout << "In ClipsGymThread get all formulated goals" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact = clips->get_facts();
	//std::string              goalID = "";
	std::vector<std::string> maskedGoals;
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("goal");

		if (found != std::string::npos) {
			std::string s           = "mode";
			std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
			//std::cout << slot_values << std::endl;
			if (slot_values == "FORMULATED") {
				std::string goal_class = getClipsSlotValuesAsString(fact->slot_value("class"));
				//std::cout << goal_class << std::endl;

				std::string goal_params = getClipsSlotValuesAsString(fact->slot_value("params"));
				//std::cout << goal_params << std::endl;
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
ClipsGymThread::waitForFreeRobot()
{
	logger->log_info(name(), "RL: Waiting for finished selection... %d ", lockRobot);
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	while(lockRobot){
		py::gil_scoped_release release;
		std::this_thread::sleep_for(1000ms);
		py::gil_scoped_acquire acquire;
	}
	logger->log_info(name(), "RL: Waiting for free robot... ");
	lockRobot = true;
	bool robotFound = false;

	freeRobot = "None";
	while(!robotFound) {
		//py::gil_scoped_release release;
		std::this_thread::sleep_for(100ms);
		//py::gil_scoped_acquire acquire;
		clips.lock();
		CLIPS::Fact::pointer fact = clips->get_facts();
		while(fact) {
			CLIPS::Template::pointer 	tmpl = fact->get_template();
			std::string					fact_name = tmpl->name();
			if(fact_name == "wm-fact" && getClipsSlotValuesAsString(fact->slot_value("key")).find("robot-waiting") != std::string::npos) {
				std::string key = getClipsSlotValuesAsString(fact->slot_value("key"));
				size_t pos = key.find("#r#");
				freeRobot = key.substr(pos+3);
				
				CLIPS::Fact::pointer g_fact = clips->get_facts();
				while (g_fact) {
					CLIPS::Template::pointer g_tmpl  = g_fact->get_template();
					std::string              g_fact_name = g_tmpl->name();
					CLIPS::Fact::pointer 	 gm = NULL;
					if (g_fact_name == "goal") {
						std::string goalid = getClipsSlotValuesAsString(g_fact->slot_value("id"));
						//logger->log_info(name(), "RL: Goal found with id %s", goalid.c_str());
						CLIPS::Fact::pointer gm_fact = clips->get_facts();
						while (gm_fact) {				
							CLIPS::Template::pointer 	gm_tmpl = gm_fact->get_template();
							std::string					gm_fact_name = gm_tmpl->name();
							if (gm_fact_name == "goal-meta"){
								std::string gm_goalid = getClipsSlotValuesAsString(gm_fact->slot_value("goal-id"));
								//logger->log_info(name(), "RL: Goal-meta fact found with id %s", gm_goalid.c_str());
								if (gm_goalid == goalid) {
									
									gm = gm_fact;
									break;
								}
							}
							gm_fact = gm_fact->next();
						}
						//logger->log_info(name(), "RL: Correct Goal-meta fact found");
						std::string assigned_to	  = getClipsSlotValuesAsString(gm->slot_value("assigned-to"));
						std::string mode          = getClipsSlotValuesAsString(g_fact->slot_value("mode"));
						std::string is_executable = getClipsSlotValuesAsString(g_fact->slot_value("is-executable"));

						if (mode == "FORMULATED" && is_executable == "TRUE" && assigned_to == freeRobot) {
							robotFound = true;
							break;
						}
					}
					g_fact = g_fact->next();
				}
				if(robotFound){
					break;
				}
			}
			//logger->log_info(name(), "RL: Waiting for free robot...");
			fact = fact->next();
		}
		clips.unlock();

	}
	logger -> log_info(name(), "Free Robot %s", freeRobot.c_str());
}

void
ClipsGymThread::unlockRobot()
{
	freeRobot = "None";
	lockRobot = false;
}

std::vector<GoalAction>
ClipsGymThread::getExecutableGoalsForFreeRobot()
{
	std::cout << "In ClipsGymThread get all executable goals for free robot" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	logger->log_info(name(), "RL: Executable Goals for %s", freeRobot.c_str());
	clips.lock();
	bool goalsExecutable = false;
	std::vector<GoalAction> maskedGoals;

	while (!goalsExecutable){
		clips.unlock();
		//logger->log_info(name(), "RL: No Executable goals found, retrying...");
		std::this_thread::sleep_for(10ms);
		clips.lock();
		CLIPS::Fact::pointer fact = clips->get_facts();
		
		while (fact) {
			CLIPS::Template::pointer tmpl  = fact->get_template();
			std::string              fact_name = tmpl->name();
			CLIPS::Fact::pointer 	 gm = NULL;
			if (fact_name == "goal") {
				std::string goalid = getClipsSlotValuesAsString(fact->slot_value("id"));
				//logger->log_info(name(), "RL: Goal found with id %s", goalid.c_str());
				CLIPS::Fact::pointer gm_fact = clips->get_facts();
				while (gm_fact) {				
					CLIPS::Template::pointer 	gm_tmpl = gm_fact->get_template();
					std::string					gm_fact_name = gm_tmpl->name();
					if (gm_fact_name == "goal-meta"){
						std::string gm_goalid = getClipsSlotValuesAsString(gm_fact->slot_value("goal-id"));
						//logger->log_info(name(), "RL: Goal-meta fact found with id %s", gm_goalid.c_str());
						if (gm_goalid == goalid) {
							
							gm = gm_fact;
							break;
						}
					}
					gm_fact = gm_fact->next();
				}
				//logger->log_info(name(), "RL: Correct Goal-meta fact found");
				std::string assigned_to	  = getClipsSlotValuesAsString(gm->slot_value("assigned-to"));
				std::string mode          = getClipsSlotValuesAsString(fact->slot_value("mode"));
				std::string is_executable = getClipsSlotValuesAsString(fact->slot_value("is-executable"));

				if (mode == "FORMULATED" && (is_executable == "FALSE" || assigned_to != freeRobot)) {
					std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));
					//logger->log_info(name(), "Goal is not executable: %s", goal_id.c_str());
				}
				if (mode == "FORMULATED" && is_executable == "TRUE" && assigned_to == freeRobot) {
					std::string goal_class      = getClipsSlotValuesAsString(fact->slot_value("class"));
					py::list    allowed_classes = getGoalClassList();
					auto        vec             = allowed_classes.cast<std::vector<std::string>>();
					std::vector<std::string>::iterator loc = std::find(vec.begin(), vec.end(), goal_class);
					if (loc == vec.end()) {
						logger->log_info(name(), "not in List %s", goal_class.c_str());
					} else {
						std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));
						GoalAction  goal    = GoalAction(goal_class, goal_id);

						std::list params = extractGoalParamsFromClipsValues(fact->slot_value("params"));
						goal.setParams(params);

						filterParams(&goal);
						std::cout << "Params String of goal action after Filter" << goal.getParamsString()
								<< std::endl;
						goalsExecutable = true;
						maskedGoals.push_back(goal);

						logger->log_info(name(),
										"RL: %s",
										goal.getGoalString().c_str());
					}
				}
			}
			fact = fact->next();
		}
		//logger->log_info(name(), "RL: Executable Goals found %d", goalsExecutable);
	}
	//std::cout<<maskedGoals <<std::endl;
	logger->log_info(name(), "RL: Finished passing all executable goals");
	clips.unlock();

	currentExecutableGoals = maskedGoals;

	return maskedGoals;
}



//std::vector<std::string>
std::vector<GoalAction>
ClipsGymThread::getAllFormulatedExecutableGoals()
{
	std::cout << "In ClipsGymThread get all executable goals" << std::endl;
	logger->log_info(name(), "RL: All formulated executable Goals environment ");
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer fact = clips->get_facts();
	//std::string              goalID = "";
	//std::vector<std::string> maskedGoals;
	std::vector<GoalAction> maskedGoals;
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("goal");

		if (found != std::string::npos) {
			std::string mode          = getClipsSlotValuesAsString(fact->slot_value("mode"));
			std::string is_executable = getClipsSlotValuesAsString(fact->slot_value("is-executable"));

			if (mode == "FORMULATED" && is_executable == "FALSE") {
				std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));
				logger->log_info(name(), "Goal is not executable: %s", goal_id.c_str());
			}
			//std::cout << slot_values << std::endl;
			if (mode == "FORMULATED" && is_executable == "TRUE") {
				std::string goal_class      = getClipsSlotValuesAsString(fact->slot_value("class"));
				py::list    allowed_classes = getGoalClassList();
				auto        vec             = allowed_classes.cast<std::vector<std::string>>();
				std::vector<std::string>::iterator loc = std::find(vec.begin(), vec.end(), goal_class);
				if (loc == vec.end()) {
					logger->log_info(name(), "not in List %s", goal_class.c_str());
				} else {
					std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));
					GoalAction  goal    = GoalAction(goal_class, goal_id);

					std::list params = extractGoalParamsFromClipsValues(fact->slot_value("params"));
					goal.setParams(params);

					filterParams(&goal);
					std::cout << "Params String of goal action after Filter" << goal.getParamsString()
					          << std::endl;

					//std::string goal_params = getClipsSlotValuesAsString(fact->slot_value("params"));

					//std::string goal_params =
					// getParamsClipsSlotValuesOfGoalAsString(goal_class, fact->slot_value("params"));

					//std::cout << goal_params << std::endl;
					//maskedGoals.push_back(goal_class + "#" + goal_params);
					maskedGoals.push_back(goal);

					logger->log_info(name(),
					                 "RL: %s",
					                 goal.getGoalString().c_str()); //"#", goal_params.c_str());
				}
			}
		}
		fact = fact->next();
	}
	//std::cout<<maskedGoals <<std::endl;
	logger->log_info(name(), "RL: Finished passing all executable goals");
	clips.unlock();

	currentExecutableGoals = maskedGoals;

	return maskedGoals;
}

void
ClipsGymThread::resetCX()
{
	std::cout << "In ClipsGymThread resetCX" << std::endl;

	logger->log_info(name(), "RL: resetCX start");
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//clips->clear(), clips->reset() haben zu coredumps geführt
	clips.lock();

	clips->assert_fact("(reset-game (stage STAGE-0) (stage-time (time)) )");
	//clips->assert_fact("(reset-domain-facts)");
	clips.unlock();
	//TODO add loop checking for reset done

	float speed     = config->get_float("/env/speed");
	float max_time  = config->get_float("/env/resetCX/max_time");  //45 sec without speedup
	float wait_time = config->get_float("/env/resetCX/wait_time"); //4 sec
	if (speed != 0.0 || speed != 1.0) {
		wait_time = wait_time / speed;
	}

	bool env_feedback = false;
	int  elapsed_time = 0;
	while (!env_feedback && elapsed_time < max_time) {
		std::this_thread::sleep_for(wait_time * 1000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping RESET Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl = fact->get_template();
			std::size_t found             = tmpl->name().find("reset-game-finished"); // TODO: rename this
			//std::size_t              found = tmpl->name().find("reset-domain-finish");
			if (found != std::string::npos) {
				logger->log_info(name(), "\nIn ClipsGymThread reset completed!!! \n");
				fact->retract();
				env_feedback = true;
				break;
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
		elapsed_time += wait_time;
	}
	logger->log_info(name(), "RL: Finished resetCX");
}

void
ClipsGymThread::clips_context_init(const std::string &env_name, LockPtr<CLIPS::Environment> &clips)
{
	std::cout << "Hello World!-from ClipsGymThread clips_context_init\n" << std::endl;
	clips_env_name  = env_name;
	envs_[env_name] = clips;
	logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();
	clips->evaluate("(printout t \"Hello from CLIPS aspect in ClipsGymThread \" crlf crlf)");

	clips->add_function("rl-loop-start",
	                    sigc::slot<void>(
	                      sigc::bind<0>(sigc::mem_fun(*this, &ClipsGymThread::rl_loop_start),
	                                    env_name)));

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
ClipsGymThread::getClipsEnv()
{
	fawkes::LockPtr<CLIPS::Environment> clips = envs_[clips_env_name];
	return clips;
}

std::vector<std::string>
//py::list
ClipsGymThread::getParamNameDomainObjectsComb(std::string param_name, std::string param_type)
{
	std::vector<std::string> paramNameDomainObjectsComb;
	//py::list paramNameDomainObjectsComb;
	auto domainObjects = getDomainObjects(param_type);
	for (std::string obj : domainObjects) {
		std::string comb = param_name + "#" + obj;
		paramNameDomainObjectsComb.push_back(comb);
		//paramNameDomainObjectsComb.append((py::str)comb);
	}
	return paramNameDomainObjectsComb;
}

/* Checks if paramTypeDomainObjectsMap contains entries for this type
*	yes: returns map entry
* 	no: return CX entries
*/
std::vector<std::string>
ClipsGymThread::getDomainObjects(std::string a_type)
{
	if (paramTypeDomainObjectsMap.find(a_type) != paramTypeDomainObjectsMap.end()) {
		return paramTypeDomainObjectsMap[a_type];
	} else {
		auto domainObjects = getDomainModelObjectsFromCX(a_type);
		if (domainObjects.size() > 0) {
			paramTypeDomainObjectsMap.insert(
			  std::pair<std::string, std::vector<std::string>>(a_type, domainObjects));
		}
		return domainObjects;
	}
}

//get all type instances / literals of this domain (ggfls. zwischenspeichern)
std::vector<std::string>
ClipsGymThread::getDomainModelObjectsFromCX(std::string a_type)
{
	std::cout << "clipsGymThread: getDomainModelObjectsFromCX start" << std::endl;
	logger->log_info(name(), "RL: getDomainModelObjects of type %s", a_type.c_str());
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	std::vector<std::string>            domainObjects; //= new std::vector<std::string>();
	clips.lock();
	CLIPS::Fact::pointer fact = clips->get_facts();

	while (fact) {
		/*CLIPS::Template::pointer tmpl = fact->get_template();
		std::string tmpl_name = tmpl->name();
		//std::size_t found = tmpl->name().find("domain-fact");
		tmpl_name: domain-object-type
		tmpl_name: domain-predicate
		tmpl_name: domain-operator-parameter
		tmpl_name: domain-operator
		
		if(domainObjects.end() == std::find(domainObjects.begin(), domainObjects.end(), tmpl_name))
		{
			domainObjects.push_back(tmpl_name);
			std::cout << "tmpl_name: " <<tmpl_name <<std::endl;
		}*/

		CLIPS::Template::pointer tmpl      = fact->get_template();
		std::string              tmpl_name = tmpl->name();

		std::size_t found = tmpl->name().find("domain-object");

		if (found != std::string::npos) {
			std::string obj_type = getClipsSlotValuesAsString(fact->slot_value("type"));
			if (obj_type == a_type) {
				std::string obj_name = getClipsSlotValuesAsString(fact->slot_value("name"));
				domainObjects.push_back(obj_name);
				//logger->log_info(name(), "RL: domain-object %s", obj_name.c_str());
			}
		}
		fact = fact->next();
	}
	clips.unlock();
	return domainObjects;
}

//std::vector<std::string>
py::dict
ClipsGymThread::getDomainPredicates()
{
	std::cout << "ClipsGymThread: getDomainPredicates start" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	std::vector<std::string>            domainObjects; //= new std::vector<std::string>();
	clips.lock();
	CLIPS::Fact::pointer fact = clips->get_facts();
	py::dict             mapPredicateParams;
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("domain-predicate");

		if (found != std::string::npos) {
			std::string name = getClipsSlotValuesAsString(fact->slot_value("name"));

			std::vector<CLIPS::Value> param_names = fact->slot_value("param-names");
			std::vector<CLIPS::Value> param_types = fact->slot_value("param-types");
			py::dict                  mapParamsNameType;
			for (size_t i = 0; i < param_names.size(); i++) {
				py::str param  = (py::str)(param_names[i].as_string());
				py::str p_type = (py::str)(param_types[i].as_string());
				//std::vector<std::string> domain_objects = getDomainObjects(clips_type.as_string());
				mapParamsNameType[param] = p_type;
			}
			mapPredicateParams[(py::str)name] = mapParamsNameType;
			//std::cout << "Predicate: " << name << std::endl;
		}
		fact = fact->next();
	}
	clips.unlock();
	//py::print(mapPredicateParams);
	return mapPredicateParams;
}

//ToDo return std::vector<string> literals
std::string
ClipsGymThread::create_rl_env_state_from_facts()
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
		//std::size_t found2 = tmpl->name().find("wm-fact");//"predicate");
		//std::size_t found3 = tmpl->name().find("goal");

		if (found != std::string::npos) {
			std::vector<std::string> slot_names = fact->slot_names();
			std::string              fact_value = "";
			for (std::string s : slot_names) {
				fact_value += " Slot " + s + ": ";
				//std::cout << "Slot name: " + s << std::endl;
				std::vector<CLIPS::Value> slot_values = fact->slot_value(s);
				std::string               value       = getClipsSlotValuesAsString(slot_values);

				//std::cout << value << std::endl;
				if (s == "name") {
					env_state_string += "\"" + value + "(";
				}
				if (s == "param-values") {
					env_state_string += value + ")\",";
				}
				fact_value += " " + value;
			}
			//std::cout << fact_value <<std::endl;
		}
		fact = fact->next();
	}
	env_state_string = env_state_string.substr(0, env_state_string.length() - 1) + "}";
	//std::cout << env_state_string << std::endl;
	std::cout << "Finished passing all facts " << std::endl;
	clips.unlock();
	std::cout << "Unlock clips done: Clips pointer refcount: " << clips.refcount() << std::endl;

	return env_state_string;
}
