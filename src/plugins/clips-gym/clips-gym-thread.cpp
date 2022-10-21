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

#include <iostream>
#include <regex>
#include <string>

//for calling boost python from plugin
//#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <core/threading/mutex_locker.h>
#include <pybind11/embed.h>

#include <boost/bind/bind.hpp>
#include <clipsmm.h>

//#include "clips-observation-info.h"

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

	py::class_<ClipsObservationInfo>(m, "ClipsObservationInfo").def(py::init<>())
		.def_readonly("observation", &ClipsObservationInfo::observation)
		.def_readonly("reward", &ClipsObservationInfo::reward)
		.def_readonly("done", &ClipsObservationInfo::done)
		.def_readonly("info", &ClipsObservationInfo::info);
		/* std::list<int> observation;
	int            reward;
	bool           done;
	std::string    info; */
	py::class_<ClipsGymThread>(m, "ClipsGymThread")
	  .def(py::init<>())
	  .def("getInstance", &ClipsGymThread::getInstance, py::return_value_policy::reference)
	  .def("step", &ClipsGymThread::step)
	  .def("resetCX", &ClipsGymThread::resetCX)
	  .def("create_rl_env_state_from_facts", &ClipsGymThread::create_rl_env_state_from_facts)
	  .def("getAllFormulatedGoals", &ClipsGymThread::getAllFormulatedGoals)
	  .def("getAllFormulatedExecutableGoals", &ClipsGymThread::getAllFormulatedExecutableGoals)
	  .def("generateActionSpace", &ClipsGymThread::generateActionSpace)
	  .def("generateObservationSpace", &ClipsGymThread::generateObservationSpace)
	  .def("getParamsNameTypeMapOfGoal", &ClipsGymThread::getParamsNameTypeMapOfGoal)
	  .def("getParamNameDomainObjectsComb", &ClipsGymThread::getParamNameDomainObjectsComb)
	  .def("getGoalClassList", &ClipsGymThread::getGoalClassList);
}

void
ClipsGymThread::init()
{
	std::cout << "Hello World!-from ClipsGymThread" << std::endl;

	
	//std::string temp = config->get_string("/goal-space/TOWER-C1/buttom");
	//std::cout << temp << std::endl;

	/* std::vector<std::string> goalClasses = config->get_strings("/goal-space/classes");
	for(std::string c: goalClasses)
	{
		std::cout << c <<std::endl;
		std::string temp = "/param-names/"+c;
		std::cout << temp << std::endl;

		std::vector<std::string> params = config->get_strings("/param-names/"+c);
		for(std::string p: params)
		{	
			std::string cfg_param = "/param-names/param-types/"+p;
			std::cout << "is string: " <<config->is_string(cfg_param) <<std::endl;
			
			if(config->is_list(cfg_param) == 1)
			{
				std::vector<std::string> type_list = config->get_strings(cfg_param);
				for(std::string t: type_list)
				{
					std::cout << p <<": " <<t<< std::endl;
				}
			}
			else if(config->is_string(cfg_param) == 1)
			{
				std::string type = config->get_string(cfg_param);
				std::cout << p <<": " <<type<< std::endl;
			}
		}
	} */
	//wakeup(); //activates loop
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

//std::string
ClipsObservationInfo
ClipsGymThread::step(std::string next_goal)
{
	std::cout << "In ClipsGymThread step function" << std::endl;
	std::cout << "next_goal from python: " << next_goal << std::endl;
	ClipsObservationInfo obs_info = ClipsObservationInfo();
	obs_info.reward = 0;
	//Transform string to goal
	//std::string n_goal = "TOWER-C1#b#d#";

	std::string goalID = getGoalId(next_goal);
	if(goalID =="")
	{
		std::cout << "Goal id not found!" <<std::endl;
		std::string env_state = create_rl_env_state_from_facts();
		std::cout << "End Clips Gym Thread step function" << std::endl;

		obs_info.observation = env_state;
		return obs_info;
	}
	assertRlGoalSelectionFact(goalID);
	std::cout << "fact asserted, start running clips" << std::endl;

	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//Check frequently if the selected goal is evaluated
	bool env_feedback = false;
	int max_time = 20;//seconds
	int elapsed_time=0;
	while (!env_feedback && elapsed_time < max_time) {
		int time = 5; //sec
		std::this_thread::sleep_for(time*1000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping Step Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl  = fact->get_template();
			std::size_t              found = tmpl->name().find("rl-finished-goal");
			if (found != std::string::npos) {
				std::string goalID  = getClipsSlotValuesAsString(fact->slot_value("goal-id"));
				std::string outcome = getClipsSlotValuesAsString(fact->slot_value("outcome"));
				std::string result = getClipsSlotValuesAsString(fact->slot_value("result"));
				std::cout << "In ClipsGymThread step: Goal: " << goalID
				          << " is evaluated with outcome: " << outcome 
						  << " result: " << result << std::endl;

				obs_info.reward = fact->slot_value("result")[0].as_integer();
				//TODO check why its no valid syntax
				obs_info.info = "Goal-id "+ goalID+ " Outcome " + outcome ;  //TODO add also goal-id to info
				//TODO: check if goalID is the same
				env_feedback = true;
				fact->retract();
				std::cout << "In ClipsGymThread step: after retracting rl-finished-goal fact" << std::endl;
				break;
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
		elapsed_time +=time;
	}
	std::string env_state = create_rl_env_state_from_facts();

	std::cout << "End Clips Gym Thread step function" << std::endl;

	obs_info.observation = env_state;
	//ClipsObservationInfo obs_info = ClipsObservationInfo(env_state);
	return obs_info;//env_state;
}


py::list
ClipsGymThread::expandGrid(std::map<std::string, std::vector<std::string>> map) //py::dict dictionary)
{
	py::list expandGridEntriey;
	//py::scoped_interpreter guard{};
	try{
		//py::object main_namespace = py::module_::import("__main__").attr("__dict__");
		//py::exec("import sys", main_namespace);
		//py::exec("print(\"Hello From python\")", main_namespace);

		py::object sys = py::module_::import("sys");
		py::exec("print(\"Hello From python - 1\")");


		//To Do guard
		//from itertools import product
		py::object product = py::module_::import("itertools").attr("product");
		py::object pd = py::module_::import("pandas");
		py::object dataFrame = pd.attr("DataFrame");
		py::object dictionary = py::cast(map);

		py::exec("print(\"Hello From python - 2 \")");
		py::exec("print(\"Hello From python - 2 - \", dictionary)", py::globals(),dictionary);
		
		py::exec(R"( df =  DataFrame([row for row in product(*dictionary.values())], 
						columns=dictionary.keys()))", py::globals(),dictionary); //, dictionary);
		
		py::exec("print(\"Hello From python - 3 - \", df)");
		py::exec(R"( x = df.to_string(header = False, index = False, index_names=False).split('\n') )");

		py::exec("print(\"Hello From python - 4 \")");
		py::exec(R"( expandGridEntriey = ['#', join(col.split()) for col in x] )"); //, expandGridEntriey); 
		//df.apply(lambda row: row[bla]+ )
		
		py::exec("print(\"Hello From python - 5\")");				
		py::exec("print(expandGridEntriey)"); //,expandGridEntriey);
	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}
	return expandGridEntriey;
}

py::list
ClipsGymThread::getGoalClassList()
{
	std::vector<std::string> goalClasses = config->get_strings("/goal-space/classes");
	py::list g_classes;
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
	for(std::string s : formulated_goals)
	{
		std::cout << s <<std::endl;
	}

	/*		
	std::map<std::string, std::vector<std::string>>possibleGoalsPartStrings;
	//py::dict possibleGoalsPartStrings;

	//Get Goal Class list
	std::vector<std::string> goalClasses = config->get_strings("/goal-space/classes");
	//TOWER-C1: {buttom: block, top: block}
	//for(std::string goalClass: goalClasses)
	//{
		std::string goalClass = goalClasses[0];
		std::cout << "ClipsGymThread generateActionSpace goal class:" << goalClass << std::endl;
		//get key value map of param-name and param-type
		std::map<std::string, std::string> mapParamNameType = getParamsNameTypeMapOfGoal(goalClass);
		//For each param of the goal do:
		for(std::pair<std::string, std::string> pair: mapParamNameType)
		{
			//{buttom#a, buttom#b,...}	
			std::vector<std::string> 
			paramNameDOComb = getParamNameDomainObjectsComb(pair.first, pair.second);
			//possibleGoalsPartStrings.push_back(paramNameDOComb);
			std::cout << "generateActionSpace " << paramNameDOComb[0] <<std::endl;
			possibleGoalsPartStrings.insert(std::pair<std::string, std::vector<std::string>>(pair.first,paramNameDOComb));
			//possibleGoalsPartStrings[(py::str)pair.first] = py::cast(paramNameDOComb);
		}
	//std::cout << "generateActionSpace " <<(std::string) (((py::list)possibleGoalsPartStrings["buttom"])[0])<<std::endl;
	//}

	auto goal_space = expandGrid(possibleGoalsPartStrings);
	*/

	//TODO: implement generation based on clips goals
	std::string space[] = {"TOWER-C1#buttom#a#top#c",
	                       "TOWER-C1#buttom#b#top#d",
	                       "TOWER-C1#buttom#e#top#d",
						   "TOWER-C2#buttom#b#middle#d#top#e"}; //, "TOWER-C1#buttom#a#top#e"};

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
	/*std::vector<std::string> action_splitted = splitActionToGoalParams(action);
	std::cout << "ClipsGym: splitted action " << std::endl;
	for (std::string s : action_splitted) {
		std::cout << s << std::endl;
	}*/
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
			std::string current_class = getClipsSlotValuesAsString(fact->slot_value("class"));
			std::string current_params = getClipsSlotValuesAsString(fact->slot_value("params"));
			
			std::string current_action = current_class + "#"+current_params;
			std::cout << "current action: " << current_action << std::endl;
			if(current_action == action)
			{
				goalID = getClipsSlotValuesAsString(fact->slot_value("id"));
				std::cout << "correct class and params! GoalID is: " << goalID << std::endl;
				break;
			}
			
			/*
			std::vector<std::string> slot_names         = fact->slot_names();
			bool                     correct_goal_class = false;
			bool                     correct_params     = true;
			std::string              temp_id            = "";
			for (std::string s : slot_names) {
				if (s == "class") {
					std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
					std::cout << "Class: " + slot_values << std::endl;
					if (action_splitted[0] == slot_values) {
						correct_goal_class = true;
					}
				}
				//TODO: what about two goals mapping one action, or action mapping no goals
				if (s == "params") {
					std::string slot_values = getClipsSlotValuesAsString(fact->slot_value(s));
					std::cout << "params: " << slot_values << std::endl;
					for (size_t i = 1; i < action_splitted.size(); i++) {
						//std::cout << "search for: " << action_splitted[i] << std::endl;
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
			*/
		}
		fact = fact->next();
	}
	std::cout << "Finished passing all goals" << std::endl;
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


//get key value map of param-name and param-type
//std::map<std::string,std::string>
py::dict
ClipsGymThread::getParamsNameTypeMapOfGoal(std::string goalClass)
{
	//TODO check if goalClass not found
	
	std::vector<std::string> params = config->get_strings("/param-names/"+goalClass);
	//std::map<std::string, std::string> mapParamsNameType; 
	py::dict mapParamsNameType; 
	
	for(std::string param: params)
	{	
		std::string cfg_param = "/param-names/param-types/"+param;
		std::cout << "is string: " <<config->is_string(cfg_param) <<std::endl;
		
		/*if(config->is_list(cfg_param) == 1)
		{
			std::vector<std::string> type_list = config->get_strings(cfg_param);
			mapParamsNameType.insert(std::pair<std::string, std::string>(param,p_splitted[1]));
			for(std::string t: type_list)
			{
				std::cout << param <<": " <<t<< std::endl;
			}
		}*/
	
		std::string p_type = config->get_string(cfg_param);
		std::cout << param <<": " <<p_type<< std::endl;
		//mapParamsNameType.insert(std::pair<std::string, std::string>(param,p_type));
		//mapParamsNameType.append(((py::str) param,(py::str) p_type));
		mapParamsNameType[(py::str) param] = (py::str) p_type;
	}
	


	//=new std::map<std::string, std::string>();
	/*std::string::size_type   begin = 0;
	for(std::string p:params)
	{
		std::cout << "ClipsGymThread params:" << p << std::endl;
		std::vector<std::string> p_splitted; // = new std::vector<std::string>();
		for (std::string::size_type end = 0; (end = p.find(":", end)) != std::string::npos; ++end) {
			std::string temp = p.substr(begin, end - begin);
			p_splitted.push_back(temp);
			std::cout <<temp<<std::endl;
			begin = end + 1;
		}
		mapParamsNameType.insert(std::pair<std::string, std::string>(p_splitted[0],p_splitted[1]));
	}*/
	return mapParamsNameType;
}

/* std::vector<std::string> *
ClipsGymThread::getParamsValues(std::vector<CLIPS::Value> slot_values, std::string goalClass)
{
	std::vector<std::string> keys = getParamsKeysOfGoal(goalClass);
	std::map<std::string, std::string> *paramsMap = new std::map<std::string, std::string>();
	for (std::size_t i = 0; i < slot_values.size(); i++) {
		auto v = slot_values[i];
		switch (v.type()) {
		case CLIPS::TYPE_FLOAT:
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
} */

std::vector<std::string>
ClipsGymThread::getAllFormulatedGoals()
{
	std::cout << "In ClipsGymThread get all formulated goals" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer     fact   = clips->get_facts();
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

std::vector<std::string>
ClipsGymThread::getAllFormulatedExecutableGoals()
{
	std::cout << "In ClipsGymThread get all executable goals" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	clips.lock();
	CLIPS::Fact::pointer     fact   = clips->get_facts();
	//std::string              goalID = "";
	std::vector<std::string> maskedGoals;
	while (fact) {
		CLIPS::Template::pointer tmpl  = fact->get_template();
		std::size_t              found = tmpl->name().find("goal");

		if (found != std::string::npos) {
			std::string mode = getClipsSlotValuesAsString(fact->slot_value("mode"));
			std::string is_executable = getClipsSlotValuesAsString(fact->slot_value("is-executable"));
			//std::cout << slot_values << std::endl;
			if (mode == "FORMULATED" && is_executable == "TRUE") {
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
	std::cout << "Finished passing all executable goals" << std::endl;
	clips.unlock();
	return maskedGoals;
}

void
ClipsGymThread::resetCX()
{
	std::cout << "In ClipsGymThread resetCX" << std::endl;
	if(resetCount == 0)
	{
		resetCount++;
		return;
	}

	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	//clips->clear(), clips->reset() haben zu coredumps geführt
	clips.lock();
	clips->assert_fact("(reset-domain-facts)");
	clips.unlock();
	//TODO add loop checking for reset done

	bool env_feedback = false;
	int max_time = 20;//seconds
	int elapsed_time=0;
	while (!env_feedback && elapsed_time < max_time) {
		int time = 5; //sec
		std::this_thread::sleep_for(time*1000ms);
		clips.lock();
		clips->evaluate("(printout t \"In Sleeping RESET Function \" crlf) ");
		CLIPS::Fact::pointer fact = clips->get_facts();

		while (fact) {
			CLIPS::Template::pointer tmpl  = fact->get_template();
			std::size_t              found = tmpl->name().find("reset-domain-finish");
			if (found != std::string::npos) {
				std::cout << "In ClipsGymThread reset completed!!! \n" << std::endl;
				fact->retract();
				env_feedback = true;
				break;
			}
			fact = fact->next();
		}

		//TODO: check outcome - set return 1 for completed and 0 otherwise

		clips.unlock();
		elapsed_time +=time;
	}

	std::cout << "Finished resetCX" << std::endl;
	resetCount++;
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
	fawkes::LockPtr<CLIPS::Environment> clips = envs_[clisp_env_name];
	return clips;
}


std::vector<std::string> 
//py::list
ClipsGymThread::getParamNameDomainObjectsComb(std::string param_name, std::string param_type)
{
	std::vector<std::string> paramNameDomainObjectsComb;
	//py::list paramNameDomainObjectsComb;
	auto domainObjects = getDomainObjects(param_type);
	for(std::string obj : domainObjects)
	{

		std::string comb = param_name + "#" +obj;
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
	if(paramTypeDomainObjectsMap.find(a_type) != paramTypeDomainObjectsMap.end())
	{
		return paramTypeDomainObjectsMap[a_type];
	}
	else
	{
		auto domainObjects = getDomainModelObjectsFromCX(a_type);
		paramTypeDomainObjectsMap.insert(std::pair<std::string,std::vector<std::string>>(a_type, domainObjects));
		return domainObjects;
	}
}

//get all type instances / literals of this domain (ggfls. zwischenspeichern)
std::vector<std::string>
ClipsGymThread::getDomainModelObjectsFromCX(std::string a_type)
{
	std::cout << "clipsGymThread: getDomainModelObjectsFromCX start" << std::endl;
	fawkes::LockPtr<CLIPS::Environment> clips = getClipsEnv();
	std::vector<std::string> domainObjects; //= new std::vector<std::string>();
	clips.lock();
	CLIPS::Fact::pointer fact             = clips->get_facts();
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


		CLIPS::Template::pointer tmpl = fact->get_template();
		std::size_t found = tmpl->name().find("domain-object");

		if (found != std::string::npos) {

			std::string obj_type =getClipsSlotValuesAsString( fact->slot_value("type"));
			if(obj_type == a_type)
			{
				std::string name = getClipsSlotValuesAsString(fact->slot_value("name"));
				domainObjects.push_back(name);
				std::cout << obj_type << ": " <<name << std::endl;
			}
			/*std::vector<std::string> slot_names = fact->slot_names();
			for(std::string s: slot_names)
			{
				//std::cout << "Slot name: " + s << std::endl;
				std::vector<CLIPS::Value> slot_values = fact->slot_value(s);
				std::string               value       = getClipsSlotValuesAsString(slot_values);

				std::cout <<  " Slot " + s + ": "+value <<std::endl;
			}*/
		}
		fact = fact->next();
	}
	clips.unlock();
	return domainObjects;
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
