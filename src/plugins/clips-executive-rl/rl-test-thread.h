/***************************************************************************
 *  rl_test_thread.h -
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

//#pragma once
//#ifndef _PLUGINS_RL_TEST_THREAD_H_
//#define _PLUGINS_RL_TEST_THREAD_H_
//#include <bsoncxx/document/value.hpp>

#include <aspect/aspect_provider.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/RLAgentGoalSelectionInterface.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <py-guard.h>

//#include <boost/python.hpp>
//namespace py = boost::python;
#include <pybind11/embed.h>
namespace py = pybind11;

#include <clipsmm.h>
#include <future>

class RLTestThread : public fawkes::Thread,
                     public fawkes::LoggingAspect,
                     public fawkes::ConfigurableAspect,
                     public fawkes::BlackBoardAspect,
                     public fawkes::BlackBoardInterfaceListener,
                     public fawkes::CLIPSFeature,
                     public fawkes::CLIPSFeatureAspect
{
public:
	RLTestThread();

	void         init() override;
	virtual void finalize();
	virtual void loop();

	// for CLIPSFeature
	virtual void clips_context_init(const std::string                   &env_name,
	                                fawkes::LockPtr<CLIPS::Environment> &clips);
	virtual void clips_context_destroyed(const std::string &env_name);

	//bool        trainingRlAgent();

	bool startedTraining;

	bool              startedExecution;
	std::future<bool> training_done;

	std::future<bool> goal_prediction_done;

	std::string getGoalId(std::string action);
	void        assertRlGoalSelectionFact(std::string goalID);
	std::string create_rl_env_state_from_facts();

	std::string getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values);

	std::string executeRlAgent(std::string fact_string);

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	//py::object py_scope;
	PyGuard    *py_guard;
	std::string goal;
	std::string clips_env_name;

	fawkes::RLAgentGoalSelectionInterface                     *rl_gs_interface;
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;

	constexpr static char cfg_prefix_[] = "/plugins/rl-test/static/";

	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) noexcept;

	fawkes::LockPtr<CLIPS::Environment> getClipsEnv();
	void                                clips_rl_extract_executable_facts(std::string  env_name,
	                                                                      CLIPS::Value parent_goal_id,
	                                                                      std::string  to);
	void rl_goal_selection(std::string env_name, CLIPS::Value parent_goal_id, std::string to);

	/* config values*/
	bool        cfg_rl_agent_active;
	bool        cfg_training_mode;
	std::string cfg_rl_agent_name;
	std::string cfg_rl_agent_dir;
	std::string cfg_python_dir; //location of execution and trainings-script
	std::string cfg_execution_script;
	std::string cfg_env_script;
	std::string cfg_env_dir;
	std::string cfg_bin_plugins_dir;
	std::string cfg_bin_lib_dir;

	int count_startedExecution;
};
//#endif