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
//#include <plugins/clips/aspect/clips_feature.h>

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <interfaces/RLAgentGoalSelectionInterface.h>
#include <plugins/clips/aspect/clips_feature.h>

//#include <boost/python.hpp>
//namespace py = boost::python;
#include <pybind11/embed.h>
namespace py = pybind11;

//#include <plugins/clips/aspect/clips.h>
//#include <plugins/robot-memory/aspect/robot_memory_aspect.h>

#include <clipsmm.h>

//

//						 public fawkes::CLIPSFeature,
//                     	 public fawkes::CLIPSFeatureAspect
/* 						 public fawkes::AspectProviderAspect,
						public fawkes::RobotMemoryAspect,
						 */

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

	void        trainingRlAgent();
	std::string executeRlAgent(std::string facts);

	bool startedTraining;
	//py::object main_namespace;

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::map<std::string, fawkes::LockPtr<CLIPS::Environment>> envs_;
	//CLIPS::Values clips_now();

	//fawkes::ExecutionTimeEstimatorManager       rl_test_manager_;
	//fawkes::ExecutionTimeEstimatorsAspectIniFin provider_inifin_;
	constexpr static char cfg_prefix_[] = "/plugins/rl-test/static/";

	fawkes::RLAgentGoalSelectionInterface *rl_gs_interface;
	std::string                            goal;

	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message   *message) noexcept;

	fawkes::LockPtr<CLIPS::Environment> getClipsEnv(std::string env_name);
	void                                clips_rl_extract_executable_facts(std::string  env_name,
	                                                                      CLIPS::Value parent_goal_id,
	                                                                      std::string  to);
	void        rl_goal_selection(std::string env_name, CLIPS::Value parent_goal_id, std::string to);
	std::string create_rl_env_state_from_facts(std::string env_name);

	std::string clipsValueToString(CLIPS::Value v);
};
//#endif