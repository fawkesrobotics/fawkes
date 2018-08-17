
/***************************************************************************
 *  be_adapter.h - PLEXIL adapter for the Behavior Engine
 *
 *  Created: Tue Aug 14 15:21:49 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLEXIL_SKILL_ADAPTER_H_
#define __PLUGINS_PLEXIL_SKILL_ADAPTER_H_

#include <InterfaceAdapter.hh>
#include <Value.hh>

#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <logging/logger.h>
#include <config/config.h>
#include <interfaces/SkillerInterface.h>

#include <mutex>

namespace fawkes {
	class ActionSkillMapping;
}

/** Interface adapter to provide logging facilities. */
class BehaviorEnginePlexilAdapter
	: public PLEXIL::InterfaceAdapter,
	  public fawkes::BlackBoardInterfaceListener
{
public:
	BehaviorEnginePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface);
	BehaviorEnginePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
	                   pugi::xml_node const xml);

	/// @cond DELETED
	BehaviorEnginePlexilAdapter() = delete;
	BehaviorEnginePlexilAdapter(const BehaviorEnginePlexilAdapter &) = delete;
	BehaviorEnginePlexilAdapter & operator=(const BehaviorEnginePlexilAdapter &) = delete;
	/// @endcond

	virtual ~BehaviorEnginePlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
  void invokeAbort(PLEXIL::Command *cmd);

	virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

private:
	struct skill_config {
		/// Specify a skill argument.
		struct skill_argument {
			/// Name of skill argument
			std::string name;
			/// Expected type of skill argument
			PLEXIL::ValueType type;
		};
	
		std::string name;
		std::vector<skill_argument> args;
		std::string template_str;
	};

	std::string format_skillstring(const std::vector<PLEXIL::Value>& values);
	std::string map_skillstring(const std::string& name,
	                            const skill_config& skill_config,
	                            const std::vector<PLEXIL::Value>& values);
	void call_skill(const std::string& skill_string, PLEXIL::Command* cmd);

private:
	fawkes::Configuration *     config_;
	fawkes::Logger *            logger_;
	fawkes::BlackBoard *        blackboard_;
	fawkes::SkillerInterface *  skiller_if_;

	std::shared_ptr<fawkes::ActionSkillMapping> action_skill_mapping_;

	std::mutex                  exec_mutex_;
	
	std::string                 skill_string_;
	unsigned int                skill_msgid_;

	PLEXIL::Command *           current_cmd_;

	std::map<std::string, skill_config> cfg_skills_;

};

extern "C" {
  void initBehaviorEngineAdapter();
}

#endif
