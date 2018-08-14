
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
#include <interfaces/SkillerInterface.h>

#include <mutex>

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
	std::string format_skillstring(const std::vector<PLEXIL::Value>& values);

private:
	fawkes::Logger *            logger_;
	fawkes::BlackBoard *        blackboard_;
	fawkes::SkillerInterface *  skiller_if_;

	std::mutex                  exec_mutex_;
	
	std::string                 skill_string_;
	unsigned int                skill_msgid_;

	PLEXIL::Command *           current_cmd_;
};

extern "C" {
  void initBehaviorEngineAdapter();
}

#endif // POSIX_TIME_ADAPTER_H
