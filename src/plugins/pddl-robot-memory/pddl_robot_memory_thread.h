
/***************************************************************************
 *  pddl_robot_memory_thread.h - pddl_robot_memory
 *
 *  Plugin created: Thu Oct 13 13:34:05 2016

 *  Copyright  2016  Frederik Zwilling
 *
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

#ifndef _PLUGINS_PDDL_ROBOT_MEMORYTHREAD_H_
#define _PLUGINS_PDDL_ROBOT_MEMORYTHREAD_H_

#include "interfaces/PddlGenInterface.h"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <core/threading/thread.h>
#include <ctemplate/template.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>

#include <bsoncxx/document/view.hpp>
#include <string>

namespace fawkes {
}

class PddlRobotMemoryThread : public fawkes::Thread,
                              public fawkes::LoggingAspect,
                              public fawkes::ConfigurableAspect,
                              public fawkes::BlackBoardAspect,
                              public fawkes::RobotMemoryAspect,
                              public fawkes::BlackBoardInterfaceListener
{
public:
	PddlRobotMemoryThread();

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	fawkes::PddlGenInterface *gen_if;

	std::string collection;
	std::string input_path;
	std::string output_path;
	std::string goal;
	uint32_t    start_time;

	void fill_dict_from_document(ctemplate::TemplateDictionary *dict,
	                             const bsoncxx::document::view &obj,
	                             std::string                    prefix = "");
	void generate();

	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message *  message) noexcept;
};

#endif
