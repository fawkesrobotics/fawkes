
/***************************************************************************
 *  plexil_thread.h - PLEXIL executive plugin
 *
 *  Created: Mon Aug 13 11:19:14 2018
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

#ifndef __PLUGINS_PLEXIL_PLEXIL_THREAD_H_
#define __PLUGINS_PLEXIL_PLEXIL_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <utils/time/time.h>

#include <memory>
#include <fstream>

#include <AdapterFactory.hh>

namespace PLEXIL {
  class ExecApplication;
}

class PlexilLogStreamBuffer;
class ClockPlexilTimeAdapter;
class LoggingPlexilAdapter;
class BehaviorEnginePlexilAdapter;
class ThreadNamePlexilAdapter;

class PlexilExecutiveThread
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
	PlexilExecutiveThread();
	virtual ~PlexilExecutiveThread();

	virtual void init();
	virtual void once();
	virtual void loop();
	virtual bool prepare_finalize_user();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

private:
	/// @cond INTERNAL
	//  Data structure for config parse result
	struct plexil_interface_config {
		std::string type;
		std::map<std::string, std::string> attr;
		std::map<std::string, std::string> args;

		struct verbatim_arg {
			std::string tag;
			bool        has_text;
			std::string text;
			std::map<std::string, std::string> attr;
		};
		std::map<std::string, verbatim_arg> verbatim_args;
	};
	/// @endcond

	std::map<std::string, plexil_interface_config>
	read_plexil_interface_configs(const std::string& config_prefix);
	void add_plexil_interface_configs(pugi::xml_node &parent,
	                                  const std::map<std::string,
	                                  PlexilExecutiveThread::plexil_interface_config> &configs,
	                                  const char* tag_name, const char* type_attr_name);

 private:
	std::string cfg_spec_;
	std::string cfg_plan_plx_;

	std::shared_ptr<PLEXIL::ExecApplication> plexil_;
	PLEXIL::ConcreteAdapterFactory<ClockPlexilTimeAdapter> *      clock_adapter_;
	PLEXIL::ConcreteAdapterFactory<LoggingPlexilAdapter> *        log_adapter_;
	PLEXIL::ConcreteAdapterFactory<BehaviorEnginePlexilAdapter> * be_adapter_;
	PLEXIL::ConcreteAdapterFactory<ThreadNamePlexilAdapter> *     thread_adapter_;

	std::shared_ptr<PlexilLogStreamBuffer> log_buffer_;
	std::shared_ptr<std::ostream>          log_stream_;

	std::shared_ptr<pugi::xml_document> plan_plx_;
	
};

#endif
