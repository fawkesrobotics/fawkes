/***************************************************************************
 *  exog_manager.h - Insert exog actions into Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
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

#ifndef FAWKES_GOLOGPP_BLACKBOARD_H_
#define FAWKES_GOLOGPP_BLACKBOARD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/thread.h>
#include <golog++/model/action.h>
#include <golog++/model/execution.h>

namespace gologpp {
class Type;
}

namespace fawkes_gpp {

class GologppThread;

///////////////////////////////////////////////////////////////////////////////
class ConfigError : public fawkes::Exception
{
public:
	ConfigError(const std::string &);
};

///////////////////////////////////////////////////////////////////////////////
class ExogManagerThread : public fawkes::BlackBoardAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::LoggingAspect,
                          public fawkes::Thread,
                          public fawkes::BlockedTimingAspect
{
public:
	ExogManagerThread(GologppThread *exec_thread);

	virtual void init() override;
	virtual void finalize() override;

	static const std::string cfg_prefix;

private:
	void                                     exog_queue_push(gologpp::shared_ptr<gologpp::ExogEvent>);
	gologpp::shared_ptr<gologpp::ExogAction> find_mapped_exog(const std::string &mapped_name);

	///////////////////////////////////////////////////////////////////
	class BlackboardEventHandler
	{
	public:
		BlackboardEventHandler(fawkes::BlackBoard *,
		                       gologpp::shared_ptr<gologpp::ExogAction>,
		                       ExogManagerThread &exog_mgr);

		gologpp::shared_ptr<gologpp::ExogEvent> make_exog_event(fawkes::Interface *) const;

	protected:
		fawkes::BlackBoard *                              blackboard_;
		gologpp::shared_ptr<gologpp::ExogAction>          target_exog_;
		std::unordered_map<std::string, gologpp::arity_t> fields_order_;
		ExogManagerThread &                               exog_manager_;
	};

	///////////////////////////////////////////////////////////////////
	class InterfaceWatcher : public BlackboardEventHandler, public fawkes::BlackBoardInterfaceListener
	{
	public:
		InterfaceWatcher(fawkes::BlackBoard *,
		                 const std::string &id,
		                 gologpp::shared_ptr<gologpp::ExogAction>,
		                 ExogManagerThread &exog_mgr);
		virtual ~InterfaceWatcher() override;

		virtual void bb_interface_data_changed(fawkes::Interface *) throw() override;

	private:
		fawkes::Interface *iface_;
	};

	//////////////////////////////////////////////////////////////////
	class PatternObserver : public BlackboardEventHandler, public fawkes::BlackBoardInterfaceObserver
	{
	public:
		PatternObserver(fawkes::BlackBoard *,
		                const std::string &pattern,
		                gologpp::shared_ptr<gologpp::ExogAction>,
		                ExogManagerThread &exog_mgr);
		virtual ~PatternObserver() override;

		virtual void bb_interface_created(const char *type, const char *id) throw() override;

	private:
		std::string pattern_;
	};

	//////////////////////////////////////////////////////////////////
	std::unordered_map<std::string, gologpp::shared_ptr<gologpp::ExogAction>> mapped_exogs_;
	std::vector<InterfaceWatcher>                                             watchers_;
	std::vector<PatternObserver>                                              observers_;
	GologppThread *                                                           exec_thread_;
};

} // namespace fawkes_gpp

#endif
