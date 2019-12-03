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

#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <config/config.h>
#include <core/threading/thread.h>
#include <golog++/model/action.h>
#include <golog++/model/execution.h>
#include <logging/logger.h>

namespace gologpp {
class Type;
}

namespace fawkes {
namespace gpp {

class GologppThread;

///////////////////////////////////////////////////////////////////////////////
class ConfigError : public Exception
{
public:
	ConfigError(const std::string &);
};

///////////////////////////////////////////////////////////////////////////////
class ExogManager
{
public:
	ExogManager(GologppThread *exec_thread,
	            Configuration *,
	            const std::string &cfg_prefix,
	            BlackBoard *,
	            Logger *);

	const char *name();

private:
	void                                     exog_queue_push(gologpp::shared_ptr<gologpp::ExogEvent>);
	gologpp::shared_ptr<gologpp::ExogAction> find_mapped_exog(const std::string &mapped_name);

	///////////////////////////////////////////////////////////////////
	class BlackboardEventHandler
	{
	public:
		BlackboardEventHandler(BlackBoard *                             bb,
		                       gologpp::shared_ptr<gologpp::ExogAction> exog,
		                       ExogManager &                            exog_mgr);
		BlackboardEventHandler(const BlackboardEventHandler &) = delete;
		BlackboardEventHandler &operator=(const BlackboardEventHandler &) = delete;
		BlackboardEventHandler(BlackboardEventHandler &&)                 = default;

		gologpp::shared_ptr<gologpp::ExogEvent> make_exog_event(Interface *) const;

		static std::string extract_type_name(const std::string &iface_uid);
		static std::string extract_id(const std::string &iface_uid);

	protected:
		BlackBoard *                                      blackboard_;
		gologpp::shared_ptr<gologpp::ExogAction>          target_exog_;
		std::unordered_map<std::string, gologpp::arity_t> fields_order_;
		ExogManager &                                     exog_manager_;
	};

	///////////////////////////////////////////////////////////////////
	class InterfaceWatcher : public BlackboardEventHandler, public BlackBoardInterfaceListener
	{
	public:
		InterfaceWatcher(BlackBoard *,
		                 const std::string &id,
		                 gologpp::shared_ptr<gologpp::ExogAction>,
		                 ExogManager &exog_mgr);
		virtual ~InterfaceWatcher() override;

		virtual void bb_interface_data_changed(Interface *) throw() override;

	private:
		Interface *iface_;
	};

	//////////////////////////////////////////////////////////////////
	class PatternObserver : public BlackboardEventHandler, public BlackBoardInterfaceObserver
	{
	public:
		PatternObserver(BlackBoard *,
		                const std::string &pattern,
		                gologpp::shared_ptr<gologpp::ExogAction>,
		                ExogManager &exog_mgr);
		virtual ~PatternObserver() override;

		virtual void bb_interface_created(const char *type, const char *id) throw() override;

	private:
		std::string pattern_;
		std::mutex  handler_mutex_;
	};

	//////////////////////////////////////////////////////////////////
	std::unordered_map<std::string, gologpp::shared_ptr<gologpp::ExogAction>> mapped_exogs_;
	std::vector<std::unique_ptr<InterfaceWatcher>>                            watchers_;
	std::vector<std::unique_ptr<PatternObserver>>                             observers_;
	GologppThread *                                                           exec_thread_;
	Configuration *                                                           config_;
	BlackBoard *                                                              blackboard_;
	Logger *                                                                  logger_;

	static const std::unordered_map<interface_fieldtype_t, std::string> iface_type_to_golog_type_;
};

} // namespace gpp
} // namespace fawkes

#endif
