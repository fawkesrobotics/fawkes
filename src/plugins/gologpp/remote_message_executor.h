/***************************************************************************
 *  remote_message_executor.h - Send messages to a remote blackboard
 *
 *  Created: Fri 14 Feb 2020 13:36:03 CET 13:36
 *  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#pragma once

#include "message_action_executor.h"
namespace fawkes {
namespace gpp {
class RemoteBBMessageActionExecutor : public BBMessageActionExecutor
{
public:
	RemoteBBMessageActionExecutor(Logger *           logger,
	                              const std::string &agent_param_name,
	                              const std::string &agent_param_value,
	                              const std::string &hostname,
	                              unsigned short int port,
	                              Configuration *    config,
	                              const std::string &cfg_prefix);
	bool can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const override;

protected:
	const char *name() const;

private:
	const std::string agent_param_name_;
	const std::string agent_param_value_;
};
} // namespace gpp
} // namespace fawkes
