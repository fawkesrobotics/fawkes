/***************************************************************************
 *  message_action_executor.h - Action executor for blackboard messages
 *
 *  Created: Wed 30 Oct 2019 13:05:15 CET 13:05
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "action_executor.h"

#include <map>
#include <string>

namespace fawkes {

class BlackBoard;
class Configuration;
class Interface;
class Logger;

namespace gpp {
class BBMessageActionExecutor : public ActionExecutor
{
public:
	BBMessageActionExecutor(Logger            *logger,
	                        BlackBoard        *blackboard,
	                        Configuration     *config,
	                        const std::string &cfg_prefix);
	virtual ~BBMessageActionExecutor();
	void start(std::shared_ptr<gologpp::Activity> activity) override;
	void stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity) override;
	bool can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const override;

private:
	BlackBoard                        *blackboard_;
	Configuration                     *config_;
	std::string                        cfg_prefix_;
	std::map<std::string, Interface *> open_interfaces_;
};
} // namespace gpp
} // namespace fawkes
