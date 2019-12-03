/***************************************************************************
 *  remote_skiller_executor.h - Execute Golog++ actions as skills remotely
 *
 *  Created: Tue 03 Dec 2019 14:33:42 CET 14:33
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

#include "skiller_action_executor.h"

namespace fawkes {
namespace gpp {
class RemoteSkillerActionExecutor : public SkillerActionExecutor
{
public:
	RemoteSkillerActionExecutor(Logger *           logger,
	                            const std::string &agent_name_key,
	                            const std::string &agent_name_value,
	                            const std::string &hostname,
	                            unsigned short int port,
	                            Configuration *    config,
	                            const std::string &cfg_prefix);
	virtual ~RemoteSkillerActionExecutor() override;
	bool can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const override;

protected:
	const char *name() const;

private:
	const std::string agent_name_key_;
	const std::string agent_name_value_;
};
} // namespace gpp
} // namespace fawkes
