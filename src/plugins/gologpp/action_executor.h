/***************************************************************************
 *  action_executor.h - An abstract action executor for Golog++
 *
 *  Created: Thu 03 Oct 2019 08:41:55 CEST 08:41
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

#ifndef FAWKES_GOLOGPP_ACTION_EXECUTOR_H
#define FAWKES_GOLOGPP_ACTION_EXECUTOR_H

#include <golog++/model/gologpp.h>

#include <string>

namespace fawkes {
class Logger;
}

namespace fawkes_gpp {
class ActionExecutor
{
public:
	ActionExecutor(fawkes::Logger *logger);
	virtual void start(std::shared_ptr<gologpp::Activity> activity)                      = 0;
	virtual void stop(std::shared_ptr<gologpp::Activity> activity)                       = 0;
	virtual bool can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const = 0;

protected:
	std::shared_ptr<gologpp::Activity> running_activity_;
	fawkes::Logger *                   logger_;
};

} // namespace fawkes_gpp

#endif /* !FAWKES_GOLOGPP_ACTION_EXECUTOR_H */
