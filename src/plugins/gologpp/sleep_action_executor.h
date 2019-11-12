/***************************************************************************
 *  sleep_action_executor.h - A simple sleep action
 *
 *  Created: Tue 12 Nov 2019 14:09:55 CET 14:09
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

#include <condition_variable>
#include <future>
#include <list>
#include <mutex>

namespace fawkes {

namespace gpp {

class SleepActionExecutor : public ActionExecutor
{
public:
	SleepActionExecutor(Logger *logger);
	virtual ~SleepActionExecutor();
	void start(std::shared_ptr<gologpp::Activity> activity) override;
	void stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity) override;
	bool can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const override;

private:
	std::list<std::future<void>> running_sleeps_;
	std::mutex                   cancel_mutex_;
	std::condition_variable      wait_cond_;
};

} // namespace gpp
} // namespace fawkes
