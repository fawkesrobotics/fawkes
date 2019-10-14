/***************************************************************************
 *  action_executor_dispatcher.h - Dispatch a Golog++ activity to executors
 *
 *  Created: Thu 03 Oct 2019 10:43:28 CEST 10:43
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

#ifndef FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_H
#define FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_H

#include "../action_executor.h"

#include <aspect/aspect.h>

namespace fawkes_gpp {

class ActionExecutorDispatcher
{
public:
	std::shared_ptr<ActionExecutor> get_executor(std::shared_ptr<gologpp::Activity>);
	void                            register_executor(std::shared_ptr<ActionExecutor> executor);
	std::vector<std::shared_ptr<ActionExecutor>> get_executors() const;

private:
	std::vector<std::shared_ptr<ActionExecutor>> action_executors_;
};

} // namespace fawkes_gpp

namespace fawkes {

class GologppDispatcherAspect : public virtual Aspect
{
public:
	GologppDispatcherAspect();
	void init_GologppDispatcherAspect(fawkes_gpp::ActionExecutorDispatcher *dispatcher);
	void finalize_GologppDispatcherAspect();

protected:
	fawkes_gpp::ActionExecutorDispatcher *gologpp_dispatcher;
};

} // namespace fawkes

#endif /* !FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_H */
