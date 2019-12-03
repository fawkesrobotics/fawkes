/***************************************************************************
 *  action_executor_dispatcher_inifin.h - Inifin for the Golog++ Executor
 *
 *  Created: Sat 12 Oct 2019 12:06:31 CEST 12:06
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

#ifndef FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_INIFIN_H
#define FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_INIFIN_H

#include "action_executor_dispatcher.h"

#include <aspect/inifins/inifin.h>

namespace fawkes {

class GologppDispatcherAspectIniFin : public virtual AspectIniFin
{
public:
	GologppDispatcherAspectIniFin();
	virtual ~GologppDispatcherAspectIniFin();
	virtual void init(Thread *thread);
	virtual void finalize(Thread *thread);

private:
	GologppDispatcherAspect *      get_aspect(Thread *thread) const;
	gpp::ActionExecutorDispatcher *dispatcher_;
};

} // namespace fawkes

#endif /* !FAWKES_GOLOGPP_ASPECT_ACTION_EXECUTOR_DISPATCHER_INIFIN_H */
