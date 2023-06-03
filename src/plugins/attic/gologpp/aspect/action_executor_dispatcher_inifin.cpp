/***************************************************************************
 *  action_executor_dispatcher_inifin.cpp - Inifin for the Golog++ Executor
 *
 *  Created: Sat 12 Oct 2019 12:14:27 CEST 12:14
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

#include "action_executor_dispatcher_inifin.h"

namespace fawkes {

/** @class GologppDispatcherAspectIniFin
 * The initializer/finalizer for the GologppDispatcherAspect.
 * This AspectIniFin is responsible for creating the dispatcher,
 * which can then be used by all executors.
 */

/** Constructor. */
GologppDispatcherAspectIniFin::GologppDispatcherAspectIniFin()
: AspectIniFin("GologppDispatcherAspect")
{
	dispatcher_ = new gpp::ActionExecutorDispatcher();
}

/** Destructor*. */
GologppDispatcherAspectIniFin::~GologppDispatcherAspectIniFin()
{
	delete dispatcher_;
}

GologppDispatcherAspect *
GologppDispatcherAspectIniFin::get_aspect(Thread *thread) const
{
	GologppDispatcherAspect *exec_thread = dynamic_cast<GologppDispatcherAspect *>(thread);
	if (!exec_thread) {
		throw CannotInitializeThreadException(
		  "Thread '%s' claims to have the GologppDispatcherAspect, but RRTI says it has not.",
		  thread->name());
	}
	return exec_thread;
}

/** Initialize the thread with the aspect.
 * This initializes the given thread with the dispatcher.
 * @param thread The thread to initialize.
 */
void
GologppDispatcherAspectIniFin::init(Thread *thread)
{
	auto exec_thread = get_aspect(thread);
	exec_thread->init_GologppDispatcherAspect(dispatcher_);
}

/** Finalize the aspect of the given thread.
 * @param thread The thread to finalize.
 */
void
GologppDispatcherAspectIniFin::finalize(Thread *thread)
{
	auto exec_thread = get_aspect(thread);
	exec_thread->finalize_GologppDispatcherAspect();
}

} // namespace fawkes
