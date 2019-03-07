
/***************************************************************************
 *  asp_inifin.cpp - Fawkes ASPAspect initializer/finalizer
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
 *             2018 Tim Niemueller [www.niemueller.org]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <core/threading/thread_finalizer.h>
#include <logging/logger.h>
#include <plugins/asp/aspect/asp.h>
#include <plugins/asp/aspect/asp_inifin.h>
#include <plugins/asp/aspect/clingo_access.h>
#include <plugins/asp/aspect/clingo_control_manager.h>

namespace fawkes {

/**
 * @class ASPAspectIniFin <plugins/asp/aspect/asp_inifin.h>
 * ASPAspect initializer/finalizer.
 * This initializer/finalizer will provide the ASP node handle to threads with the ASPAspect.
 * @author Björn Schäpers
 *
 * @property ASPAspectIniFin::ctrl_mgr_
 * @brief The control manager.
 */

/** Constructor. */
ASPAspectIniFin::ASPAspectIniFin() : AspectIniFin("ASPAspect")
{
}

/** Destructor. */
ASPAspectIniFin::~ASPAspectIniFin()
{
}

void
ASPAspectIniFin::init(Thread *thread)
{
	ASPAspect *asp_thread = dynamic_cast<ASPAspect *>(thread);
	if (asp_thread == nullptr) {
		throw CannotInitializeThreadException("Thread '%s' claims to have the ASPAspect, "
		                                      "but RTTI says it has not.",
		                                      thread->name());
	}

	asp_thread->init_ASPAspect(
	  ctrl_mgr_->create_control(asp_thread->control_name_, asp_thread->log_comp_));
}

void
ASPAspectIniFin::finalize(Thread *thread)
{
	ASPAspect *asp_thread = dynamic_cast<ASPAspect *>(thread);
	if (asp_thread == nullptr) {
		throw CannotFinalizeThreadException("Thread '%s' claims to have the ASPAspect, "
		                                    "but RTTI says it has not.",
		                                    thread->name());
	}

	asp_thread->finalize_ASPAspect();
}

/** Sets the control manager.
 * @param[in] ctrl_mgr The new control manager
 */
void
ASPAspectIniFin::set_control_manager(const LockPtr<ClingoControlManager> &ctrl_mgr)
{
	ctrl_mgr_ = ctrl_mgr;
}

} // end namespace fawkes
