/***************************************************************************
 *  clingo_manager_inifin.cpp - Fawkes ClingoManagerAspect initializer/finalizer
 *
 *  Created: Sat Oct 29 11:30:07 2016
 *  Copyright  2016 Björn Schäpers
 *
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

#include "clingo_manager.h"
#include "clingo_manager_inifin.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/**
 * @class ClingoManagerAspectIniFin <plugins/asp/aspect/clingo_manager_inifin.h>
 * ClingoManagerAspect initializer/finalizer.
 * @author Björn Schäpers
 *
 * @property ClingoManagerAspectIniFin::ClingoCtrlMgr
 * @brief The Clingo control manager.
 */

/** Constructor. */
ClingoManagerAspectIniFin::ClingoManagerAspectIniFin(void) : AspectIniFin("ClingoManagerAspect")
{
	return;
}

/** Destructor. */
ClingoManagerAspectIniFin::~ClingoManagerAspectIniFin(void)
{
	return;
}

void
ClingoManagerAspectIniFin::init(Thread *thread)
{
	auto clingo_thread = dynamic_cast<ClingoManagerAspect *>(thread);
	if ( clingo_thread == nullptr )
	{
		throw CannotInitializeThreadException(
			"Thread '%s' claims to have the ClingoManagerAspect, but RTTI says it has not. ", thread->name());
	} //if ( clingo_thread == nullptr )

	clingo_thread->ClingoCtrlMgr = ClingoCtrlMgr;
	return;
}

void
ClingoManagerAspectIniFin::finalize(Thread *thread)
{
	auto clingo_thread = dynamic_cast<ClingoManagerAspect *>(thread);
	if ( clingo_thread == nullptr )
	{
		throw CannotFinalizeThreadException(
			"Thread '%s' claims to have the ClingoManagerAspect, but RTTI says it has not. ", thread->name());
	} //if ( clingo_thread == nullptr )

	clingo_thread->ClingoCtrlMgr.clear();
	return;
}

/**
 * Set Clingo control manger.
 * @param[in] clingoCtrlMgr Clingo control manager
 */
void
ClingoManagerAspectIniFin::set_manager(LockPtr<ClingoControlManager>& clingoCtrlMgr)
{
	ClingoCtrlMgr = clingoCtrlMgr;
	return;
}

} // end namespace fawkes
