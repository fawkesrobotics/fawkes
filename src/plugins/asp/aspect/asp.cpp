
/***************************************************************************
 *  asp.cpp - ASP aspect for Fawkes
 *
 *  Created: Thu Oct 20 15:49:31 2016
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

#include "asp.h"
#include "clingo_access.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ASPAspect <plugins/asp/aspect/asp.h>
 * Thread aspect to get access to an ASP solver.
 * Give this aspect to your thread to get a Clingo Control for exclusive usage.
 *
 * @ingroup Aspects
 * @author Björn Schäpers
 *
 * @property ASPAspect::ControlName
 * @brief The name for the control in the manager.
 *
 * @property ASPAspect::LogComponent
 * @brief The component for the logger.
 *
 * @property ASPAspect::ClingoAcc
 * @brief Clingo Control for exclusive usage.
 */


/**
 * Constructor.
 * @param[in] controlName The desired control name.
 * @param[in] logComponent The component for the logger.
 */
ASPAspect::ASPAspect(const std::string&& controlName, const std::string&& logComponent) :
	ControlName(std::move(controlName)), LogComponent(std::move(logComponent))
{
	add_aspect("ASPAspect");
	return;
}

/** Virtual empty destructor. */
ASPAspect::~ASPAspect(void)
{
	return;
}


/** Init ASP aspect.
 * This sets the Clingo Control.
 * It is guaranteed that this is called for an ASP Thread before start is called (when running regularly inside Fawkes).
 * @param[in] clingo The Clingo Control.
 */
void
ASPAspect::init_ASPAspect(const LockPtr<ClingoAccess>& clingo)
{
	ClingoAcc = clingo;
	return;
}

/** Finalize ASP aspect.
 * This clears the Clingo Control.
 */
void
ASPAspect::finalize_ASPAspect(void)
{
	ClingoAcc.clear();
	return;
}

} // end namespace fawkes
