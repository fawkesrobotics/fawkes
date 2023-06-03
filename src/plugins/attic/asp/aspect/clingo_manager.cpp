/***************************************************************************
 *  clingo_manager.cpp - Clingo manager aspect for Fawkes
 *
 *  Created: Sat Oct 29 11:30:07 2016
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

#include <plugins/asp/aspect/clingo_manager.h>

namespace fawkes {

/** @class ClingoManagerAspect <plugins/asp/aspect/clingo_manager.h>
 * Thread aspect to access the Clingo Control manager.

 * Give this aspect to your thread if you want to access the Clingo control manager.
 * Use this with extreme care and only if you know what you are doing.
 * If you want to create a Clingo control to work with use the ASPAspect.
 *
 * @ingroup Aspects
 * @author Björn Schäpers
 *
 * @property ClingoManagerAspect::clingo_ctrl_mgr
 * The Clingo control manager.
 */

/** Constructor. */
ClingoManagerAspect::ClingoManagerAspect(void)
{
	add_aspect("ClingoManagerAspect");
}

/** Virtual empty destructor. */
ClingoManagerAspect::~ClingoManagerAspect(void)
{
}

/** Init ClingoManagerAspect.
 * This sets the Clingo Control Manager.
 * @param[in] clingo_ctrl_mgr The Clingo Control Manager
 */
void
ClingoManagerAspect::init_ClingoManagerAspect(const LockPtr<ClingoControlManager> &clingo_ctrl_mgr)
{
	this->clingo_ctrl_mgr = clingo_ctrl_mgr;
}

/** Finalize ASP aspect.
 * This clears the Clingo Control.
 */
void
ClingoManagerAspect::finalize_ClingoManagerAspect(void)
{
	clingo_ctrl_mgr.clear();
}

} // end namespace fawkes
