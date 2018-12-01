/***************************************************************************
 *  clingo_manager.h - Clingo manager aspect for Fawkes
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

#ifndef _PLUGINS_ASP_ASPECT_CLINGO_MANAGER_H_
#define _PLUGINS_ASP_ASPECT_CLINGO_MANAGER_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

#include <plugins/asp/aspect/clingo_control_manager.h>

namespace fawkes {

class ClingoManagerAspect : public virtual Aspect
{
public:
	ClingoManagerAspect(void);
	virtual ~ClingoManagerAspect(void);

	void init_ClingoManagerAspect(const LockPtr<ClingoControlManager>& clingo_ctrl_mgr);
	void finalize_ClingoManagerAspect(void);

protected:
	LockPtr<ClingoControlManager> clingo_ctrl_mgr;
};

} // end namespace fawkes

#endif
