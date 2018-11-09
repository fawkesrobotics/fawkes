/***************************************************************************
 *  clingo_manager_inifin.h - Fawkes ClingoManagerAspect initializer/finalizer
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

#ifndef __PLUGINS_ASP_ASPECT_CLINGO_MANAGER_INIFIN_H_
#define __PLUGINS_ASP_ASPECT_CLINGO_MANAGER_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <core/utils/lockptr.h>

namespace fawkes {

class ClingoControlManager;

class ClingoManagerAspectIniFin : public AspectIniFin
{
	public:
	ClingoManagerAspectIniFin(void);
	~ClingoManagerAspectIniFin(void);

	void init(Thread *thread) override;
	void finalize(Thread *thread) override;

	void set_control_manager(LockPtr<ClingoControlManager>& clingo_ctrl_mgr);

private:
	LockPtr<ClingoControlManager> clingo_ctrl_mgr_;
};

} // end namespace fawkes

#endif
