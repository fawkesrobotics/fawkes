
/***************************************************************************
 *  asp_inifin.h - Fawkes ASPAspect initializer/finalizer
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

#ifndef _PLUGINS_ASP_ASPECT_ASP_INIFIN_H_
#define _PLUGINS_ASP_ASPECT_ASP_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <core/utils/lockptr.h>

#include <plugins/asp/aspect/asp.h>

namespace fawkes {

class ClingoControlManager;
class Logger;

class ASPAspectIniFin : public AspectIniFin
{
public:
	ASPAspectIniFin(void);
	~ASPAspectIniFin(void);

	void init(Thread *thread) override;
	void finalize(Thread *thread) override;

	void set_control_manager(const LockPtr<ClingoControlManager>& ctrl_mgr);

private:
	LockPtr<ClingoControlManager> ctrl_mgr_;
};

} // end namespace fawkes

#endif
