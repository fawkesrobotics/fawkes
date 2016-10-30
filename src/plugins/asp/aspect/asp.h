
/***************************************************************************
 *  asp.h - ASP aspect for Fawkes
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

#ifndef __PLUGINS_ASP_ASPECT_ASP_H_
#define __PLUGINS_ASP_ASPECT_ASP_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

#include <string>

namespace Clingo {
  class Control;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ASPAspect : public virtual Aspect
{
	private:
	const std::string ControlName;

	void init_ASPAspect(LockPtr<Clingo::Control> clingoControl);
	void finalize_ASPAspect();

	protected:
	LockPtr<Clingo::Control> ClingoControl;

	public:
	ASPAspect(const std::string&& controlName);
	virtual ~ASPAspect(void);

	friend class ASPAspectIniFin;
};

} // end namespace fawkes

#endif
