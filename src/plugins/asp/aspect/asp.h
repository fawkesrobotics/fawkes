
/***************************************************************************
 *  asp.h - ASP aspect for Fawkes
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

#ifndef _PLUGINS_ASP_ASPECT_ASP_H_
#define _PLUGINS_ASP_ASPECT_ASP_H_

#include <aspect/aspect.h>
#include <core/utils/lockptr.h>

#include <string>

namespace fawkes {

class ClingoAccess;

class ASPAspect : public virtual Aspect
{
public:
	ASPAspect(const std::string&& control_name, const std::string&& log_component = std::string());
	virtual ~ASPAspect(void);

private:
	const std::string control_name_;
	const std::string log_comp_;

	void init_ASPAspect(const LockPtr<ClingoAccess>& clingo);
	void finalize_ASPAspect(void);

	protected:
	LockPtr<ClingoAccess> clingo;

	/** Additional access by ASPAspectIniFin
	 * @relates ASPAspectIniFin
	 */
	friend class ASPAspectIniFin;
};

} // end namespace fawkes

#endif
