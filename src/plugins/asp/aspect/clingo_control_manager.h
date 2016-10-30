/***************************************************************************
 *  clingo_control_manager.h - Clingo control manager
 *
 *  Created: Thu Oct 27 16:23:32 2016
 *  Copyright  2016  Björn Schäpers
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

#ifndef __PLUGINS_ASP_ASPECT_CLINGO_CONTROL_MANAGER_H_
#define __PLUGINS_ASP_ASPECT_CLINGO_CONTROL_MANAGER_H_

#include <core/utils/lockptr.h>

#include <unordered_map>

namespace Clingo {
	class Control;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

class ClingoControlManager
{
	private:
	Logger *Log;
	std::unordered_map<std::string, LockPtr<Clingo::Control>> Controls;

	public:
	ClingoControlManager(void);
	virtual ~ClingoControlManager(void);

	void setLogger(Logger *logger);

	LockPtr<Clingo::Control> create_control(const std::string& ctrl_name, const std::string& log_component_name);
	void destroy_control(const std::string& ctrl_name);

	const std::unordered_map<std::string, LockPtr<Clingo::Control>>& controls(void) const;
};

} // end namespace fawkes

#endif
