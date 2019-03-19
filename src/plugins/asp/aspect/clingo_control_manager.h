/***************************************************************************
 *  clingo_control_manager.h - Clingo control manager
 *
 *  Created: Thu Oct 27 16:23:32 2016
 *  Copyright  2016  Björn Schäpers
 *             2018  Tim Niemueller [www.niemueller.org]
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

#ifndef _PLUGINS_ASP_ASPECT_CLINGO_CONTROL_MANAGER_H_
#define _PLUGINS_ASP_ASPECT_CLINGO_CONTROL_MANAGER_H_

#include <core/utils/lockptr.h>

#include <unordered_map>

namespace fawkes {

class ClingoAccess;
class Logger;

class ClingoControlManager
{
public:
	ClingoControlManager(void);
	virtual ~ClingoControlManager(void);

	void set_logger(Logger *logger);

	LockPtr<ClingoAccess> create_control(const std::string &ctrl_name,
	                                     const std::string &log_component_name);
	void                  destroy_control(const std::string &ctrl_name);

	const std::unordered_map<std::string, LockPtr<ClingoAccess>> &controls(void) const;

private:
	Logger *                                               logger_;
	std::unordered_map<std::string, LockPtr<ClingoAccess>> controls_;
};

} // end namespace fawkes

#endif
