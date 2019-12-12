/***************************************************************************
 *  execution_time_estimator.h - An execution time estimator for skills
 *
 *  Created: Thu 12 Dec 2019 15:10:06 CET 15:10
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#pragma once

#include <string>

namespace fawkes {
namespace skiller_simulator {

class ExecutionTimeEstimator
{
public:
	virtual float get_executioN_time(const std::string &skill_string) const = 0;
	virtual bool  can_execute(const std::string &skill_string) const        = 0;
};

} // namespace skiller_simulator
} // namespace fawkes
