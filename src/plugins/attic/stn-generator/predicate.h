
/***************************************************************************
 *  predicate.h - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#ifndef PLUGINS_PREDICATE_H_
#define PLUGINS_PREDICATE_H_

#include <iostream>
#include <string>
#include <vector>

namespace fawkes {
namespace stn {

class Predicate
{
public:
	Predicate(const std::string &name, bool condition, const std::vector<std::string> &attrs);
	virtual ~Predicate(){};

	friend std::ostream &operator<<(std::ostream &, const Predicate &);
	bool                 operator==(const Predicate &rhs);

	std::string                     name() const;
	bool                            condition() const;
	const std::vector<std::string> &attrs() const;

private:
	std::string              name_;
	bool                     condition_;
	std::vector<std::string> attrs_;
};
} // namespace stn
} // namespace fawkes
#endif
