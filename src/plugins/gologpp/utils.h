/***************************************************************************
 *  utils.h - Common utility functions used with Golog++
 *
 *  Created: Wed 30 Oct 2019 14:36:46 CET 14:36
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

#include <golog++/model/value.h>

#include <boost/variant/variant.hpp>

namespace gologpp {
class Value;
}

namespace fawkes {
class InterfaceFieldIterator;

namespace gpp {

class ValueToFieldVisitor : public boost::static_visitor<>
{
public:
	ValueToFieldVisitor(InterfaceFieldIterator *field);
	void operator()(unsigned int v);
	void operator()(int v);
	void operator()(unsigned long v);
	void operator()(long v);
	void operator()(double v);
	void operator()(std::string v);
	void operator()(bool v);
	void operator()(gologpp::CompoundType::Representation v);
	void operator()(gologpp::ListType::Representation v);
	void operator()(void *v);

private:
	InterfaceFieldIterator *field;
};

void value_to_field(const gologpp::Value &value, InterfaceFieldIterator *field);

} // namespace gpp
} // namespace fawkes
