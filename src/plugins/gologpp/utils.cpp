/***************************************************************************
 *  utils.cpp - Common utility functions used with Golog++
 *
 *  Created: Wed 30 Oct 2019 14:31:18 CET 14:31
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

#include "utils.h"

#include <blackboard/blackboard.h>
#include <golog++/model/value.h>

namespace fawkes {
namespace gpp {

ValueToFieldVisitor::ValueToFieldVisitor(InterfaceFieldIterator *field) : field(field)
{
}

void
ValueToFieldVisitor::operator()(unsigned int v)
{
	switch (field->get_type()) {
	case IFT_INT32: field->set_int32(v); break;
	case IFT_INT64: field->set_int64(v); break;
	case IFT_UINT16: field->set_uint16(v); break;
	case IFT_UINT32: field->set_uint32(v); break;
	case IFT_UINT64: field->set_uint64(v); break;
	default: throw Exception("Invalid cast from unsigned int to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(int v)
{
	switch (field->get_type()) {
	case IFT_INT32: field->set_int32(v); break;
	case IFT_INT64: field->set_int64(v); break;
	default: throw Exception("Invalid cast from int to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(unsigned long v)
{
	switch (field->get_type()) {
	case IFT_INT64: field->set_int64(v); break;
	case IFT_UINT32: field->set_uint32(v); break;
	case IFT_UINT64: field->set_uint64(v); break;
	default: throw Exception("Invalid cast from unsigned long to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(long v)
{
	switch (field->get_type()) {
	case IFT_UINT32: field->set_uint32(v); break;
	case IFT_INT64: field->set_int64(v); break;
	default: throw Exception("Invalid cast from long to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(double v)
{
	switch (field->get_type()) {
	case IFT_DOUBLE: field->set_double(v); break;
	default: throw Exception("Invalid cast from double to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(std::string v)
{
	switch (field->get_type()) {
	case IFT_STRING: field->set_string(v.c_str());
	default: throw Exception("Invalid cast from string to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(bool v)
{
	switch (field->get_type()) {
	case IFT_BOOL: field->set_bool(v);
	default: throw Exception("Invalid cast from bool to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(void *v)
{
	switch (field->get_type()) {
	default: throw Exception("Invalid cast from void* to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(gologpp::CompoundType::Representation v)
{
	switch (field->get_type()) {
	default: throw Exception("Invalid cast from compound to %s", field->get_typename());
	}
}

void
ValueToFieldVisitor::operator()(gologpp::ListType::Representation v)
{
	switch (field->get_type()) {
	default: throw Exception("Invalid cast from list to %s", field->get_typename());
	}
}

void
value_to_field(const gologpp::Value &value, InterfaceFieldIterator *field)
{
	ValueToFieldVisitor visitor(field);
	boost::apply_visitor(visitor, value.representation());
}

} // namespace gpp
} // namespace fawkes
