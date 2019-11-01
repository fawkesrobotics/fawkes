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

void
value_to_field(const gologpp::Value &value, InterfaceFieldIterator *field)
{
	switch (field->get_type()) {
	case IFT_BOOL: field->set_bool(value); break;
	case IFT_INT8: field->set_int8((int)value); break;
	case IFT_UINT8: field->set_uint8((uint)value); break;
	case IFT_INT16: field->set_int16((int)value); break;
	case IFT_UINT16: field->set_uint16((uint)value); break;
	case IFT_INT32: field->set_int32((int)value); break;
	case IFT_UINT32: field->set_uint32((uint)value); break;
	case IFT_INT64: field->set_int64((long)value); break;
	case IFT_UINT64: field->set_uint64((unsigned long)value); break;
	case IFT_FLOAT: field->set_float((double)value); break;
	case IFT_DOUBLE: field->set_float((double)value); break;
	case IFT_STRING: field->set_string(static_cast<std::string>(value).c_str()); break;
	case IFT_ENUM: field->set_enum(value); break;
	case IFT_BYTE: throw NotImplementedException("Cannot convert Golog++ value into byte"); break;
	}
}

} // namespace gpp
} // namespace fawkes
