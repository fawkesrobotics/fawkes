
/***************************************************************************
 *  change_field.h - Detect whether an update actually changes a field
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

#ifndef _INTERFACE_CHANGE_FIELD_H_
#define _INTERFACE_CHANGE_FIELD_H_

#include <core/exceptions/software.h>

#include <cstring>
#include <type_traits>

namespace fawkes {

/** Set a field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, class DataT>
bool
change_field(FieldT &field, const DataT &value)
{
	bool rv = field != value;
	field   = value;
	return rv;
}

/** Set a string field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size>
bool
change_field(FieldT (&field)[Size], const char *value)
{
	bool change = ::strncmp(field, value, Size);
	::strncpy(field, value, Size - 1);
	field[Size - 1] = 0;
	return change;
}

/** Set an array field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size, class DataT>
typename std::enable_if<!std::is_same<FieldT, char>::value, bool>::type
change_field(FieldT (&field)[Size], const DataT *value)
{
	bool change = ::memcmp(field, value, Size);
	::memcpy(field, value, sizeof(FieldT) * Size);
	return change;
}

/** Set an array field value at a certain index and return whether it changed
 * @param field The interface field to change
 * @param index Index into the array field
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size, class DataT>
bool
change_field(FieldT (&field)[Size], unsigned int index, const DataT &value)
{
	if (index >= Size)
		throw Exception("Index value %u is out of bounds (0..%u)", index, Size - 1);
	bool change  = field[index] != value;
	field[index] = value;
	return change;
}

} // namespace fawkes

#endif // _INTERFACE_CHANGE_FIELD_H_
