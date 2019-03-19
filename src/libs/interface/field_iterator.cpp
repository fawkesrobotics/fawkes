
/***************************************************************************
 *  field_iterator.cpp - Iterate over field of an interface or a message
 *
 *  Created: Fri Jul 17 21:28:58 2009
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *             2009  Daniel Beck
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

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <interface/field_iterator.h>
#include <interface/interface.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace fawkes {

/** @class InterfaceFieldIterator <interface/interface.h>
 * Interface field iterator.
 * This iterator is part of the BlackBoard introspection API. It can be used to
 * iterate over all available fields and values of an interface without actually
 * knowing the specific type of the interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * Creates an invalid iterator.
 */
InterfaceFieldIterator::InterfaceFieldIterator()
{
	interface_    = NULL;
	infol_        = NULL;
	value_string_ = NULL;
}

/** Constructor.
 * This creates an iterator pointing to the given entry of the info list.
 * @param interface interface this field iterator is assigned to
 * @param info_list pointer to info list entry to start from
 */
InterfaceFieldIterator::InterfaceFieldIterator(Interface *                  interface,
                                               const interface_fieldinfo_t *info_list)
{
	interface_    = interface;
	infol_        = info_list;
	value_string_ = NULL;
}

/** Copy constructor.
 * @param fit iterator to copy
 */
InterfaceFieldIterator::InterfaceFieldIterator(const InterfaceFieldIterator &fit)
{
	interface_ = fit.interface_;
	infol_     = fit.infol_;
	if (fit.value_string_) {
		value_string_ = strdup(fit.value_string_);
	} else {
		value_string_ = NULL;
	}
}

/** Destructor. */
InterfaceFieldIterator::~InterfaceFieldIterator()
{
	if (value_string_)
		free(value_string_);
}

/** Prefix increment.
 * @return reference to this instance
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator++()
{
	if (infol_ != NULL) {
		infol_ = infol_->next;
		if (value_string_)
			free(value_string_);
		value_string_ = NULL;
	}

	return *this;
}

/** Postfix increment operator.
 * @param inc ignored
 * @return instance before advancing to the next shared memory segment
 */
InterfaceFieldIterator
InterfaceFieldIterator::operator++(int inc)
{
	InterfaceFieldIterator rv(*this);
	++(*this);
	return rv;
}

/** Advance by i steps.
 * @param i number of (matching) segments to advance.
 * @return reference to this after advancing
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator+(unsigned int i)
{
	for (unsigned int j = 0; j < i; ++j) {
		++(*this);
	}
	return *this;
}

/** Advance by i steps.
 * @param i number of (matching) segments to advance.
 * @return reference to this after advancing
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator+=(unsigned int i)
{
	for (unsigned int j = 0; j < i; ++j) {
		++(*this);
	}
	return *this;
}

/** Check iterators for equality.
 * @param fi iterator to compare to
 * @return true if iterators point to the the same field, false otherwise
 */
bool
InterfaceFieldIterator::operator==(const InterfaceFieldIterator &fi) const
{
	return (infol_ == fi.infol_);
}

/** Check iterators for inequality.
 * @param fi iterator to compare to
 * @return true if iteraters point to the different fields, false otherwise
 */
bool
InterfaceFieldIterator::operator!=(const InterfaceFieldIterator &fi) const
{
	return !(*this == fi);
}

/** Get FieldHeader.
 * @return shared memory header
 */
const void *InterfaceFieldIterator::operator*() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else {
		return infol_->value;
	}
}

/** Make this instance point to the same segment as fi.
 * @param fi field iterator to compare
 * @return reference to this instance
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator=(const InterfaceFieldIterator &fi)
{
	interface_ = fi.interface_;
	infol_     = fi.infol_;

	return *this;
}

/** Get type of current field.
 * @return field type
 */
interface_fieldtype_t
InterfaceFieldIterator::get_type() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get type of end element");
	} else {
		return infol_->type;
	}
}

/** Get type of current field as string.
 * @return field type as string
 */
const char *
InterfaceFieldIterator::get_typename() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get type of end element");
	} else {
		switch (infol_->type) {
		case IFT_BOOL: return "bool";
		case IFT_INT8: return "int8";
		case IFT_UINT8: return "uint8";
		case IFT_INT16: return "int16";
		case IFT_UINT16: return "uint16";
		case IFT_INT32: return "int32";
		case IFT_UINT32: return "uint32";
		case IFT_INT64: return "int64";
		case IFT_UINT64: return "uint64";
		case IFT_FLOAT: return "float";
		case IFT_DOUBLE: return "double";
		case IFT_BYTE: return "byte";
		case IFT_STRING: return "string";
		case IFT_ENUM: return infol_->enumtype;
		default: return "unknown";
		}
	}
}

/** Check if field is an enum.
 * @return true if the value is an enum, false otherwise
 */
bool
InterfaceFieldIterator::is_enum() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get type of end element");
	} else {
		return infol_->type == IFT_ENUM;
	}
}

/** Return the list of possible enum value names.
 * @return a list of the possible enum values.
 */
std::list<const char *>
InterfaceFieldIterator::get_enum_valuenames() const
{
	std::list<const char *>              enums;
	interface_enum_map_t::const_iterator enum_it;
	for (enum_it = infol_->enum_map->begin(); enum_it != infol_->enum_map->end(); ++enum_it) {
		enums.push_back(enum_it->second.c_str());
	}
	return enums;
}

/** Get name of current field.
 * @return field name
 */
const char *
InterfaceFieldIterator::get_name() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get name of end element");
	} else {
		return infol_->name;
	}
}

/** Get value of current field.
 * @return field value
 */
const void *
InterfaceFieldIterator::get_value() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else {
		return infol_->value;
	}
}

/** Get length of current field.
 * @return length of field
 */
size_t
InterfaceFieldIterator::get_length() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get length of end element");
	} else {
		return infol_->length;
	}
}

/** Get value of current field as string.
 * @param array_sep in the case that the field is an array the given string is
 * used to split the individual elements in the array string representation
 * @return field value as string
 */
const char *
InterfaceFieldIterator::get_value_string(const char *array_sep)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else {
		if (value_string_ == NULL) {
			if (infol_->length == 0)
				throw OutOfBoundsException("Field length out of bounds",
				                           infol_->length,
				                           1,
				                           (unsigned int)0xFFFFFFFF);

			char *tmp1 = strdup("");
			char *tmp2;

			if (infol_->type != IFT_STRING) {
				for (size_t i = 0; i < infol_->length; ++i) {
					int rv = 0;
					switch (infol_->type) {
					case IFT_BOOL:
						rv = asprintf(&tmp2, "%s%s", tmp1, (((bool *)infol_->value)[i]) ? "true" : "false");
						break;
					case IFT_INT8: rv = asprintf(&tmp2, "%s%i", tmp1, ((int8_t *)infol_->value)[i]); break;
					case IFT_INT16: rv = asprintf(&tmp2, "%s%i", tmp1, ((int16_t *)infol_->value)[i]); break;
					case IFT_INT32: rv = asprintf(&tmp2, "%s%i", tmp1, ((int32_t *)infol_->value)[i]); break;
					case IFT_INT64:
#if (defined(__WORDSIZE) && __WORDSIZE == 64) || (defined(LONG_BIT) && LONG_BIT == 64) \
  || defined(__x86_64__)
						rv = asprintf(&tmp2, "%s%li", tmp1, ((int64_t *)infol_->value)[i]);
#else
						rv = asprintf(&tmp2, "%s%lli", tmp1, ((int64_t *)infol_->value)[i]);
#endif
						break;
					case IFT_UINT8: rv = asprintf(&tmp2, "%s%u", tmp1, ((uint8_t *)infol_->value)[i]); break;
					case IFT_UINT16:
						rv = asprintf(&tmp2, "%s%u", tmp1, ((uint16_t *)infol_->value)[i]);
						break;
					case IFT_UINT32:
						rv = asprintf(&tmp2, "%s%u", tmp1, ((uint32_t *)infol_->value)[i]);
						break;
					case IFT_UINT64:
#if (defined(__WORDSIZE) && __WORDSIZE == 64) || (defined(LONG_BIT) && LONG_BIT == 64) \
  || defined(__x86_64__)
						rv = asprintf(&tmp2, "%s%lu", tmp1, ((uint64_t *)infol_->value)[i]);
#else
						rv = asprintf(&tmp2, "%s%llu", tmp1, ((uint64_t *)infol_->value)[i]);
#endif
						break;
					case IFT_FLOAT: rv = asprintf(&tmp2, "%s%f", tmp1, ((float *)infol_->value)[i]); break;
					case IFT_DOUBLE: rv = asprintf(&tmp2, "%s%f", tmp1, ((double *)infol_->value)[i]); break;
					case IFT_BYTE: rv = asprintf(&tmp2, "%s%u", tmp1, ((uint8_t *)infol_->value)[i]); break;
					case IFT_STRING:
						// cannot happen, caught with surrounding if statement

					case IFT_ENUM:
						rv = asprintf(&tmp2,
						              "%s%s",
						              tmp1,
						              interface_->enum_tostring(infol_->enumtype, ((int *)infol_->value)[i]));
						break;
					}

					if (rv == -1) {
						throw OutOfMemoryException(
						  "InterfaceFieldIterator::get_value_string(): asprintf() failed (1)");
					}

					free(tmp1);
					tmp1 = tmp2;
					if ((infol_->length > 1) && (i < infol_->length - 1)) {
						if (asprintf(&tmp2, "%s%s", tmp1, array_sep) == -1) {
							throw OutOfMemoryException(
							  "InterfaceFieldIterator::get_value_string(): asprintf() failed (2)");
						}
						free(tmp1);
						tmp1 = tmp2;
					}
				}

				value_string_ = tmp1;
			} else {
				// it's a string, or a small number
				if (infol_->length > 1) {
					if (asprintf(&value_string_, "%s", (const char *)infol_->value) == -1) {
						throw OutOfMemoryException(
						  "InterfaceFieldIterator::get_value_string(): asprintf() failed (3)");
					}
				} else {
					if (asprintf(&value_string_, "%c", *((const char *)infol_->value)) == -1) {
						throw OutOfMemoryException(
						  "InterfaceFieldIterator::get_value_string(): asprintf() failed (4)");
					}
				}
			}
		}
		return value_string_;
	}
}

/** Get value of current field as bool.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type bool
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
bool
InterfaceFieldIterator::get_bool(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_BOOL) {
		throw TypeMismatchException("Requested value is not of type bool");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((bool *)infol_->value)[index];
	}
}

/** Get value of current field as integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
int8_t
InterfaceFieldIterator::get_int8(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT8) {
		throw TypeMismatchException("Requested value is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((int8_t *)infol_->value)[index];
	}
}

/** Get value of current field as unsigned integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
uint8_t
InterfaceFieldIterator::get_uint8(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT8) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((uint8_t *)infol_->value)[index];
	}
}

/** Get value of current field as integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
int16_t
InterfaceFieldIterator::get_int16(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT16) {
		throw TypeMismatchException("Requested value is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((int16_t *)infol_->value)[index];
	}
}

/** Get value of current field as unsigned integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
uint16_t
InterfaceFieldIterator::get_uint16(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT16) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((uint16_t *)infol_->value)[index];
	}
}

/** Get value of current field as integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
int32_t
InterfaceFieldIterator::get_int32(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT32) {
		throw TypeMismatchException("Requested value is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((int32_t *)infol_->value)[index];
	}
}

/** Get value of current field as unsigned integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
uint32_t
InterfaceFieldIterator::get_uint32(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT32) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((uint32_t *)infol_->value)[index];
	}
}

/** Get value of current field as integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
int64_t
InterfaceFieldIterator::get_int64(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT64) {
		throw TypeMismatchException("Requested value is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((int64_t *)infol_->value)[index];
	}
}

/** Get value of current field as unsigned integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
uint64_t
InterfaceFieldIterator::get_uint64(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT64) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((uint64_t *)infol_->value)[index];
	}
}

/** Get value of current field as float.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type float
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
float
InterfaceFieldIterator::get_float(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_FLOAT) {
		throw TypeMismatchException("Requested value is not of type float");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((float *)infol_->value)[index];
	}
}

/** Get value of current field as double.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type float
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
double
InterfaceFieldIterator::get_double(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_DOUBLE) {
		throw TypeMismatchException("Requested value is not of type double");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((double *)infol_->value)[index];
	}
}

/** Get value of current field as byte.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type byte
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
uint8_t
InterfaceFieldIterator::get_byte(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_BYTE) {
		throw TypeMismatchException("Requested value is not of type byte");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((uint8_t *)infol_->value)[index];
	}
}

/** Get value of current enum field as integer.
 * @return field value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
int32_t
InterfaceFieldIterator::get_enum(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_ENUM) {
		throw TypeMismatchException("Requested value is not of type enum");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		return ((int32_t *)infol_->value)[index];
	}
}

/** Get value of current enum field as string.
 * @return field value as string
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 * @exception IllegalArgumentException thrown if the value is set to an integer
 * which is not represented by any of the canonical enum values
 */
const char *
InterfaceFieldIterator::get_enum_string(unsigned int index) const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_ENUM) {
		throw TypeMismatchException("Requested value is not of type enum");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		int32_t                              int_val = ((int32_t *)infol_->value)[index];
		interface_enum_map_t::const_iterator ev      = infol_->enum_map->find(int_val);
		if (ev == infol_->enum_map->end()) {
			throw IllegalArgumentException("Integer value is not a canonical enum value");
		}
		return ev->second.c_str();
	}
}

/** Get value of current field as bool array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type bool or field
 * is not an array (length is 1)
 */
bool *
InterfaceFieldIterator::get_bools() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_BOOL) {
		throw TypeMismatchException("Requested value is not of type bool");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		return (bool *)infol_->value;
	}
}

/** Get value of current field as integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
int8_t *
InterfaceFieldIterator::get_int8s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT8) {
		throw TypeMismatchException("Requested value is not of type int");
	} else {
		return (int8_t *)infol_->value;
	}
}

/** Get value of current field as unsigned integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * or field is not an array (length is 1)
 */
uint8_t *
InterfaceFieldIterator::get_uint8s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT8) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else {
		return (uint8_t *)infol_->value;
	}
}

/** Get value of current field as integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
int16_t *
InterfaceFieldIterator::get_int16s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT16) {
		throw TypeMismatchException("Requested value is not of type int");
	} else {
		return (int16_t *)infol_->value;
	}
}

/** Get value of current field as unsigned integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * or field is not an array (length is 1)
 */
uint16_t *
InterfaceFieldIterator::get_uint16s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT16) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else {
		return (uint16_t *)infol_->value;
	}
}

/** Get value of current field as integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
int32_t *
InterfaceFieldIterator::get_int32s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT32) {
		throw TypeMismatchException("Requested value is not of type int");
	} else {
		return (int32_t *)infol_->value;
	}
}

/** Get value of current field as unsigned integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * or field is not an array (length is 1)
 */
uint32_t *
InterfaceFieldIterator::get_uint32s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT32) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else {
		return (uint32_t *)infol_->value;
	}
}

/** Get value of current field as integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
int64_t *
InterfaceFieldIterator::get_int64s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_INT64) {
		throw TypeMismatchException("Requested value is not of type int");
	} else {
		return (int64_t *)infol_->value;
	}
}

/** Get value of current field as unsigned integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * or field is not an array (length is 1)
 */
uint64_t *
InterfaceFieldIterator::get_uint64s() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_UINT64) {
		throw TypeMismatchException("Requested value is not of type unsigned int");
	} else {
		return (uint64_t *)infol_->value;
	}
}

/** Get value of current field as float array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type float or field
 * is not an array (length is 1)
 */
float *
InterfaceFieldIterator::get_floats() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_FLOAT) {
		throw TypeMismatchException("Requested value is not of type float");
	} else {
		return (float *)infol_->value;
	}
}

/** Get value of current field as double array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type double or field
 * is not an array (length is 1)
 */
double *
InterfaceFieldIterator::get_doubles() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_DOUBLE) {
		throw TypeMismatchException("Requested value is not of type double");
	} else {
		return (double *)infol_->value;
	}
}

/** Get value of current field as byte array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type byte or field
 * is not an array (length is 1)
 */
uint8_t *
InterfaceFieldIterator::get_bytes() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_BYTE) {
		throw TypeMismatchException("Requested value is not of type byte");
	} else {
		return (uint8_t *)infol_->value;
	}
}

/** Get value of current enum field as integer array.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
int32_t *
InterfaceFieldIterator::get_enums() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_ENUM) {
		throw TypeMismatchException("Requested value is not of type enum");
	} else {
		return (int32_t *)infol_->value;
	}
}

/** Get value of current field as string.
 * @return field value
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type string
 */
const char *
InterfaceFieldIterator::get_string() const
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot get value of end element");
	} else if (infol_->type != IFT_STRING) {
		throw TypeMismatchException("Requested value is not of type string");
	} else {
		return (const char *)infol_->value;
	}
}

/** Set value of current field as bool.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type bool
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_bool(bool v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_BOOL) {
		throw TypeMismatchException("Field to be written is not of type bool");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(bool);
		memcpy((void *)dst, &v, sizeof(bool));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_int8(int8_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT8) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(int8_t);
		memcpy((void *)dst, &v, sizeof(int8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_uint8(uint8_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT8) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(uint8_t);
		memcpy((void *)dst, &v, sizeof(uint8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_int16(int16_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT16) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(int16_t);
		memcpy((void *)dst, &v, sizeof(int16_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_uint16(uint16_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT16) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(uint16_t);
		memcpy((void *)dst, &v, sizeof(uint16_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_int32(int32_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT32) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(int32_t);
		memcpy((void *)dst, &v, sizeof(int32_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_uint32(uint32_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT32) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(uint32_t);
		memcpy((void *)dst, &v, sizeof(uint32_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_int64(int64_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT64) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(int64_t);
		memcpy((void *)dst, &v, sizeof(int64_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_uint64(uint64_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT64) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(uint64_t);
		memcpy((void *)dst, &v, sizeof(uint64_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as float.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type float
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_float(float v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_FLOAT) {
		throw TypeMismatchException("Field to be written is not of type float");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(float);
		memcpy((void *)dst, &v, sizeof(float));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as double.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type double
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_double(double v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_DOUBLE) {
		throw TypeMismatchException("Field to be written is not of type double");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(double);
		memcpy((void *)dst, &v, sizeof(double));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as byte.
 * @param v the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type byte
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_byte(uint8_t v, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_BYTE) {
		throw TypeMismatchException("Field to be written is not of type byte");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		char *dst = (char *)infol_->value + index * sizeof(uint8_t);
		memcpy((void *)dst, &v, sizeof(uint8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as enum (from an integer).
 * @param e the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_enum(int32_t e, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_ENUM) {
		throw TypeMismatchException("Field to be written is not of type enum");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		interface_enum_map_t::const_iterator ev = infol_->enum_map->find(e);
		if (ev == infol_->enum_map->end()) {
			throw IllegalArgumentException("Integer value is not a canonical enum value");
		}
		char *dst = (char *)infol_->value + index * sizeof(int32_t);
		memcpy((void *)dst, &e, sizeof(int32_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as enum (from an integer).
 * @param e the new value
 * @param index array index (only use if field is an array)
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int
 * @exception OutOfBoundsException thrown if index is out of bounds
 */
void
InterfaceFieldIterator::set_enum_string(const char *e, unsigned int index)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_ENUM) {
		throw TypeMismatchException("Field to be written is not of type enum");
	} else if (index >= infol_->length) {
		throw OutOfBoundsException("Field index out of bounds", index, 0, infol_->length);
	} else {
		interface_enum_map_t::const_iterator ev;
		for (ev = infol_->enum_map->begin(); ev != infol_->enum_map->end(); ++ev) {
			if (ev->second == e) {
				char *dst = (char *)infol_->value + index * sizeof(int32_t);
				memcpy((void *)dst, &ev->first, sizeof(int32_t));
				if (interface_)
					interface_->mark_data_changed();
				return;
			}
		}
		// else value was not found
		throw IllegalArgumentException("Integer value is not a canonical enum value");
	}
}

/** Set value of current field as bool array.
 * @param v an array of bools
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type bool or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_bools(bool *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_BOOL) {
		throw TypeMismatchException("Field to be written is not of type bool");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(bool));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer array.
 * @param v an array of ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_int8s(int8_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT8) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(int8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer array.
 * @param v an array of unsigned ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_uint8s(uint8_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT8) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(uint8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer array.
 * @param v an array of ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_int16s(int16_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT16) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(int16_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer array.
 * @param v an array of unsigned ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_uint16s(uint16_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT16) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(uint16_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer array.
 * @param v an array of ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_int32s(int32_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT32) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(int32_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer array.
 * @param v an array of unsigned ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_uint32s(uint32_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT32) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(uint32_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as integer array.
 * @param v an array of ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_int64s(int64_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_INT64) {
		throw TypeMismatchException("Field to be written is not of type int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(int64_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as unsigned integer array.
 * @param v an array of unsigned ints
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type unsigned int or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_uint64s(uint64_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_UINT64) {
		throw TypeMismatchException("Field to be written is not of type unsigned int");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(uint64_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as float array.
 * @param v an array of floats
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type float or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_floats(float *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_FLOAT) {
		throw TypeMismatchException("Field to be written is not of type float");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(float));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as double array.
 * @param v an array of doubles
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type double or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_doubles(double *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_DOUBLE) {
		throw TypeMismatchException("Field to be written is not of type double");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(double));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as byte array.
 * @param v an array of bytes
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type byte or field
 * is not an array (length is 1)
 */
void
InterfaceFieldIterator::set_bytes(uint8_t *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_BYTE) {
		throw TypeMismatchException("Field to be written is not of type byte");
	} else if (infol_->length == 1) {
		throw TypeMismatchException("Field %s is not an array", infol_->name);
	} else {
		memcpy(infol_->value, v, infol_->length * sizeof(uint8_t));
		if (interface_)
			interface_->mark_data_changed();
	}
}

/** Set value of current field as string.
 * @param v a string
 * @exception NullPointerException invalid iterator, possibly end iterator
 * @exception TypeMismatchException thrown if field is not of type string
 */
void
InterfaceFieldIterator::set_string(const char *v)
{
	if (infol_ == NULL) {
		throw NullPointerException("Cannot set value of end element");
	} else if (infol_->type != IFT_STRING) {
		throw TypeMismatchException("Field to be written is not of type string");
	} else {
		strncpy((char *)infol_->value, v, infol_->length);
		if (interface_)
			interface_->mark_data_changed();
	}
}

} // end namespace fawkes
