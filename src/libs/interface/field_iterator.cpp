
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

#include <interface/field_iterator.h>
#include <interface/interface.h>

#include <core/exceptions/software.h>
#include <core/exceptions/system.h>

#include <cstdlib>
#include <cstring>
#include <cstdio>

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
  __interface = NULL;
  __infol = NULL;
  __value_string = NULL;
}


/** Constructor.
 * This creates an iterator pointing to the given entry of the info list.
 * @param interface interface this field iterator is assigned to
 * @param info_list pointer to info list entry to start from
 */
  InterfaceFieldIterator::InterfaceFieldIterator(const Interface *interface,
						 const interface_fieldinfo_t *info_list)
{
  __interface = interface;
  __infol = info_list;
  __value_string = NULL;
}


/** Copy constructor.
 * @param fit iterator to copy
 */
InterfaceFieldIterator::InterfaceFieldIterator(const InterfaceFieldIterator &fit)
{
  __infol = fit.__infol;
  if ( fit.__value_string ) {
    __value_string = strdup(fit.__value_string);
  } else {
    __value_string = NULL;
  }
}


/** Destructor. */
InterfaceFieldIterator::~InterfaceFieldIterator()
{
  if ( __value_string )  free(__value_string);
}


/** Prefix increment.
 * @return reference to this instance
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator++()
{
  if ( __infol != NULL ) {
    __infol = __infol->next;
    if ( __value_string )  free(__value_string);
    __value_string = NULL;
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
InterfaceFieldIterator::operator==(const InterfaceFieldIterator & fi) const
{
  return (__infol == fi.__infol);
}


/** Check iterators for inequality.
 * @param fi iterator to compare to
 * @return true if iteraters point to the different fields, false otherwise
 */
bool
InterfaceFieldIterator::operator!=(const InterfaceFieldIterator & fi) const
{
  return ! (*this == fi);
}


/** Get FieldHeader.
 * @return shared memory header
 */
const void *
InterfaceFieldIterator::operator*() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else {
    return __infol->value;
  }
}


/** Make this instance point to the same segment as fi.
 * @param fi field iterator to compare
 * @return reference to this instance
 */
InterfaceFieldIterator &
InterfaceFieldIterator::operator=(const InterfaceFieldIterator & fi)
{
  __interface = fi.__interface;
  __infol     = fi.__infol;

  return *this;
}


/** Get type of current field.
 * @return field type
 */
interface_fieldtype_t
InterfaceFieldIterator::get_type() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get type of end element");
  } else {
    return __infol->type;
  }
}


/** Get type of current field as string.
 * @return field type as string
 */
const char *
InterfaceFieldIterator::get_typename() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get type of end element");
  } else {
    switch (__infol->type) {
    case IFT_BOOL:     return "bool";
    case IFT_INT8:     return "int8";
    case IFT_UINT8:    return "uint8";
    case IFT_INT16:    return "int16";
    case IFT_UINT16:   return "uint16";
    case IFT_INT32:    return "int32";
    case IFT_UINT32:   return "uint32";
    case IFT_INT64:    return "int64";
    case IFT_UINT64:   return "uint64";
    case IFT_FLOAT:    return "float";
    case IFT_BYTE:     return "byte";
    case IFT_STRING:   return "string";
    case IFT_ENUM:     return __infol->enumtype;
    default:           return "unknown";
    }
  }
}


/** Get name of current field.
 * @return field name
 */
const char *
InterfaceFieldIterator::get_name() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get name of end element");
  } else {
    return __infol->name;
  }
}


/** Get value of current field.
 * @return field value
 */
const void *
InterfaceFieldIterator::get_value() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else {
    return __infol->value;
  }
}


/** Get length of current field.
 * @return length of field
 */
size_t
InterfaceFieldIterator::get_length() const
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get length of end element");
  } else {
    return __infol->length;
  }
}


/** Get value of current field as string.
 * @return field value as string
 */
const char *
InterfaceFieldIterator::get_value_string()
{
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else {
    if ( __value_string == NULL ) {
      if ( __infol->length == 0 )  throw OutOfBoundsException("Field length out of bounds",
							      __infol->length, 1, (unsigned int)0xFFFFFFFF);

      char *tmp1 = strdup("");
      char *tmp2;

      if ( __infol->type != IFT_STRING ) {
	for (size_t i = 0; i < __infol->length; ++i) {
          int rv = 0;
	  switch (__infol->type) {
	  case IFT_BOOL:
	    rv = asprintf(&tmp2, "%s%s", tmp1, (((bool *)__infol->value)[i]) ? "true" : "false");
	    break;
	  case IFT_INT8:
	    rv = asprintf(&tmp2, "%s%i", tmp1, ((int8_t *)__infol->value)[i]);
	    break;
	  case IFT_INT16:
	    rv = asprintf(&tmp2, "%s%i", tmp1, ((int16_t *)__infol->value)[i]);
	    break;
	  case IFT_INT32:
	    rv = asprintf(&tmp2, "%s%i", tmp1, ((int32_t *)__infol->value)[i]);
	    break;
	  case IFT_INT64:
#if __WORDSIZE == 64
	    rv = asprintf(&tmp2, "%s%li", tmp1, ((int64_t *)__infol->value)[i]);
#else
	    rv = asprintf(&tmp2, "%s%lli", tmp1, ((int64_t *)__infol->value)[i]);
#endif
	    break;
	  case IFT_UINT8:
	    rv = asprintf(&tmp2, "%s%u", tmp1, ((uint8_t *)__infol->value)[i]);
	    break;
	  case IFT_UINT16:
	    rv = asprintf(&tmp2, "%s%u", tmp1, ((uint16_t *)__infol->value)[i]);
	    break;
	  case IFT_UINT32:
	    rv = asprintf(&tmp2, "%s%u", tmp1, ((uint32_t *)__infol->value)[i]);
	    break;
	  case IFT_UINT64:
#if __WORDSIZE == 64
	    rv = asprintf(&tmp2, "%s%lu", tmp1, ((uint64_t *)__infol->value)[i]);
#else
	    rv = asprintf(&tmp2, "%s%llu", tmp1, ((uint64_t *)__infol->value)[i]);
#endif
	    break;
	  case IFT_FLOAT:
	    rv = asprintf(&tmp2, "%s%f", tmp1, ((float *)__infol->value)[i]);
	    break;
	  case IFT_BYTE:
	    rv = asprintf(&tmp2, "%s%u", tmp1, ((uint8_t *)__infol->value)[i]);
	    break;
	  case IFT_STRING:
	    // cannot happen, caught with surrounding if statement

	  case IFT_ENUM:
	    rv = asprintf(&tmp2, "%s%s", tmp1, __interface->enum_tostring(__infol->enumtype, ((int *)__infol->value)[i]));
	    break;
	  }

	  if ( rv == -1 ) {
	    throw OutOfMemoryException("InterfaceFieldIterator::get_value_string(): asprintf() failed (1)");
	  }
	  
	  free(tmp1);
	  tmp1 = tmp2;
	  if ( (__infol->length > 1) && (i < __infol->length - 1) ) {
	    if (asprintf(&tmp2, "%s, ", tmp1) == -1) {
	      throw OutOfMemoryException("InterfaceFieldIterator::get_value_string(): asprintf() failed (2)");
	    }
	    free(tmp1);
	    tmp1 = tmp2;
	  }
	}

	__value_string = tmp1;
      } else {
	// it's a string, or a small number
	if ( __infol->length > 1 ) {
	  if (asprintf(&__value_string, "%s", (const char *)__infol->value) == -1) {
	    throw OutOfMemoryException("InterfaceFieldIterator::get_value_string(): asprintf() failed (3)");
	  }
	} else {
	  if (asprintf(&__value_string, "%c", *((const char *)__infol->value)) == -1) {
	    throw OutOfMemoryException("InterfaceFieldIterator::get_value_string(): asprintf() failed (4)");
	  }
	}
      }
    }
    return __value_string;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_BOOL ) {
    throw TypeMismatchException("Requested value is not of type bool");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((bool *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT8 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((int8_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT8 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((uint8_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT16 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((int16_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT16 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((uint16_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT32 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((int32_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT32 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((uint32_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT64 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((int64_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT64 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((uint64_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_FLOAT ) {
    throw TypeMismatchException("Requested value is not of type float");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((float *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_BYTE ) {
    throw TypeMismatchException("Requested value is not of type float");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    return ((uint8_t *)__infol->value)[index];
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_BOOL ) {
    throw TypeMismatchException("Requested value is not of type bool");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    return (bool *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT8 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else {
    return (int8_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT8 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else {
    return (uint8_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT16 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else {
    return (int16_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT16 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else {
    return (uint16_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT32 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else {
    return (int32_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT32 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else {
    return (uint32_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_INT64 ) {
    throw TypeMismatchException("Requested value is not of type int");
  } else {
    return (int64_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_UINT64 ) {
    throw TypeMismatchException("Requested value is not of type unsigned int");
  } else {
    return (uint64_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_FLOAT ) {
    throw TypeMismatchException("Requested value is not of type float");
  } else {
    return (float *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_BYTE ) {
    throw TypeMismatchException("Requested value is not of type float");
  } else {
    return (uint8_t *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot get value of end element");
  } else if ( __infol->type != IFT_STRING ) {
    throw TypeMismatchException("Requested value is not of type string");
  } else {
    return (const char *)__infol->value;
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_BOOL ) {
    throw TypeMismatchException("Field to be written is not of type bool");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(bool);
    memcpy((void *) dst, &v, sizeof(bool));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT8 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(int8_t);
    memcpy((void *) dst, &v, sizeof(int8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT8 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(uint8_t);
    memcpy((void *) dst, &v, sizeof(uint8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT16 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(int16_t);
    memcpy((void *) dst, &v, sizeof(int16_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT16 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(uint16_t);
    memcpy((void *) dst, &v, sizeof(uint16_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT32 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(int32_t);
    memcpy((void *) dst, &v, sizeof(int32_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT32 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(uint32_t);
    memcpy((void *) dst, &v, sizeof(uint32_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT64 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(int64_t);
    memcpy((void *) dst, &v, sizeof(int64_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT64 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(uint64_t);
    memcpy((void *) dst, &v, sizeof(uint64_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_FLOAT ) {
    throw TypeMismatchException("Field to be written is not of type float");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(float);
    memcpy((void *) dst, &v, sizeof(float));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_BYTE ) {
    throw TypeMismatchException("Field to be written is not of type byte");
  } else if (index >= __infol->length) {
    throw OutOfBoundsException("Field index out of bounds", index, 0, __infol->length);
  } else {
    char* dst = (char *) __infol->value + index * sizeof(uint8_t);
    memcpy((void *) dst, &v, sizeof(uint8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_BOOL ) {
    throw TypeMismatchException("Field to be written is not of type bool");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(bool));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT8 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(int8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT8 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(uint8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT16 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(int16_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT16 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(uint16_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT32 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(int32_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT32 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(uint32_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_INT64 ) {
    throw TypeMismatchException("Field to be written is not of type int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(int64_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_UINT64 ) {
    throw TypeMismatchException("Field to be written is not of type unsigned int");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(uint64_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_FLOAT ) {
    throw TypeMismatchException("Field to be written is not of type float");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(float));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_BYTE ) {
    throw TypeMismatchException("Field to be written is not of type byte");
  } else if (__infol->length == 1) {
    throw TypeMismatchException("Field %s is not an array", __infol->name);
  } else {
    memcpy(__infol->value, v, __infol->length * sizeof(uint8_t));
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
  if ( __infol == NULL ) {
    throw NullPointerException("Cannot set value of end element");
  } else if ( __infol->type != IFT_STRING ) {
    throw TypeMismatchException("Field to be written is not of type string");
  } else {
    strncpy((char *) __infol->value, v, __infol->length);
  }
}

} // end namespace fawkes
