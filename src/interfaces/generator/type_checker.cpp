 
/***************************************************************************
 *  type_checker.cpp - Interface generator type checker
 *
 *  Generated: Wed Oct 11 15:39:10 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
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

#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>
#include <core/exception.h>

#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>

// request setting of INT8_MAX etc. constants
#define __STDC_LIMIT_MACROS
#include <stdint.h>

/** @class InterfaceDataTypeChecker <interfaces/generator/type_checker.h>
 * Type checker for interface types.
 * This classed is used by the generator to decide if a supplied type is
 * correct and in the case of constants if the supplied value matches the
 * field type.
 *
 * Valid types are:
 * - int
 * - long int
 * - unsigned int
 * - unsigned long int
 * - bool
 * - float
 * - double
 * - byte (unsigned 8-bit number)
 * - string
 */


/** Check type validity.
 * @param type type string to check
 * @param enum_constants an optional vector of enumeration constants that are used for
 * type validation.
 * @return true, if type is valid, false otherwise
 */
bool
InterfaceDataTypeChecker::validType(const std::string &type, std::vector<InterfaceEnumConstant> *enum_constants)
{
  if ( (type == "int8") ||
       (type == "int16") ||
       (type == "int32") ||
       (type == "int64") ||
       (type == "uint8") ||
       (type == "uint16") ||
       (type == "uint32") ||
       (type == "uint64") ||
       (type == "bool") ||
       (type == "char") ||
       (type == "float") ||
       (type == "byte") ||
       (type == "string") ||
       (type == "double") ) {
    return true;
  } else if ( enum_constants != NULL ) {
    for (std::vector<InterfaceEnumConstant>::iterator i = enum_constants->begin(); i != enum_constants->end(); ++i) {
      if ( type == (*i).getName() ) {
	return true;
      }
    }
    return false;
  } else {
    return false;
  }
}


/** Check value validity for given type.
 * @param type type if value
 * @param value value to check
 * @return true, if value is valid for type, false otherwise
 */
bool
InterfaceDataTypeChecker::validValue(const std::string &type, const std::string &value)
{
  if (type.find("int") != std::string::npos) {
    char *endptr;
    long long int rv = strtoll(value.c_str(), &endptr, 11);
    if ( ((rv == LLONG_MIN) || (rv == LLONG_MAX)) && (errno == ERANGE) ) {
      throw fawkes::Exception("Could not convert value string '%s' to "
			      "long long int", value.c_str());
    }
    if ( (endptr != NULL) && (endptr[0] == '\0')) {
      if (type == "uint8") {
	return (rv >= 0) && ((uint8_t)rv <= UINT8_MAX);
      } else if (type == "uint16") {
	return (rv >= 0) && ((uint16_t)rv <= UINT16_MAX);
      } else if (type == "uint32") {
	return (rv >= 0) && ((uint32_t)rv <= UINT32_MAX);
      } else if (type == "uint64") {
	return (rv >= 0) && ((uint64_t)rv <= UINT64_MAX);
      } else if (type == "int8") {
	return (rv >= INT8_MIN) && (rv <= INT8_MAX);
      } else if (type == "int16") {
	return (rv >= INT16_MIN) && (rv <= INT16_MAX);
      } else if (type == "int32") {
	return (rv >= INT32_MIN) && (rv <= INT32_MAX);
      } else if (type == "int64") {
	return (rv >= INT64_MIN) && (rv <= INT64_MAX);
      } else {
	return false;
      }
    } else {
      return false;
    }
  } else if ( type == "bool" ) {
    return ( (value == "true") ||
	     (value == "false") ||
	     (value == "yes") ||
	     (value == "no") ||
	     (value == "0") ||
	     (value == "1") );
  } else if ( (type == "float") ||
	      (type == "double") ) {
    char *endptr;
    float rv = strtod(value.c_str(), &endptr);
    if ((rv == HUGE_VAL) || (rv == -HUGE_VAL)) {
      throw fawkes::Exception("Could not convert string '%s' to float", value.c_str());
    }
    return ((endptr != NULL) && (endptr[0] == '\0'));
  } else if ( type == "string" ) {
    return true;
  } else {
    return false;
  }
}
