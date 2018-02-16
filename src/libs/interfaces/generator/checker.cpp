 
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

#include <interfaces/generator/checker.h>
#include <interfaces/generator/exceptions.h>
#include <core/exception.h>

#include <cstdlib>
#include <climits>
#include <cmath>
#include <cerrno>

// request setting of INT8_MAX etc. constants
#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#include <stdint.h>

/** @class InterfaceChecker <interfaces/generator/checker.h>
 * @brief Check interface type and identifier validity.
 */


/** Decide if a supplied type is correct and in the case of constants if the
 * supplied value matches the field type.
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
 * @param type type string to check
 * @param enum_constants an optional vector of enumeration constants that are used for
 * type validation.
 * @return true, if type is valid, false otherwise
 */
bool
InterfaceChecker::validType(const std::string &type, std::vector<InterfaceEnumConstant> *enum_constants)
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
    std::vector<InterfaceEnumConstant>::iterator i;
    for (i = enum_constants->begin(); i != enum_constants->end(); ++i) {
      if ( type == (*i).get_name() ) {
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
InterfaceChecker::validValue(const std::string &type, const std::string &value)
{
  if (type.find("int") != std::string::npos) {
    errno = 0;
    char *endptr;
    long long int rv = strtoll(value.c_str(), &endptr, 10);
    if ( ((rv == LLONG_MIN) || (rv == LLONG_MAX)) && (errno == ERANGE) ) {
      throw fawkes::Exception("Could not convert value string '%s' to "
			      "long long int", value.c_str());
    }
    if ( (endptr != NULL) && (endptr[0] == '\0')) {
      if (type == "uint8") {
	return (rv >= 0) && (rv <= UINT8_MAX);
      } else if (type == "uint16") {
	return (rv >= 0) && (rv <= UINT16_MAX);
      } else if (type == "uint32") {
	return (rv >= 0) && (rv <= UINT32_MAX);
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

/** Check identifiers.
 * Identifiers that are used by the implementation and cannot be used
 * as field or message names are rejected.
 * @param name identifier to check
 * @param reserved_names reserved names to reject
 * @return true if name is valid, false otherwise
 */
bool
InterfaceChecker::validName(const std::string &name, const std::set<std::string> &reserved_names)
{
  if (name.substr(0, 4) == "set_")
    return reserved_names.find(name.substr(5)) == reserved_names.end();
  if (name.substr(0, 3) == "is_")
    return reserved_names.find(name.substr(4)) == reserved_names.end();
  else
    return reserved_names.find(name) == reserved_names.end();
}


const std::set<std::string> reserved_names_interface() {
  return {
    "id", "clone", "oftype", "datachunk", "datasize",
    "type", "uid", "serial", "mem_serial", "hash", "hash_size",
    "hash_printable", "writer", "validity", "valid",
    "owner", "from_chunk", "create_message", "copy_values",
    "enum_tostring", "resize_buffers", "num_buffers",
    "copy_shared_to_buffer", "copy_private_to_buffer", "read_from_buffer",
    "compare_buffers", "buffer_timestamp", "read", "write", "has_writer",
    "num_readers", "writer", "readers", "changed",
    "auto_timestamping", "timestamp", "clock", "mark_data_changed",
    "get_message_types", "msgq_enqueue", "msgq_enqueue_copy",
    "msgq_remove", "msgq_size", "msgq_flush", "msgq_lock", "msgq_try_lock",
    "msgq_unlock", "msgq_pop", "msgq_first", "msgq_empty", "msgq_append",
    "msgq_first_is", "msgq_first", "msgq_first_safe", "msgq_begin",
    "msgq_end", "fields", "fields_end", "num_fields", "parse_uid",
    "reserved_names", "message_valid", "add_fieldinfo", "add_messageinfo",
    "data_ptr", "data_size", "data_changed", "data_ts", "type_id",
    "instance_serial", "mediators", "memory", "readwrite", "owner"
  };
};



const std::set<std::string> reserved_names_message() {
  return {
    "id", "mark_enqueued", "enqueued", "time_enqueued", "sender_id", "sender_thread_name",
    "interface", "type", "fields", "fields_end", "num_fields", "datachunk", "datasize",
    "hops", "from_chunk", "recipient", "clone", "of_type", "as_type"
  };
};
