 
/***************************************************************************
 *  type_checker.cpp - Interface generator type checker
 *
 *  Generated: Wed Oct 11 15:39:10 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>

/** @class InterfaceDataTypeChecked interfaces/generator/type_checker.h
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
 * - char
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
  if (  (type == "int") ||
	(type == "long int") ||
	(type == "unsigned int") ||
	(type == "unsigned long int") ||
	(type == "bool") ||
	(type == "char") ||
	(type == "float") ||
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
  if ( (type == "int") || ( type == "long int")) {
    char *endptr;
    strtol(value.c_str(), &endptr, 10);
    return ( (endptr != NULL) && (endptr[0] == '\0'));
  } else if ((type == "unsigned int") || (type == "unsigned long int")) {  
    char *endptr;
    int val = strtol(value.c_str(), &endptr, 10);
    if ( (endptr == NULL) || (endptr[0] != '\0') ) {
      return false;
    } else {
      return (val >= 0);
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
    strtod(value.c_str(), &endptr);
    return ((endptr != NULL) && (endptr[0] == '\0'));
  } else if ( type == "char" ) {
    return true;
  } else {
    return false;
  }
}
