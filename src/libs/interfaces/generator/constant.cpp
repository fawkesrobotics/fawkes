 
/***************************************************************************
 *  constant.cpp - Interface generator constant representation
 *
 *  Generated: Wed Oct 11 15:33:39 2006
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

#include <interfaces/generator/constant.h>
#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>

/** @class InterfaceConstant interfaces/generator/constant.h
 * Interface generator internal representation of a constant as parsed from
 * the XML template file.
 */


/** Constructor
 * @param name name of constant
 * @param type type of constant
 * @param value value of constant
 * @param comment comment of message
 * @exception InterfaceGeneratorInvalidTypeException thrown if InterfaceDataTypeChecker
 * reports an invalid type.
 * @exception InterfaceGeneratorInvalidValueException thrown if InterfaceDataTypeChecker
 * reports an illegal value for the given type.
 */
InterfaceConstant::InterfaceConstant(const std::string &name, const std::string &type,
				     const std::string &value, const std::string &comment)
{
  if ( ! InterfaceDataTypeChecker::validType(type) ) {
    throw InterfaceGeneratorInvalidTypeException("constant", name.c_str(), type.c_str());
  }
  if ( ! InterfaceDataTypeChecker::validValue(type, value) ) {
    throw InterfaceGeneratorInvalidValueException(name.c_str(), type.c_str(), value.c_str());
  }

  this->name  = name;
  this->type  = type;
  if ( type == "string" ) {
    this->value = std::string("\"") + value + "\"";
  } else {
    this->value = value;
  }
  this->comment = comment;
}


/** Get name of constant.
 * @return name of constant.
 */
std::string
InterfaceConstant::getName()
{
  return name;
}


/** Get value of constant.
 * @return value of constant.
 */
std::string
InterfaceConstant::getValue()
{
  return value;
}


/** Get type of constant.
 * @return type of constnat.
 */
std::string
InterfaceConstant::getType()
{
  if (type == "string") {
    return "char *";
  } else if (type == "byte") {
    return "uint8_t";
  } else if (type == "float" || type == "double" || type == "bool") {
    return type;
  } else {
    return type + "_t";
  }
}


/** Get comment of constant.
 * @return comment of constant.
 */
std::string
InterfaceConstant::getComment()
{
  return comment;
}
