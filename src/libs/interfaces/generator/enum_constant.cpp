 
/***************************************************************************
 *  enum_constant.cpp - Interface generator enum constant representation
 *
 *  Generated: Wed Oct 11 19:41:56 2006
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

#include <interfaces/generator/enum_constant.h>
#include <interfaces/generator/exceptions.h>
#include <interfaces/generator/checker.h>

/** @class InterfaceEnumConstant interfaces/generator/enum_constant.h
 * Interface generator internal representation of a enum constant as parsed
 * from the XML template file.
 */


/** Constructor.
 * @param name name of enumeration constant
 * @param comment comment of enumeration constant.
 */
InterfaceEnumConstant::InterfaceEnumConstant(const std::string &name,
					     const std::string &comment)
{
  if (!InterfaceChecker::validName(name, reserved_names_interface()))
    throw InterfaceGeneratorReservedIdentifierException("enum constant", name.c_str());
  __name  = name;
  __comment = comment;
  __items.clear();
}


/** Get name of enum constant.
 * @return name of enum constant.
 */
const std::string &
InterfaceEnumConstant::get_name() const
{
  return __name;
}


/** Get comment of enum constant.
 * @return comment of enum constant.
 */
const std::string &
InterfaceEnumConstant::get_comment() const
{
  return __comment;
}


/** Get enumeration items.
 * @return vector of enum items. First item in pair contains item name, second item
 * the comment.
 */
const std::vector<InterfaceEnumConstant::EnumItem> &
InterfaceEnumConstant::get_items() const
{
  return __items;
}


/** Add an item without custom value.
 * @param name name of item
 * @param comment comment of item.
 */
void
InterfaceEnumConstant::add_item(std::string name, std::string comment)
{
  if (!InterfaceChecker::validName(name, reserved_names_interface()))
    throw InterfaceGeneratorReservedIdentifierException("enum item", name.c_str());
  std::vector<EnumItem>::iterator i;
  for (i = __items.begin(); i != __items.end(); ++i) {
    if (i->name == name) {
      throw InterfaceGeneratorAmbiguousNameException(name.c_str(), "enum item");
    }
  }
  EnumItem p = {name, comment, false, 0};
  __items.push_back(p);
}


/** Add an item with custom value.
 * @param name name of item
 * @param comment comment of item.
 * @param value custom value
 */
void
InterfaceEnumConstant::add_item(std::string name, std::string comment, int value)
{
  std::vector<EnumItem>::iterator i;
  for (i = __items.begin(); i != __items.end(); ++i) {
    if (i->name == name) {
      throw InterfaceGeneratorAmbiguousNameException(name.c_str(), "enum item");
    }
  }
  EnumItem p = {name, comment, true, value};
  __items.push_back(p);
}
