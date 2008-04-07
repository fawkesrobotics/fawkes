 
/***************************************************************************
 *  enum_constant.cpp - Interface generator enum constant representation
 *
 *  Generated: Wed Oct 11 19:41:56 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
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
  this->name  = name;
  this->comment = comment;
  items.clear();
}


/** Get name of enum constant.
 * return name of enum constant.
 */
std::string
InterfaceEnumConstant::getName()
{
  return name;
}


/** Get comment of enum constant.
 * @return comment of enum constant.
 */
std::string
InterfaceEnumConstant::getComment()
{
  return comment;
}


/** Get enumeration items.
 * @return vector of enum items. First item in pair contains item name, second item
 * the comment.
 */
std::vector< std::pair< std::string,std::string > >
InterfaceEnumConstant::getItems()
{
  return items;
}


/** Add an item.
 * @param name name of item
 * @param comment comment of item.
 */
void
InterfaceEnumConstant::addItem(std::string name, std::string comment)
{
  for ( std::vector< std::pair< std::string, std::string > >::iterator i = items.begin(); i != items.end(); ++i) {
    if ( (*i).first == name ) {
      throw InterfaceGeneratorAmbiguousNameException(name.c_str(), "enum item");
    }
  }
  std::pair< std::string, std::string > p(name, comment);
  items.push_back(p);
}
