 
/***************************************************************************
 *  pseudomap.cpp - Interface generator pseudo representation
 *
 *  Created: Thu Nov 20 15:09:23 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <interfaces/generator/pseudomap.h>
#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>

#include <cstdlib>


/** @class InterfacePseudoMap "pseudomap.h"
 * Interface generator internal representation of a pseudo map as parsed from
 * the XML template file.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param name name of the pseudo map
 * @param type type of the values in the map
 * @param keytype type of the keys
 * @param comment comment of the pseudo map
 */
InterfacePseudoMap::InterfacePseudoMap(std::string name, std::string type,
				       std::string keytype, std::string comment)
{
  __name = name;
  __type = type;
  __keytype = keytype;
  __comment = comment;
}


/** Get name of field.
 * @return name of field.
 */
std::string
InterfacePseudoMap::getName() const
{
  return __name;
}


/** Get type of field.
 * @return type of field.
 */
std::string
InterfacePseudoMap::getType() const
{
    return __type;
}


/** Get comment of field.
 * @return comment of field.
 */
std::string
InterfacePseudoMap::getComment() const
{
    return __comment;
}


/** Get type of key value.
 * @return type of key
 */
std::string
InterfacePseudoMap::getKeyType() const
{
  return __keytype + "_t";
}



/** Assert validity.
 * Calling valid() acts like an assertion. An Exception is thrown if something is wrong.
 * @exception InterfaceGeneratorInvalidTypeException thrown if InterfaceDataTypeChecker
 * reports invalid type.
 * @exception InterfaceGeneratorInvalidValueException thrown if any supplied value is
 * illegal.
 * @exception InterfaceGeneratorInvalidFlagException thrown if invalid flag has been
 * supplied.
 */
void
InterfacePseudoMap::valid()
{
  if ( (__name.length() == 0) || (__name.find(" ") != std::string::npos) ) {
    throw InterfaceGeneratorInvalidValueException("name", "string", "name must neither be empty nor contain spaces");
  }
  if (__type.length() == 0) {
    throw InterfaceGeneratorInvalidValueException("type", "string", "type must not be empty");
  }
  if ( (__keytype != "int8") && (__keytype != "int16") &&
       (__keytype != "int32") && (__keytype != "int64") &&
       (__keytype != "uint8") && (__keytype != "uint16") &&
       (__keytype != "uint32") && (__keytype != "uint64") ) {
    throw InterfaceGeneratorInvalidValueException("keytype", "string", "Pseudo map keys can only be of a numeric type");
  }
  if (__keytype.length() == 0) {
    throw InterfaceGeneratorInvalidValueException("keytype", "string", "key type must not be empty");
  }
}


/** Add reference.
 * @param fieldname name of the field that is referenced
 * @param key key of the field in the pseudo map
 */
void
InterfacePseudoMap::addRef(std::string fieldname, std::string key)
{
  __parefs.push_back(make_pair(fieldname, key));
}


/** Get reference list.
 * @return reference list
 */
InterfacePseudoMap::RefList &
InterfacePseudoMap::getRefList()
{
  return __parefs;
}
