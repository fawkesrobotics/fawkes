
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

#include <interfaces/generator/checker.h>
#include <interfaces/generator/exceptions.h>
#include <interfaces/generator/pseudomap.h>

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
InterfacePseudoMap::InterfacePseudoMap(std::string name,
                                       std::string type,
                                       std::string keytype,
                                       std::string comment)
{
	name_    = name;
	type_    = type;
	keytype_ = keytype;
	comment_ = comment;
}

/** Get name of field.
 * @return name of field.
 */
std::string
InterfacePseudoMap::getName() const
{
	return name_;
}

/** Get type of field.
 * @return type of field.
 */
std::string
InterfacePseudoMap::getType() const
{
	return type_;
}

/** Get comment of field.
 * @return comment of field.
 */
std::string
InterfacePseudoMap::getComment() const
{
	return comment_;
}

/** Get type of key value.
 * @return type of key
 */
std::string
InterfacePseudoMap::getKeyType() const
{
	return keytype_ + "_t";
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
	if ((name_.length() == 0) || (name_.find(" ") != std::string::npos)) {
		throw InterfaceGeneratorInvalidValueException("name",
		                                              "string",
		                                              "name must neither be empty nor contain spaces");
	}
	if (type_.length() == 0) {
		throw InterfaceGeneratorInvalidValueException("type", "string", "type must not be empty");
	}
	if ((keytype_ != "int8") && (keytype_ != "int16") && (keytype_ != "int32")
	    && (keytype_ != "int64") && (keytype_ != "uint8") && (keytype_ != "uint16")
	    && (keytype_ != "uint32") && (keytype_ != "uint64")) {
		throw InterfaceGeneratorInvalidValueException("keytype",
		                                              "string",
		                                              "Pseudo map keys can only be of a numeric type");
	}
	if (keytype_.length() == 0) {
		throw InterfaceGeneratorInvalidValueException("keytype",
		                                              "string",
		                                              "key type must not be empty");
	}
}

/** Add reference.
 * @param fieldname name of the field that is referenced
 * @param key key of the field in the pseudo map
 */
void
InterfacePseudoMap::addRef(std::string fieldname, std::string key)
{
	parefs_.push_back(make_pair(fieldname, key));
}

/** Get reference list.
 * @return reference list
 */
InterfacePseudoMap::RefList &
InterfacePseudoMap::getRefList()
{
	return parefs_;
}
