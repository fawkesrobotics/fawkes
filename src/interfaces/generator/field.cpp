 
/***************************************************************************
 *  field.cpp - Interface generator field representation
 *
 *  Generated: Wed Oct 11 18:16:15 2006
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

#include <interfaces/generator/field.h>
#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>

#include <stdlib.h>


/** @class InterfaceField interfaces/generator/field.h
 * Interface generator internal representation of a field as parsed from
 * the XML template file.
 */


/** Constructor.
 * @param enum_constants enumeration constants that are available and which can be
 * used as value type.
 */
InterfaceField::InterfaceField(std::vector<InterfaceEnumConstant> *enum_constants)
{
  this->enum_constants = enum_constants;
  length = "";
  length_value = 0;
  is_enum_type = false;
}


/** Get name of field.
 * @return name of field.
 */
std::string
InterfaceField::getName() const
{
  return name;
}


/** Get type of field.
 * @return type of field.
 */
std::string
InterfaceField::getType() const
{
    return type;
}


/** Get comment of field.
 * @return comment of field.
 */
std::string
InterfaceField::getComment() const
{
    return comment;
}


/** Get type as used for accessor methods of class.
 * @return accessor type
 */
std::string
InterfaceField::getAccessType() const
{
  if (type == "string") {
    return "char *";
  } else {
    if ( length != "" ) {
      if (type == "byte") {
	return "uint8_t *";
      } else if (type == "float" || type == "double" || type == "bool" || is_enum_type) {
	return type + " *";
      } else {
	return type + "_t *";
      }
    } else {
      if (type == "byte") {
	return "uint8_t";
      } else if (type == "float" || type == "double" || type == "bool" || is_enum_type) {
	return type;
      } else {
	return type + "_t";
      }
    }
  }
}


/** Get non-array accessor type.
 * @return accessor type
 */
std::string
InterfaceField::getPlainAccessType() const
{
  if (type == "string") {
    return "char *";
  } else if (type == "byte") {
    return "uint8_t";
  } else if (type == "float" || type == "double" || type == "bool" || is_enum_type) {
    return type;
  } else {
    return type + "_t";
  }
}


/** Get type used to formulate struct.
 * @return struct type
 */
std::string
InterfaceField::getStructType() const
{
  if (type == "string") {
    return "char";
  } else if (type == "byte") {
    return "uint8_t";
  } else if (type == "float" || type == "double" || type == "bool" || is_enum_type) {
    return type;
  } else {
    return type + "_t";
  }
}


/** Check if type is an enum type.
 * @return true if the type of this field is an enum type, false otherwise
 */
bool
InterfaceField::isEnumType() const
{
  return is_enum_type;
}

/** Get field length.
 * @return field length
 */
std::string
InterfaceField::getLength() const
{
  return length;
}


/** Get length value.
 * This gives the length of the value as a uint instead of a string
 * which is sufficient for the generation of the interface but may not
 * be sufficient for more elaborated usage.
 * @return length of the value
 */
unsigned int
InterfaceField::getLengthValue() const
{
  return length_value;
}


/** Get valid for time.
 * @return valid for time
 */
std::string
InterfaceField::getValidFor() const
{
  return validfor;
}


/** Get default value.
 * @return default value
 */
std::string
InterfaceField::getDefaultValue() const
{
  return default_value;
}


/** Get flags.
 * @return flags.
 */
std::vector<std::string>
InterfaceField::getFlags() const
{
  return flags;
}


/** Set type of field.
 * @param type new type of field.
 */
void
InterfaceField::setType(const std::string &type)
{
  is_enum_type = false;
  if ( enum_constants != NULL ) {
    for (std::vector<InterfaceEnumConstant>::iterator i = enum_constants->begin(); i != enum_constants->end(); ++i) {
      if ( type == (*i).getName() ) {
	is_enum_type = true;
      }
    }
  }
  this->type = type;
}


/** Set name of field.
 * @param name new name of field.
 */
void
InterfaceField::setName(const std::string &name)
{
  this->name = name;
}


/** Set comment of field.
 * @param comment new comment of field.
 */
void
InterfaceField::setComment(const std::string &comment)
{
  this->comment = comment;
}


/** Set length of field.
 * @param length set length of field.
 */
void
InterfaceField::setLength(const std::string &length)
{
  this->length_value = (unsigned int)atoi(length.c_str());
  this->length = length;
}


/** Set valid for time.
 * @param validfor new valid for time
 */
void
InterfaceField::setValidFor(const std::string &validfor)
{
  this->validfor = validfor;
}


/** Set default value.
 * @param default_value new default value
 */
void
InterfaceField::setDefaultValue(const std::string &default_value)
{
  this->default_value = default_value;
}


/** Set flags.
 * @param flags new flags of field
 */
void
InterfaceField::setFlags(const std::vector<std::string> &flags)
{
  this->flags = flags;
}


/** Tokenize given string.
 * @param str tsring to tokenize
 * @param tokens vector where result will be stored
 * @param delimiters string with delimiters.
 */
void
InterfaceField::tokenize(const std::string&   str,
			 std::vector<std::string>& tokens,
			 const std::string&   delimiters)
{
  // Skip delimiters at beginning.
  std::string::size_type last_pos = str.find_first_not_of(delimiters, 0);
  // Find first "non-delimiter".
  std::string::size_type pos      = str.find_first_of(delimiters, last_pos);

  while (std::string::npos != pos || std::string::npos != last_pos) {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(last_pos, pos - last_pos));
    // Skip delimiters.  Note the "not_of"
    last_pos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, last_pos);
  }
}


/** Set attribute.
 * @param attr_name attribute name
 * @param attr_value attribute value.
 */
void
InterfaceField::setAttribute(const std::string &attr_name, const std::string &attr_value)
{
  if ( attr_name == "name" ) {
    setName(attr_value);
  } else if ( attr_name == "type" ) {
    setType(attr_value);
  } else if ( attr_name == "length" ) {
    setLength(attr_value);
  } else if ( attr_name == "validfor" ) {
    setValidFor(attr_value);
  } else if ( attr_name == "default" ) {
    setDefaultValue(attr_value);
  } else if ( attr_name == "flags" ) {
    tokenize(attr_value, flags, ",");
  }
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
InterfaceField::valid()
{
  if ( ! InterfaceDataTypeChecker::validType(type, enum_constants) ) {
    throw InterfaceGeneratorInvalidTypeException("field", name.c_str(), type.c_str());
  }
  if ( (name.length() == 0) || (name.find(" ") != std::string::npos) ) {
    throw InterfaceGeneratorInvalidValueException("name", "string", "name must not contain spaces");
  }
  if ( (length.length() > 0) && ! InterfaceDataTypeChecker::validValue("uint32", length) ) {
    throw InterfaceGeneratorInvalidValueException("length", "uint32", length.c_str());
  }
  if ( (validfor.length() > 0) && ! InterfaceDataTypeChecker::validValue("uint32", validfor) ) {
    throw InterfaceGeneratorInvalidValueException("validfor", "uint32", validfor.c_str());
  }
  if ( (default_value.length() > 0) &&
       ! InterfaceDataTypeChecker::validValue(type, default_value) ) {
    throw InterfaceGeneratorInvalidValueException("default", type.c_str(), validfor.c_str());
  }
  for (std::vector<std::string>::iterator i = flags.begin(); i != flags.end(); ++i) {
    if ( *i != "changed_indicator" ) {
      throw InterfaceGeneratorInvalidFlagException(name.c_str(), (*i).c_str());
    }
  }
  /*
  if ( (type == "char") && (length.length() == 0) ) {
    throw InterfaceGeneratorMissingAttributeException(name.c_str(), type.c_str(), "length");
  }
  */
}


/** Check order of two elements.
 * The overall order is like the following:
 * 1. unsigned int
 * 2. int
 * 3. unsigned long int
 * 4. long int
 * 5. float
 * 6. double
 * 7. bool
 * 8. byte
 * 9. char *
 * @param f field to compare to
 * @return true, if current instance is small than f, false otherwise
 */
bool
InterfaceField::operator< (const InterfaceField &f) const
{
  if ( (type == "unsigned int") ) {
    return (f.type != "unsigned int");

  } else if ( type == "int" ) {
    return ( (f.type != "int") &&
	     (f.type != "unsigned int") );


  } else if ( type == "unsigned long int" ) {
    return ( (f.type != "unsigned long int") &&
	     (f.type != "unsigned int") &&
	     (f.type != "int") );

  } else if ( type == "long int" ) {
    return ( (f.type != "long int") &&
	     (f.type != "unsigned int") &&
	     (f.type != "int") &&
	     (f.type != "unsigned long int") );

  } else if ( type == "float" ) {
    return ( (f.type != "float") &&
	     (f.type != "unsigned int") &&
	     (f.type != "int") );

  } else if ( type == "double" ) {
    return ( (f.type != "double") &&  
	     (f.type != "unsigned int") &&
	     (f.type != "int") &&
	     (f.type != "float") );

  } else if ( type == "bool" ) {
    return ( (f.type != "bool") &&
	     (f.type != "double") &&  
	     (f.type != "unsigned int") &&
	     (f.type != "int") &&
	     (f.type != "float") );

  } else if ( type == "byte" ) {
    return ( (f.type != "byte") &&
	     (f.type != "bool") &&
	     (f.type != "double") &&  
	     (f.type != "unsigned int") &&
	     (f.type != "int") &&
	     (f.type != "float") );

  } else {
    // char or unknown, char is always last and thus >=
    return false;
  }
}
