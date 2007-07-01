 
/***************************************************************************
 *  field.cpp - Interface generator field representation
 *
 *  Generated: Wed Oct 11 18:16:15 2006
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

#include <interfaces/generator/field.h>
#include <interfaces/generator/type_checker.h>
#include <interfaces/generator/exceptions.h>

#include <stdlib.h>


/** @class InterfaceField interfaces/generator/field.h
 * Interface generator internal representation of a field as parsed from
 * the XML template file.
 */


/** Constructor */
InterfaceField::InterfaceField()
{
  bits_val = 0;
  length = "";
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
  if ( type == "char" ) {
    return "char *";
  } else if ( length != "" ) {
    return type + " *";
  } else {
    return type;
  }
}


/** Get field length.
 * @return field length
 */
std::string
InterfaceField::getLength() const
{
  return length;
}

unsigned int
InterfaceField::getLengthValue() const
{
  return length_value;
}


/** Get number of bits as string-
 * @return number of bits as string.
 */
std::string
InterfaceField::getBits() const
{
  return bits;
}


/** Get number of bits as value.
 * @return number of bits as value.
 */
unsigned int
InterfaceField::getNumBits() const
{
  return bits_val;
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


/** Set number of bits.
 * @param bits new number of bits.
 */
void
InterfaceField::setBits(const std::string &bits)
{
  bits_val = atoi(bits.c_str());
  if ( bits_val >= 32 ) {
    throw InterfaceGeneratorInvalidValueException("bits", "unsigned int", "must be in range [1..31]");
  }
  this->bits = bits;
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
  } else if ( attr_name == "bits" ) {
    setBits(attr_value);
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
 * @exception InterfaceGeneratorInvalidAttributeException thrown if bits supplied for
 * field not of type int and unsigned int
 * @exception InterfaceGeneratorInvalidFlagException thrown if invalid flag has been
 * supplied.
 */
void
InterfaceField::valid()
{
  if ( ! InterfaceDataTypeChecker::validType(type) ) {
    throw InterfaceGeneratorInvalidTypeException("field", name.c_str(), type.c_str());
  }
  if ( (name.length() == 0) || (name.find(" ") != std::string::npos) ) {
    throw InterfaceGeneratorInvalidValueException("name", "string", "name must not contain spaces");
  }
  if ( (length.length() > 0) && ! InterfaceDataTypeChecker::validValue("unsigned int", length) ) {
    throw InterfaceGeneratorInvalidValueException("length", "unsigned int", length.c_str());
  }
  if ( bits.length() > 0 ) {
    if ( (type != "int") &&
	 (type != "unsigned int") ) {
      throw InterfaceGeneratorInvalidAttributeException(name.c_str(), type.c_str(), "bits");
    } else if ( ! InterfaceDataTypeChecker::validValue("unsigned int", bits) ) {
      throw InterfaceGeneratorInvalidValueException("bits", "unsigned int", bits.c_str());
    }
  }
  if ( (validfor.length() > 0) && ! InterfaceDataTypeChecker::validValue("unsigned int", validfor) ) {
    throw InterfaceGeneratorInvalidValueException("validfor", "unsigned int", validfor.c_str());
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
  if ( (type == "char") && (length.length() == 0) ) {
    throw InterfaceGeneratorMissingAttributeException(name.c_str(), type.c_str(), "length");
  }
}


/** Check order of two elements.
 * The overall order is like the following:
 * 1. unsigned int (w/o bit fields)
 * 2. int (w/o bit fields)
 * 3. float
 * 4. double
 * 5. bit fields
 * 6. bool
 * 7. char *
 * @param f field to compare to
 * @return true, if current instance is small than f, false otherwise
 */
bool
InterfaceField::operator< (const InterfaceField &f) const
{
  if ( (type == "unsigned int") && (bits.length() == 0) ) {
    return ( (f.type != "unsigned int") || (f.bits.length() > 0));

  } else if ( (type == "int") && (bits.length() == 0) ) {
    return ( ((f.type != "int") &&
	      (f.type != "unsigned int") ) ||
	     (f.bits.length() > 0) );

  } else if ( type == "float" ) {
    return ( (f.type != "float") &&
	     (((f.type != "unsigned int") &&
	       (f.type != "int") ) ||
	      (f.bits.length() > 0) ) );

  } else if ( type == "double" ) {
    return ( (f.type != "double") &&  
	     (((f.type != "unsigned int") &&
	       (f.type != "int") &&
	       (f.type != "float") ) ||
	      (f.bits.length() > 0)
	      )
	     );

  } else if ( (type == "unsigned int") && (bits.length() > 0) ) {
    return ( (f.type != "unsigned int") &&
	     ((f.type != "int") || (f.bits.length() > 0) ) );

  } else if ( (type == "int") && (bits.length() > 0) ) {
    return ( (f.type != "unsigned int") &&
	     (f.type != "int") );

  } else {
    // char or unknown, char is always last and thus >=
    return false;
  }
}
