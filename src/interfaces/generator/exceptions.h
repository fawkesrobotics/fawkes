 
/***************************************************************************
 *  exceptions.h - Interface generator exceptions
 *
 *  Generated: Tue Oct 10 18:11:59 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __INTERFACES_GENERATOR_EXCEPTIONS_H_
#define __INTERFACES_GENERATOR_EXCEPTIONS_H_

#include <core/exception.h>

/** Thrown if document was invalid.
 * This may happen if the document is not well-formed or if the file does not
 * exist.
 */
class InterfaceGeneratorInvalidDocumentException : public Exception {
 public:
  /** Constructor
   * @param msg message
   */
  InterfaceGeneratorInvalidDocumentException(const char *msg)
    : Exception("Interface document invalid.")
  {
    append(msg);
  }
};

/** Thrown if document contains illegal content.
 * This happens if there was content in the file which was while syntactically correct
 * semantically wrong. Examples for this are more than one data segment or no one at all.
 */
class InterfaceGeneratorInvalidContentException : public Exception {
 public:
  /** Constructor
   * @param msg message
   */
  InterfaceGeneratorInvalidContentException(const char *msg)
    : Exception()
  {
    append("Invalid content: %s", msg);
  }
};


/** Thrown if illegal type is supplied.
 * Only a few basic types are allowed. If a typo occured or an unknown type was used
 * this exception is thrown.
 */
class InterfaceGeneratorInvalidTypeException : public Exception {
 public:
  /** Constructor
   * @param item item type
   * @param name item name
   * @param type invalid data type
   */
  InterfaceGeneratorInvalidTypeException(const char *item, const char *name, const char *type)
    : Exception()
  {
    append("Invalid type for %s item '%s': %s", item, name, type);
  }
};

/** Thrown if illegal value is supplied.
 * Thrown if wrong value was supplied for a given value
 */
class InterfaceGeneratorInvalidValueException : public Exception {
 public:
  /** Constructor
   * @param name item name
   * @param type data type
   * @param value invalid value
   */
 InterfaceGeneratorInvalidValueException(const char *name, const char *type, const char *value)
    : Exception()
  {
    append("Invalid value for '%s' of type %s: %s", name, type, value);
  }
};

/** Thrown if illegal attribute is supplied.
 * Thrown if illegal attribute was supplied for a given value
 */
class InterfaceGeneratorInvalidAttributeException : public Exception {
 public:
  /** Constructor
   * @param name item name
   * @param type data type
   * @param attr invalid attribute
   */
 InterfaceGeneratorInvalidAttributeException(const char *name, const char *type, const char *attr)
    : Exception()
  {
    append("Attribute '%s' may not be specified for '%s' of type %s", attr, name, type);
  }
};


/** Thrown if illegal flag is supplied.
 * Thrown if illegal flag was supplied for a given value
 */
class InterfaceGeneratorInvalidFlagException : public Exception {
 public:
  /** Constructor
   * @param name item name
   * @param flag invalid flag
   */
 InterfaceGeneratorInvalidFlagException(const char *name, const char *flag)
    : Exception()
  {
    append("Illegal flag '%s' set for %s", flag, name);
  }
};


/** Thrown if required attribute is missing supplied.
 * Thrown if required attribute was not supplied for a given value
 */
class InterfaceGeneratorMissingAttributeException : public Exception {
 public:
  /** Constructor
   * @param name item name
   * @param type data type
   * @param attr missing attribute
   */
 InterfaceGeneratorMissingAttributeException(const char *name, const char *type, const char *attr)
    : Exception()
  {
    append("Attribute '%s' is required '%s' of type %s", attr, name, type);
  }
};


/** Thrown if name is ambiguous. */
class InterfaceGeneratorAmbiguousNameException : public Exception {
 public:
  /** Constructor
   * @param name ambiguous name
   * @param item item type
   */
 InterfaceGeneratorAmbiguousNameException(const char *name, const char *item)
    : Exception()
  {
    append("There are multiple %s items with name '%s'", item, name);
  }
};


#endif
