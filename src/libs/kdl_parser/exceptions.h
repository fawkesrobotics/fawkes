/***************************************************************************
 *  exceptions.h - KDL Parser Exceptions
 *
 *  Created: Fri Feb 14 17:35:15 2014
 *  Copyright  2014 Till Hofmann
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

#ifndef __KDLPARSER_EXCEPTIONS_H_
#define __KDLPARSER_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {

/**
 * Unknown Joint Type
 */
class KDLParserUnknownJointTypeException : public Exception
{
  public:
    /** Constructor.
     * @param joint_type The unknown joint type in the URDF model
     */
    KDLParserUnknownJointTypeException(const char * joint_type)
    {
      append("Found unknown Joint Type %s", joint_type);
    }
};

/**
 * URDF Model generation failed for unknown reasons
 */
class KDLParserModelGenerationFailedException : public Exception
{
  public:
    KDLParserModelGenerationFailedException()
    {
      append("Could not generate robot model");
    }
};

/**
 * Tried to to parse Collada data which is not supported
 */
class URDFColladaNotSupportedException : public Exception
{
  public:
    URDFColladaNotSupportedException()
    {
      append("Collada Data models are currently not supported.");
    }
};

/**
 * Failed to parse XML Document
 */
class URDFXMLDocumentParseErrorException : public Exception
{
  public:
    URDFXMLDocumentParseErrorException()
    {
      append("Could not parse the XML document");
    }
};

/**
 * Failed to parse XML Element
 */
class URDFXMLElementParseErrorException : public Exception
{
  public:
    URDFXMLElementParseErrorException()
    {
      append("Could not parse the xml element");
    }
};

} // namespace fawkes






#endif
