 
/***************************************************************************
 *  message.cpp - Interface generator message representation
 *
 *  Generated: Wed Oct 11 22:24:16 2006 (Germany - Slowakia  4:1)
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

#include <interfaces/generator/message.h>
#include <interfaces/generator/exceptions.h>


/** @class InterfaceMessage message.h <interfaces/generator/message.h>
 * Interface generator internal representation of a message as parsed from
 * the XML template file.
 */

/** Constructor
 * @param name name of message
 * @param comment comment of message
 */
InterfaceMessage::InterfaceMessage(const std::string &name, const std::string &comment)
{
  if ( name.find("Message") != std::string::npos )  {
    this->name = name;
  } else {
    this->name = name + "Message";
  }
  this->comment = comment;
  fields.clear();
}


/** Get name of message.
 * @return name of message.
 */
std::string
InterfaceMessage::getName()
{
  return name;
}


/** Get comment of message.
 * @return comment of message.
 */
std::string
InterfaceMessage::getComment()
{
  return comment;
}


/** Set fields of message.
 * @param fields fields of message.
 */
void
InterfaceMessage::setFields(const std::vector<InterfaceField> &fields)
{
  this->fields = fields;
}


/** Get fields of message.
 * @return fields of message.
 */
std::vector<InterfaceField>
InterfaceMessage::getFields()
{
  return fields;
}
