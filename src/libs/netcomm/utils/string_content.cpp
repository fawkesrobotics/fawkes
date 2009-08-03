
/***************************************************************************
 *  string_content.cpp - A dynamically sized string message content
 *
 *  Created: Mon Mar 17 13:58:03 2008
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <netcomm/utils/string_content.h>

#include <core/exception.h>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class StringContent <netcomm/utils/string_content.h>
 * Content for a variable length string.
 * This content class can be used with a FawkesNetworkMessage. It takes a
 * single string of variable size and stuffs it into a message.
 *
 * @author Tim Niemueller
 */


/** Primary constructor.
 * @param initial_string initial string
 */
StringContent::StringContent(const char *initial_string)
{
  __string_owner = true;
  set_string(initial_string);
}


/** Constructor.
 * This ctor can be used with FawkesNetworkMessage::msgc().
 * @param cid component ID, ignored
 * @param msgid message ID, ignored
 * @param payload Payload, checked if it can be a valid string.
 * @param payload_size size in bytes of payload
 */
StringContent::StringContent(unsigned int cid, unsigned int msgid,
			     void *payload, size_t payload_size)
{
  __string_owner = false;
  _payload = payload;
  _payload_size = payload_size;
  __string = (char *)payload;
  if ( __string[payload_size - 1] != 0 ) {
    // string is not null-terminated, it has not been created with this class, error!
    throw Exception("String content of network message is not null-terminated.");
  }
}


/** Destructor. */
StringContent::~StringContent()
{
  if ( __string_owner ) {
    free(__string);
  }
}


/** Set the string.
 * Can only be called if the instance has been created with the primary constructor.
 * @param s the new string, must be null-terminated.
 */
void
StringContent::set_string(const char *s)
{
  if ( __string_owner ) {
    free(__string);
    __string = strdup(s);
    _payload = __string;
    _payload_size = strlen(__string) + 1;
  } else {
    throw Exception("Cannot set read-only string extracted from network message.");
  }
}


/** Get string.
 * @return null-terminated string
 */
const char *
StringContent::get_string() const
{
  return __string;
}


/** Get length of string.
 * @return string length
 */
size_t
StringContent::get_string_length()
{
  return _payload_size - 1;
}


void
StringContent::serialize()
{
  // nothing to do...
}

} // end namespace fawkes
