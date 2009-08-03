
/***************************************************************************
 *  string_content.h - A dynamically sized string message content
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

#ifndef __NETCOMM_UTILS_STRING_CONTENT_H_
#define __NETCOMM_UTILS_STRING_CONTENT_H_

#include <netcomm/fawkes/message_content.h>
#include <sys/types.h>

namespace fawkes {

class StringContent : public FawkesNetworkMessageContent
{
 public:
  StringContent(const char *initial_string);
  StringContent(unsigned int cid, unsigned int msgid, void *payload, size_t payload_size);
  virtual ~StringContent();

  void          set_string(const char *s);
  const char *  get_string() const;
  size_t        get_string_length();

  virtual void  serialize();

 private:
  bool    __string_owner;
  char *  __string;

};

} // end namespace fawkes

#endif
