
/***************************************************************************
 *  file_reply.h - Web request file reply
 *
 *  Created: Thu Oct 23 13:47:33 2008
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

#ifndef __PLUGINS_WEBVIEW_FILE_REPLY_H_
#define __PLUGINS_WEBVIEW_FILE_REPLY_H_

#include "reply.h"
#include <cstdio>

class DynamicFileWebReply : public DynamicWebReply
{
 public:
  DynamicFileWebReply(const char *filename);
  virtual ~DynamicFileWebReply();

  virtual size_t size();
  virtual size_t next_chunk(size_t pos, char *buffer, size_t buf_max_size);

 private:
  FILE   *__file;
  size_t  __size;
  
};

#endif
