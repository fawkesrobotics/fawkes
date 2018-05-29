
/***************************************************************************
 *  file_reply.h - Web request file reply
 *
 *  Created: Thu Oct 23 13:47:33 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_WEBVIEW_FILE_REPLY_H_
#define __LIBS_WEBVIEW_FILE_REPLY_H_

#include <webview/reply.h>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class DynamicFileWebReply : public DynamicWebReply
{
 public:
	DynamicFileWebReply(const std::string& filename,
	                    const std::string& content_type = "");
  DynamicFileWebReply(const char *filename);
  DynamicFileWebReply(FILE *file, bool close_when_done = true);
  virtual ~DynamicFileWebReply();

  virtual size_t size();
  virtual size_t next_chunk(size_t pos, char *buffer, size_t buf_max_size);

 private:
  void determine_file_size();

 private:
  FILE   *file_;
  size_t  size_;
  bool    close_when_done_;
};

} // end namespace fawkes

#endif
