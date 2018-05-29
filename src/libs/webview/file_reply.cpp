
/***************************************************************************
 *  file_reply.cpp - Web request file reply
 *
 *  Created: Thu Oct 23 14:00:17 2008
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

#include <webview/file_reply.h>

#include <core/exceptions/system.h>
#include <utils/system/filetype.h>

#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class DynamicFileWebReply <webview/file_reply.h>
 * Dynamic raw file transfer reply.
 * This dynamic file transfer reply transmits the given file with a mime type
 * determined with libmagic.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename path and name of the file to transmit
 * @param content_type content type of file, will try to guess by
 * magic if not given
 */
DynamicFileWebReply::DynamicFileWebReply(const std::string& filename,
                                         const std::string& content_type)
	: DynamicWebReply(WebReply::HTTP_OK), close_when_done_(true)
{
	if (access(filename.c_str(), R_OK) != 0 ||
	    ((file_ = fopen(filename.c_str(), "r")) == NULL))
	{
    throw fawkes::CouldNotOpenFileException(filename.c_str(), errno);
  }

  determine_file_size();

  if (content_type.empty()) {
	  add_header("Content-type", fawkes::mimetype_file(filename.c_str()));
  } else {
	  add_header("Content-type", content_type);
  }
}

/** Constructor.
 * @param filename path and name of the file to transmit
 */
DynamicFileWebReply::DynamicFileWebReply(const char *filename)
  : DynamicWebReply(WebReply::HTTP_OK), close_when_done_(true)
{
  if (access(filename, R_OK) != 0 || ((file_ = fopen(filename, "r")) == NULL)) {
    throw fawkes::CouldNotOpenFileException(filename, errno);
  }

  determine_file_size();

  add_header("Content-type", fawkes::mimetype_file(filename));
}

/** Constructor.
 * @param file file handle of file to transmit
 * @param close_when_done true to close file after transmission is completed
 */
DynamicFileWebReply::DynamicFileWebReply(FILE *file, bool close_when_done)
  : DynamicWebReply(WebReply::HTTP_OK),
    file_(file), close_when_done_(close_when_done)
{
  fseek(file_, 0, SEEK_SET);
  determine_file_size();
  try {    
    add_header("Content-type", fawkes::mimetype_file(dup(fileno(file_))));
  } catch (Exception &e) {} // ignored
  fseek(file_, 0, SEEK_SET);
}

/** Destructor. */
DynamicFileWebReply::~DynamicFileWebReply()
{
  if (close_when_done_)  fclose(file_);
  file_ = NULL;
}


void
DynamicFileWebReply::determine_file_size()
{
  struct stat sbuf;
  fstat(fileno(file_), &sbuf);

  if ( S_ISDIR(sbuf.st_mode) ) {
    throw fawkes::Exception("Cannot send directory\n");
  }
  size_ = sbuf.st_size;
}

size_t
DynamicFileWebReply::size()
{
  return size_;
}

size_t
DynamicFileWebReply::next_chunk(size_t pos, char *buffer, size_t buf_max_size)
{
  if ( (fseek(file_, pos, SEEK_SET) == -1) || feof(file_) ) {
    return (size_t)-1;
  }
  return fread(buffer, 1, buf_max_size, file_);
}

} // end namespace fawkes
