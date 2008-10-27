
/***************************************************************************
 *  file_reply.cpp - Web request file reply
 *
 *  Created: Thu Oct 23 14:00:17 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "file_reply.h"

#include <core/exceptions/system.h>
#include <utils/system/filetype.h>

#include <cerrno>
#include <sys/stat.h>

/** @class DynamicFileWebReply "file_reply.h"
 * Dynamic raw file transfer reply.
 * This dynamic file transfer reply transmits the given file with a mime type
 * determined with libmagic.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename path and name of the file to transmit
 */
DynamicFileWebReply::DynamicFileWebReply(const char *filename)
  : DynamicWebReply(WebReply::HTTP_OK)
{
  if (access(filename, R_OK) != 0 || ((__file = fopen(filename, "r")) == NULL)) {
    throw fawkes::CouldNotOpenFileException(filename, errno);
  }

  struct stat sbuf;
  fstat(fileno(__file), &sbuf);

  if ( S_ISDIR(sbuf.st_mode) ) {
    throw fawkes::Exception("Cannot send directory\n");
  }
  __size = sbuf.st_size;

  add_header("Content-type", fawkes::mimetype_file(filename));
}

/** Destructor. */
DynamicFileWebReply::~DynamicFileWebReply()
{
  fclose(__file);
}

size_t
DynamicFileWebReply::size()
{
  return __size;
}

size_t
DynamicFileWebReply::next_chunk(size_t pos, char *buffer, size_t buf_max_size)
{
  if ( (fseek(__file, pos, SEEK_SET) == -1) || feof(__file) ) {
    return -1;
  }
  return fread(buffer, 1, buf_max_size, __file);
}
