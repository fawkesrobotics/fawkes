
/***************************************************************************
 *  xabsl_tools.cpp - Tools required for XABSL
 *
 *  Created: Wed Aug 06 17:25:51 2008
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

#include "xabsl_tools.h"

#include <core/exception.h>
#include <utils/logging/logger.h>

#include <cstdlib>
#include <cstring>

/** @class XabslLoggingErrorHandler "xabsl_tools.h"
 * Logging error handler for XABSL integration.
 * Simple error handler that prints errors to the Fawkes log.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger Fawkes logger
 */
XabslLoggingErrorHandler::XabslLoggingErrorHandler(fawkes::Logger *logger)
{
  __logger = logger;
}


/** Print error message.
 * @param text text of the error message
 */
void
XabslLoggingErrorHandler::printError(const char *text)
{
  __logger->log_error("XABSL", "%s", text);
}


/** Print info message.
 * @param text text of the info message
 */
void
XabslLoggingErrorHandler::printMessage(const char *text)
{
  __logger->log_info("XABSL", "%s", text);
}


/** @class XabslFileInputSource "xabsl_tools.h"
 * File input class for Xabsl integration.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param filename name of the file to read
 */
XabslFileInputSource::XabslFileInputSource(const char *filename)
{
  __filename = strdup(filename);
  __f = NULL;
}


/** Destructor. */
XabslFileInputSource::~XabslFileInputSource()
{
  close();
  free(__filename);
}


/** Open file.
 * @return true if file has been opened successfully, false otherwise
 */
bool
XabslFileInputSource::open()
{
  close();
  __f = fopen(__filename, "r");
  return (__f != NULL);
}


/** Close file. */
void
XabslFileInputSource::close()
{
  if ( __f )  fclose(__f);
  __f = NULL;
}


/** Read a double value from the file.
 * @return value read from the file
 */
double
XabslFileInputSource::readValue()
{
  char buf[20];
  if (read_from_file(buf, sizeof(buf)-1)) {
    return atof(buf);
  } else {
    return 0.;
  }
}


/** Read a string from the file.
 * @param buf buffer where the string is stored
 * @param buf_length maximum length of the string to be read, warning, this
 * method will write one more byte than buf_length. This is done to be compatible
 * with broken Xabsl.
 * @return true on success, false otherwise
 */
bool
XabslFileInputSource::readString(char *buf, int buf_length)
{
  return read_from_file(buf, buf_length);
}


/** Omit comments. */
void
XabslFileInputSource::omit_comment()
{
  while ( !feof(__f) ) {
    char c;
    if (fread(&c, 1, 1, __f)) {
      if ( c == '\n')  return;
    } else {
      return;
    }
  }
}

/** Read and possibly omit whitespace.
 * @param omit_whitespace if true whitespace is omitted
 * @return first char read or 0 on error
 */
char
XabslFileInputSource::read_and_omit_whitespace(bool omit_whitespace)
{
  while ( ! feof(__f) ) {
    char c;
    if (fread(&c, 1, 1, __f)) {
      if ( c == '/' ) {
	omit_comment();
	continue;
      }
      if ( (c != ' ') && (c != '\n') && (c != '\r') && (c != '\t') ) {
	return c;
      } else if ( ! omit_whitespace ) {
	return 0;
      }
    } else {
      throw fawkes::Exception ("XabslFileInputSource: omit_whitespace() fread failed");
    }
  }

  return 0;
}

/** Read bytes from file.
 * @param buf buffer where the string is stored
 * @param buf_length maximum length of the string to be read, warning, this
 * method will write one more byte than buf_length. This is done to be compatible
 * with broken Xabsl.
 * @return true if anything was read from the file, false if nothing has been read
 */
bool
XabslFileInputSource::read_from_file(char *buf, size_t buf_length)
{
  if ( ! __f || feof(__f) )  return false;

  memset(buf, 0, buf_length);
  size_t cur_length = 0;
  bool is_first = true;
  while (! feof(__f) && (cur_length < buf_length)) {
    char c = read_and_omit_whitespace(is_first);
    is_first = false;
    if (c) {
      buf[cur_length++] = c;
      buf[cur_length] = 0;
    } else {
      return (cur_length > 0);
    }
  }

  return (cur_length > 0);
}
