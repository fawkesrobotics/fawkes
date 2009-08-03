
/***************************************************************************
 *  xabsl_tools.h - Tools required for XABSL
 *
 *  Created: Wed Aug 06 17:20:37 2008
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

#ifndef __PLUGINS_XABSL_XABSL_TOOLS_H_
#define __PLUGINS_XABSL_XABSL_TOOLS_H_

#include <XabslEngine/XabslTools.h>

#include <cstdio>

namespace fawkes {
  class Logger;
}

class XabslLoggingErrorHandler : public xabsl::ErrorHandler
{
 public:
  XabslLoggingErrorHandler(fawkes::Logger *logger);

  virtual void printError(const char *text);
  virtual void printMessage(const char *text);

 private:
  fawkes::Logger *__logger;
};


class XabslFileInputSource : public xabsl::InputSource
{
 public:
  XabslFileInputSource(const char* filename);
  ~XabslFileInputSource();

  virtual bool open();
  virtual void close();

  virtual double readValue() ;
  virtual bool   readString(char* destination, int maxLength);

 private:
  char read_and_omit_whitespace(bool omit_whitespace);
  bool read_from_file(char *buf, size_t buf_length);
  void omit_comment();

 private:
  char *__filename;
  FILE *__f;
};

#endif
