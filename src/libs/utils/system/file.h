
/***************************************************************************
 *  file.h - file utils
 *
 *  Generated: Wed Aug 30 22:46:20 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *             2007  Daniel Beck 
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

#ifndef __UTILS_SYSTEM_FILE_H_
#define __UTILS_SYSTEM_FILE_H_

#include <core/exception.h>
#include <cstdio>

namespace fawkes {

class UnableToOpenFileException : public Exception {
 public:
  UnableToOpenFileException(const char *filename, int error);
};

class File {
 public:

  /** What to do when a file with the same name
   * already exists 
   */
  typedef enum {
    OVERWRITE,		/**< overwrite the existing file */
    APPEND,		/**< append data at the end of the existing file */
    ADD_SUFFIX		/**< add a suffix (starting with ".1") to the given filename */
  } FileOpenMethod;

  File(const char *filename, FileOpenMethod method = APPEND);
  ~File();

  FILE *        stream() const;
  const char *  filename() const;

  static bool   exists(const char *filename);
  static bool   is_regular(const char *filename);

 private:
  int fd;
  FILE *fp;
  char *fn;
};


} // end namespace fawkes

#endif
