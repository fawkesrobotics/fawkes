
/***************************************************************************
 *  file.h - file utils
 *
 *  Generated: Wed Aug 30 22:46:20 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *             2007  Daniel Beck 
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_SYSTEM_FILE_H_
#define __UTILS_SYSTEM_FILE_H_

#include <core/exception.h>

#include <stdio.h>

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

  File(char *filename, FileOpenMethod method = APPEND);
  ~File();

  FILE *stream() const;
  const char *filename() const;

  static bool exists(const char *filename);
  static bool isRegular(const char *filename);

 private:
  int fd;
  FILE *fp;
  char *fn;
};

#endif
