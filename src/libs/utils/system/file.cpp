
/***************************************************************************
 *  file.cpp - file utils
 *
 *  Generated: Wed Aug 30 22:47:11 2006
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


#include <utils/system/file.h>
#include <core/exceptions/system.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <cstdio>

namespace fawkes {

/** @class UnableToOpenFileException file.h <utils/system/file.h>
 * Opening a file failed for some reason.
 * @ingroup Exceptions
 */
/** Constructor
 * @param filename the name of the file which couldn't be opened
 * @param error the errno
 */
UnableToOpenFileException::UnableToOpenFileException(const char *filename, int error)
  : Exception(error, "Unable to open file")
{
  append("File that could not be opened: %s", filename);
}


/** @class File file.h <utils/system/file.h>
 * File utility methods.
 * Allows for opening a file and provides utilities to check if a file exists
 * or whether it is a regular file (and not a symbolic link/directory).
 * @author Tim Niemueller
 * @author Daniel Beck
 */


/** Constructor. 
 * Independent of the FileOpenMethod files are created with 
 * permissions 660
 * @param filename the filename
 * @param method the method determines what is done if a file with the
 * specified name already exists
 */
File::File(const char *filename, FileOpenMethod method)
{
  fd = -1;

  switch (method)
    {
    case OVERWRITE:
      fd = open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
      fn = strdup(filename);
      break;
      
    case APPEND:
      fd = open(filename, O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
      fn = strdup(filename);
      break;
      
    case ADD_SUFFIX:
      {
	char *filename_ext = strdup(filename);
	int index = 0;
	while (File::exists(filename_ext)) {
	  free(filename_ext);
	  if ( asprintf(&filename_ext, "%s.%d", filename, ++index) == -1 ) {
	    throw OutOfMemoryException("Could not allocate filename string");
	  }
   
	}
	fd = open(filename_ext, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	fn = filename_ext;
      }
      break;
      
      default:
	printf("%s [line %d]: Unkown method.\n", __FILE__, __LINE__);
    }

  if (-1 == fd)
    {
      throw UnableToOpenFileException(filename, errno);
    }
  
  fp = fdopen(fd, "r+");
}


/** Destructor. */
File::~File()
{
  // this also closes the underlying file descritptor fd
  fclose(fp);
  free(fn);
}


/** Get access to the file stream.
 * @return a pointer to the file stream
 */
FILE *
File::stream() const
{
  return fp;
}

/** Get the file's name.
 * @return a pointer to a char array where the filename is stored
 */
const char *
File::filename() const
{
  return fn;
}


/** Check if a file exists.
 * @param filename the name of the file to check
 * @return true, if the file exists, false otherwise
 */
bool
File::exists(const char *filename)
{
  return (access(filename, F_OK) == 0);
}


/** Check if a file is a regular file
 * @param filename the name of the file to check
 * @return true, if the given path points to a regular file, false otherwise
 */
bool
File::is_regular(const char *filename)
{
  struct stat s;

  if ( stat(filename, &s) == 0 ) {
    return S_ISREG(s.st_mode);
  } else {
    return false;
  }
}


} // end namespace fawkes
