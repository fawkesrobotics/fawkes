
/***************************************************************************
 *  file.cpp - file utils
 *
 *  Generated: Wed Aug 30 22:47:11 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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


#include <utils/system/file.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

/** @class File utils/system/file.h
 * File utility methods.
 */


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
File::isRegular(const char *filename)
{
  struct stat s;

  if ( stat(filename, &s) == 0 ) {
    return S_ISREG(s.st_mode);
  } else {
    return false;
  }
}
