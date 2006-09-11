
/***************************************************************************
 *  file.h - file utils
 *
 *  Generated: Wed Aug 30 22:46:20 2006
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

#ifndef __UTILS_SYSTEM_FILE_H_
#define __UTILS_SYSTEM_FILE_H_

/** File utility methods
 */
class File {

 public:

  /** Check if a file exists.
   * @param filename the name of the file to check
   * @return true, if the file exists, false otherwise
   */
  static bool exists(const char *filename);

  /** Check if a file is a regular file
   * @param filename the name of the file to check
   * @return true, if the given path points to a regular file, false otherwise
   */
  static bool isRegular(const char *filename);

};

#endif
