
/***************************************************************************
 *  system.cpp - basic system exceptions
 *
 *  Generated: Sun Oct 29 14:28:17 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <core/exceptions/system.h>

/** @class OutOfMemoryException <core/exceptions/system.h>
 * System ran out of memory and desired operation could not be fulfilled.
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg optional extra message, appended to exception, base message "Out of memory"
 */
OutOfMemoryException::OutOfMemoryException(const char *msg)
  : Exception("Out of memory")
{
  append(msg);
}


/** @class InterruptedException <core/exceptions/system.h>
 * The current system call has been interrupted (for instance by a signal).
 * Throw this exception if you use libc functions which return EINTR or store
 * EINTR in errno.
 * @ingroup Exceptions
 */
/** Constructor */
InterruptedException::InterruptedException()
  : Exception("Interrupted system call")
{
}


/** @class CouldNotOpenFileException <core/exceptions/system.h>
 * File could not be opened.
 * The file could not be opened. Optional error number and message describe the
 * problem in more detai.
 * @ingroup Exceptions
 */

/** Constructor with error number.
 * @param filename name of file which could not be opened
 * @param errno error number
 * @param additional_msg optional additional message
 */
CouldNotOpenFileException::CouldNotOpenFileException(const char *filename, int errno,
						     const char *additional_msg)
  : Exception("Could not open file", errno)
{
  append("Failed to open file '%s'", filename);
  append(additional_msg);
}
/** Constructor with error number.
 * @param filename name of file which could not be opened
 * @param additional_msg optional additional message
 */
CouldNotOpenFileException::CouldNotOpenFileException(const char *filename,
						     const char *additional_msg)
  : Exception("Could not open file")
{
  append("Failed to open file '%s'", filename);
  append(additional_msg);
}



/** @class FileReadException <core/exceptions/system.h>
 * File could not be read.
 * The file could not be read. Optional error number and message describe the
 * problem in more detai.
 * @ingroup Exceptions
 */

/** Constructor with error number.
 * @param filename name of file which coul not be read
 * @param errno error number
 * @param additional_msg optional additional message
 */
FileReadException::FileReadException(const char *filename, int errno,
						     const char *additional_msg)
  : Exception("Could not read from file", errno)
{
  append("Failed to read from file '%s'", filename);
  append(additional_msg);
}
/** Constructor with error number.
 * @param filename name of file which could not be read
 * @param additional_msg optional additional message
 */
FileReadException::FileReadException(const char *filename,
						     const char *additional_msg)
  : Exception("Could read from file")
{
  append("Failed to read from file '%s'", filename);
  append(additional_msg);
}
