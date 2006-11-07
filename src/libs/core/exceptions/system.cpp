
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

/** @class OutOfMemoryException core/exceptions/system.h
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


/** @class InterruptedException core/exceptions/system.h
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
