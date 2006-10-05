
/***************************************************************************
 *  software.h - basic software exceptions
 *
 *  Generated: Wed Oct 04 18:37:35 2006
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

#ifndef __CORE_EXCEPTIONS_SOFTWARE_H_
#define __CORE_EXCEPTIONS_SOFTWARE_H_

#include <core/exception.h>

/** A NULL pointer was supplied where not allowed.
 * Throw this exception if a pointer to NULL has been supplied where this is
 * not allowed.
 */
class NullPointerException : public Exception {
 public:
  /** Constructor
   * @param msg message, appended to exception, base message "NullPointerException"
   */
  NullPointerException(const char *msg) : Exception("NullPointerException")
  {
    append(msg);
  }
};

/** Division by zero
 * Throw this if a division by zero has happened or is about to happen
 */
class DivisionByZeroException : public Exception {
 public:
  /** Constructor
   * @param msg message, appended to exception, base message "Division by zero"
   */
  DivisionByZeroException(const char *msg) : Exception("Division by zero")
  {
    append(msg);
  }
};

#endif
