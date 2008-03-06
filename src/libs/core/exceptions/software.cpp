
/***************************************************************************
 *  software.cpp - basic software exceptions
 *
 *  Generated: Sun Oct 29 14:19:19 2006
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

#include <core/exceptions/software.h>

#include <cmath>

/** @class NullPointerException <core/exceptions/software.h>
 * A NULL pointer was supplied where not allowed.
 * Throw this exception if a pointer to NULL has been supplied where this is
 * not allowed.
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg message, appended to exception, base message "NullPointerException"
 */
NullPointerException::NullPointerException(const char *msg) throw()
  : Exception("NullPointerException: %s", msg)
{
}


/** @class DivisionByZeroException <core/exceptions/software.h>
 * Division by zero.
 * Throw this if a division by zero has happened or is about to happen
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg message, appended to exception, base message "Division by zero"
 */
DivisionByZeroException::DivisionByZeroException(const char *msg) throw()
  : Exception("Division by zero: %s", msg)
{
}


/** @class TypeMismatchException <core/exceptions/software.h>
 * Type mismatch.
 * Throw this exception if types of operations do not fit together.
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format
 */
TypeMismatchException::TypeMismatchException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}


/** @class UnknownTypeException <core/exceptions/software.h>
 * Unknown type.
 * Throw this exception if you get an unknown type.
 * @ingroup Exceptions
 */
/** Constructor
 * @param format message format
 */
UnknownTypeException::UnknownTypeException(const char *format, ...) throw()
  : Exception()
{
  va_list va;
  va_start(va, format);
  append_va(format, va);
  va_end(va);
}


/** @class DestructionInProgressException <core/exceptions/software.h>
 * Delete in progress.
 * Throw this exception if someone tried to access an object that is currently being
 * destroyed.
 * @ingroup Exceptions
 */
/** Constructor
 * @param msg informative message, appended to exception, base message is
 * "Destruction in progress"
 */
DestructionInProgressException::DestructionInProgressException(const char *msg) throw()
  : Exception("Destruction in progress: %s", msg)
{
}


/** @class NotLockedException <core/exceptions/software.h>
 * Operation on unlocked object.
 * Throw this exception if someone tried to operate on an object with a method that needs
 * outside locking. This can be detected utilizing Mutex::tryLock() in many situations.
 * @ingroup Exceptions
 */
/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Unsafe operation on not locked object"
 */
NotLockedException::NotLockedException(const char *msg) throw()
  : Exception("Unsafe operation on not locked object: %s", msg)
{
}


/** @class NonPointerTypeExpectedException <core/exceptions/software.h>
 * Non-pointer type expected.
 * Throw this exception if you got a pointer type where you expected to get a non-pointer
 * type variable.
 * @ingroup Exceptions
 */
/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Non-pointer type expected (template error?)"
 */
NonPointerTypeExpectedException::NonPointerTypeExpectedException(const char *msg) throw()
  :  Exception("Non-pointer type expected (template error?)")
{
  append(msg);
}


/** @class MissingParameterException <core/exceptions/software.h>
 * Expected parameter is missing.
 * Throw this exception if you expected one or more parameters that have not been
 * supplied.
 * @ingroup Exceptions
 */
/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Parameter is missing"
 */
MissingParameterException::MissingParameterException(const char *msg) throw()
  :  Exception("Parameter is missing")
{
  append(msg);
}


/** @class IllegalArgumentException <core/exceptions/software.h>
 * Expected parameter is missing.
 * Throw this exception if you got a parameter that does not meet some kind of
 * specification, i.e. it is of the wrong type or out of an allowed value range.
 * @ingroup Exceptions
 */
/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Illegal Argument"
 */
IllegalArgumentException::IllegalArgumentException(const char *msg) throw()
  :  Exception("Illegal Argument: %s", msg)
{
}


/** @class OutOfBoundsException >core/exceptions/software.h>
 * Index out of bounds.
 * Throw this exception if a value is out of bounds or if someone tries to access
 * an iterator that is not in the allowed range.
 * @ingroup Exceptions
 */

/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Out Of Bounds"
 */
OutOfBoundsException::OutOfBoundsException(const char *msg) throw()
  : Exception()
{
  append("Out Of Bounds: %s", msg);
}

/** Range constructor.
 * Additionally to the message the ranges and actual values are added to the
 * primary message.
 * @param msg informative message
 * @param val actual value
 * @param min minimum required value
 * @param max maximum allowed value
 */
OutOfBoundsException::OutOfBoundsException(const char *msg, float val,
					   float min, float max) throw()
  : Exception()
{
  if ( (roundf(val) == val) && (roundf(min) == min) && (roundf(max) == max) ) {
    // really the values are just integers
    append("Out Of Bounds (%s):  min: %.0f  max: %.0f  val: %.0f", msg, min, max, val);
  } else {
    // at least one "real" float
    append("Out Of Bounds (%s):  min: %f  max: %f  val: %f", msg, min, max, val);
  }
}


/** @class AccessViolationException <core/exceptions/software.h>
 * Access violates policy.
 * Throw this exception if a any kind of access violates the policy, for example
 * if someone tries to write to a read-only memory segment.
 * @ingroup Exceptions
 */
/** Constructor.
 * @param msg informative message, appended to exception, base message is
 * "Access Violation"
 */
AccessViolationException::AccessViolationException(const char *msg) throw()
  :  Exception("Access Violation: %s", msg)
{
}
