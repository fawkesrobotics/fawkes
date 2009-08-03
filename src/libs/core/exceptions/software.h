
/***************************************************************************
 *  software.h - basic software exceptions
 *
 *  Created: Wed Oct 04 18:37:35 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_EXCEPTIONS_SOFTWARE_H_
#define __CORE_EXCEPTIONS_SOFTWARE_H_

#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NullPointerException : public Exception {
 public:
  NullPointerException(const char *format, ...) throw();
};


class DivisionByZeroException : public Exception {
 public:
  DivisionByZeroException(const char *format, ...) throw();
};


class TypeMismatchException : public Exception {
 public:
  TypeMismatchException(const char *format, ...) throw();
};


class UnknownTypeException : public Exception {
 public:
  UnknownTypeException(const char *format, ...) throw();
};


class DestructionInProgressException : public Exception {
 public:
  DestructionInProgressException(const char *format, ...) throw();
};


class NotLockedException : public Exception {
 public:
  NotLockedException(const char *format, ...) throw();
};


class NonPointerTypeExpectedException : public Exception {
 public:
  NonPointerTypeExpectedException(const char *format, ...) throw();
};


class MissingParameterException : public Exception {
 public:
  MissingParameterException(const char *format, ...) throw();
};


class IllegalArgumentException : public Exception {
 public:
  IllegalArgumentException(const char *format, ...) throw();
};


class OutOfBoundsException : public Exception {
 public:
  OutOfBoundsException(const char *msg) throw();
  OutOfBoundsException(const char *msg, float val,
		       float min, float max) throw();
};


class AccessViolationException : public Exception {
 public:
  AccessViolationException(const char *format, ...) throw();
};


class SyntaxErrorException : public Exception {
 public:
  SyntaxErrorException(const char *format, ...) throw();
};

class NotImplementedException : public Exception {
 public:
  NotImplementedException(const char *format, ...) throw();
};


} // end namespace fawkes

#endif
