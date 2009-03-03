
/***************************************************************************
 *  software.h - basic software exceptions
 *
 *  Generated: Wed Oct 04 18:37:35 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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


class NullPointerException : public Exception {
 public:
  NullPointerException(const char *msg) throw();
};


class DivisionByZeroException : public Exception {
 public:
  DivisionByZeroException(const char *msg) throw();
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
  DestructionInProgressException(const char *msg) throw();
};


class NotLockedException : public Exception {
 public:
  NotLockedException(const char *msg) throw();
};


class NonPointerTypeExpectedException : public Exception {
 public:
  NonPointerTypeExpectedException(const char *msg) throw();
};


class MissingParameterException : public Exception {
 public:
  MissingParameterException(const char *msg) throw();
};


class IllegalArgumentException : public Exception {
 public:
  IllegalArgumentException(const char *msg) throw();
};


class OutOfBoundsException : public Exception {
 public:
  OutOfBoundsException(const char *msg) throw();
  OutOfBoundsException(const char *msg, float val,
		       float min, float max) throw();
};


class AccessViolationException : public Exception {
 public:
  AccessViolationException(const char *msg) throw();
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
