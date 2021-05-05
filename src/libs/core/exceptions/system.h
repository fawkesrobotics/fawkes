
/***************************************************************************
 *  system.h - basic system exceptions
 *
 *  Generated: Mon Sep 18 19:22:36 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef _CORE_EXCEPTIONS_SYSTEM_H_
#define _CORE_EXCEPTIONS_SYSTEM_H_

#include <core/exception.h>

namespace fawkes {

class OutOfMemoryException : public Exception
{
public:
	OutOfMemoryException(const char *format, ...) noexcept;
	OutOfMemoryException() noexcept;
};

class InterruptedException : public Exception
{
public:
	InterruptedException() noexcept;
	InterruptedException(const char *format, ...) noexcept;
};

class TimeoutException : public Exception
{
public:
	TimeoutException() noexcept;
	TimeoutException(const char *format, ...) noexcept;
};

class CouldNotOpenFileException : public Exception
{
public:
	CouldNotOpenFileException(const char *filename,
	                          int         errnum,
	                          const char *additional_msg = 0) noexcept;
	CouldNotOpenFileException(const char *filename, const char *additional_msg = 0) noexcept;
};

class FileReadException : public Exception
{
public:
	FileReadException(const char *filename, int errnum, const char *additional_msg = 0) noexcept;
	FileReadException(const char *filename, const char *additional_msg = 0) noexcept;
};

class FileWriteException : public Exception
{
public:
	FileWriteException(const char *filename, int errnum, const char *additional_msg = 0) noexcept;
	FileWriteException(const char *filename, const char *additional_msg = 0) noexcept;
};

} // end namespace fawkes

#endif
