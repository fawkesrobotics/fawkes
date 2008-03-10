
/***************************************************************************
 *  system.h - basic system exceptions
 *
 *  Generated: Mon Sep 18 19:22:36 2006
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

#ifndef __CORE_EXCEPTIONS_SYSTEM_H_
#define __CORE_EXCEPTIONS_SYSTEM_H_

#include <core/exception.h>

class OutOfMemoryException : public Exception {
 public:
  OutOfMemoryException(const char *msg = "") throw();
};


class InterruptedException : public Exception {
 public:
  InterruptedException() throw();
};


class CouldNotOpenFileException : public Exception {
 public:
  CouldNotOpenFileException(const char *filename, int errno,
			    const char *additional_msg = 0) throw();
  CouldNotOpenFileException(const char *filename, const char *additional_msg = 0) throw();
};


class FileReadException : public Exception {
 public:
  FileReadException(const char *filename, int errno,
		    const char *additional_msg = 0) throw();
  FileReadException(const char *filename, const char *additional_msg = 0) throw();
};

class FileWriteException : public Exception {
 public:
  FileWriteException(const char *filename, int errno,
		     const char *additional_msg = 0) throw();
  FileWriteException(const char *filename, const char *additional_msg = 0) throw();
};


#endif
