
/***************************************************************************
 *  shm_exceptions.h - exceptions thrown in shmem utils, do NOT put your own
 *                     application specific exceptions here!
 *
 *  Created: Thu Feb 09 13:06:52 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_SHM_EXCEPTIONS_H_
#define __UTILS_IPC_SHM_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {


class ShmCouldNotAttachException : public Exception
{
 public:
  ShmCouldNotAttachException(const char *msg);
};


class ShmNoHeaderException : public Exception
{
 public:
  ShmNoHeaderException();
};


class ShmInconsistentSegmentSizeException : public Exception
{
 public:
  ShmInconsistentSegmentSizeException(unsigned int desired_mem, unsigned int act_mem);
};


class ShmDoesNotExistException : public Exception
{
 public:
  ShmDoesNotExistException();
};


class ShmCouldNotAttachAddrDepException : public Exception
{
 public:
  ShmCouldNotAttachAddrDepException();
};


class ShmAddrOutOfBoundsException : public Exception
{
 public:
  ShmAddrOutOfBoundsException();
};

class ShmPtrOutOfBoundsException : public Exception
{
 public:
  ShmPtrOutOfBoundsException();
};


} // end namespace fawkes

#endif
