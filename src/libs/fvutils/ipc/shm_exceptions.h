
/***************************************************************************
 *  shm_exceptions.h - exceptions thrown in shmem image utils, do NOT put
 *                     your own application specific exceptions here!
 *
 *  Generated: Thu Feb 09 13:06:52 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_IPC_SHM_EXCEPTIONS_H_
#define __FIREVISION_FVUTILS_IPC_SHM_EXCEPTIONS_H_

#include <core/exception.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Throw if an inconsistent image was found. */
class InconsistentImageException : public fawkes::Exception {
 public:
  /** Constructor.
   * @param msg additional message
   */
  InconsistentImageException(const char *msg)
    : fawkes::Exception(msg)  {}
};


/** Throw if an inconsistent LUT was found. */
class InconsistentLUTException : public fawkes::Exception {
 public:
  /** Constructor.
   * @param msg additional message
   */
  InconsistentLUTException(const char *msg)
    : fawkes::Exception(msg)  {}
};

} // end namespace firevision

#endif
