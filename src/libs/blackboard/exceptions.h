
/***************************************************************************
 *  exceptions.h - BlackBoard exceptions
 *
 *  Generated: Wed Oct 04 18:37:50 2006
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

#ifndef __BLACKBOARD_EXCEPTIONS_H_
#define __BLACKBOARD_EXCEPTIONS_H_

#include <core/exception.h>

namespace fawkes {

/** A NULL pointer was supplied where not allowed.
 * Throw this exception if a pointer to NULL has been supplied where this is
 * not allowed.
 */
class BlackBoardMemMgrInvalidPointerException : public Exception {
 public:
  /** Constructor */
  BlackBoardMemMgrInvalidPointerException() : Exception("Invalid pointer to free") {}
};


/** Thrown when BlackBoard memory has been corupted
 * This exception is thrown by the memory manager if the memory has been
 * corrupted, for example if there are bytes that belong to neither a free chunk nor
 * a allocated chunk.
 */
class BBInconsistentMemoryException : public Exception {
 public:
  /** Constructor
   * @param msg message, appended to exception, base message "Memory corruption detected"
   */
  BBInconsistentMemoryException(const char *msg)
    : Exception("Memory corruption detected")
  {
    append(msg);
  }
};

/** Thrown if BlackBoard is not master and master operation has been requested.
 * This exception is thrown by the memory manager if the memory is not owned but
 * master mode is required. The interface manager throws it if you try to create
 * a new interface but are not master.
 * corrupted, for example if there are bytes that belong to neither a free chunk nor
 * a allocated chunk.
 */
class BBNotMasterException : public Exception {
 public:
  /** Constructor
   * @param msg message, appended to exception, base message "Memory corruption detected"
   */
  BBNotMasterException(const char *msg)
    : Exception("Not BlackBoard Master!")
  {
    append(msg);
  }
};


/** Thrown if shared memory could not be opened. Can happen only if opening the
 * segment as non-master.
 */
class BBMemMgrCannotOpenException : public Exception {
 public:
  /** Constructor */
  BBMemMgrCannotOpenException() : Exception("Cannot open shared memory segment") {}
};


/** Thrown if no definition of interface or interface generator found.
 */
class BlackBoardInterfaceNotFoundException : public Exception {
 public:
  /** Constructor
   * @param type type of interface that could not be found
   * @param add_msg additional message, add a space as first character
   */
  BlackBoardInterfaceNotFoundException(const char *type, const char *add_msg = "")
    : Exception("Interface of type '%s' not found.%s", type, add_msg)
  {
  }
};


/** Thrown if versions do not match.
 * If the hashes of two interfaces of the very same type do not match they
 * are incompatible.
 */
class BlackBoardInterfaceVersionMismatchException : public Exception {
 public:
  /** Constructor. */
  BlackBoardInterfaceVersionMismatchException()
    : Exception("Interface versions do not match, need to update and recompile interfaces?")
  {
  }
};


/** Thrown if a writer is already active on an interface that writing has
 * been requested for.
 */
class BlackBoardWriterActiveException : public Exception {
 public:
  /** Constructor
   * @param type type of interface that could not be found
   * @param id identifier of the interface
   */
  BlackBoardWriterActiveException(const char *id, const char *type) : Exception()
  {
    append("There is already a writer on interface '%s' of type '%s'", id, type);
  }
};


/** Thrown if BlackBoard is opened as non-master with no master alive.
 */
class BlackBoardNoMasterAliveException : public Exception {
 public:
  /** Constructor*/
  BlackBoardNoMasterAliveException() : Exception("No master BlackBoard alive") {}
};


/** Thrown if no writer interface is alive.
 */
class BlackBoardNoWritingInstanceException : public Exception
{
 public:
	/** Constructor.
	 * @param type interface type string
	 * @param id interface ID
	 */
	BlackBoardNoWritingInstanceException(const char *type, const char *id) : Exception()
	{
		append("No writing instance for interface %s::%s", type, id);
	}
};

} // end namespace fawkes

#endif
