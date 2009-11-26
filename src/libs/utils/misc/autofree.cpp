
/***************************************************************************
 *  autofree.cpp - Automatic Freeer
 *
 *  Created: Thu Nov 26 13:17:42 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <utils/misc/autofree.h>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MemAutoFree <utils/misc/autofree.h>
 * Automatically free memory on destruction.
 * This class can be used to free memory on destruction of the object.
 * This is similar to many use cases of std::auto_ptr, with the difference
 * that it calls free() to release the memory instead of delete, therefore
 * it is meant to be used with classical memory allocations, e.g. C strings.
 * In effect the instance of MemAutoFree takes ownership of the passed pointer.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param ptr pointer to delete on destruct
 */
MemAutoFree::MemAutoFree(void *ptr)
{
  __ptr = ptr;
}


/** Destructor.
 * Destroys the memory chunk unless it has been released before.
 */
MemAutoFree::~MemAutoFree()
{
  if (__ptr)  free(__ptr);
}


/** Release ownership.
 * The instance no longer owns the pointer and memory will not be deleted
 * on destruction.
 */
void
MemAutoFree::release()
{
  __ptr = NULL;
}


/** Reset pointer to a different one,
 * This will free the pointer hold up to this call and will replace it with
 * new_ptr. It is verified that the old and new pointers are different, nothing
 * will be done if they are the same.
 * @param new_ptr new pointer to own
 */
void
MemAutoFree::reset(void *new_ptr)
{
  if (__ptr != new_ptr) {
    if (__ptr)  free(__ptr);
    __ptr = new_ptr;
  }
}


} // end namespace fawkes
