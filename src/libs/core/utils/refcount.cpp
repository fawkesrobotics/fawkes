
/***************************************************************************
 *  refcount.cpp - reference counting base class
 *
 *  Created: Fri Oct 27 13:52:08 2006
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

#include <core/utils/refcount.h>
#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <unistd.h>

namespace fawkes {

/** @class RefCount core/utils/refcount.h
 * Reference counting base class.
 * Derive this class with your object if you need reference counting for the object
 * thus that it is not deleted while some code is still using an class instance.
 *
 * The RefCount class is NOT meant for direct usage. In most cases if you are aware
 * of the need of reference couting during the design of the software derive this
 * class. This is the recommended way. If you want to use reference counting with
 * a class that you cannot or do not want to modify you can use the RefCounter
 * template class to accomplish the desired task.
 * @see RefCounter
 *
 * @ingroup FCL
 * @author Tim Niemueller
 */


/** Constructor. */
RefCount::RefCount()
{
  ref_mutex = new Mutex();
  refc = 1;
}


/** Destructor. */
RefCount::~RefCount()
{
  delete ref_mutex;
}


/** Increment reference count.
 * @exception DestructionInProgressException Thrown if the only other reference holder
 * for this instance was just deleting the instance when you tried to reference it.
 * @see recount();
 */
void
RefCount::ref()
{
  ref_mutex->lock();
  if ( refc == 0 ) {
    throw DestructionInProgressException("Tried to reference that is currently being deleted");
  }
  ++refc;
  ref_mutex->unlock();
}


/** Decrement reference count and conditionally delete this instance.
 * This method will decrement the reference count of this message. If the reference count
 * reaches zero the message will be deleted automatically. So it is not safe to use this
 * instance after is has been unref()'ed.
 * For the code calling
 * @code
 * obj->unref();
 * @endcode
 * should be considered equivalent to
 * @code
 * delete obj;
 * @endcode.
 * There is no guarantee whatsover that the object can still be used afterwards.
 * It is however guaranteed, that the instance is not deleted/free from memory if
 * there are still other applications using this instance that properly ref()'ed
 * this instance before conditional delete was called.
 */
void
RefCount::unref()
{
  
  ref_mutex->lock();
  if ( refc == 0 ) {
    throw DestructionInProgressException("Tried to reference that is currently being deleted");
  }
  if ( refc > 0 )  --refc;
  if ( refc == 0 ) {
    // commit suicide
    delete this;
    return;
  }
  ref_mutex->unlock();
}


/** Get reference count for this instance.
 * The reference count is used to determine if a message should really be destructed
 * or not.
 * Do not rely on this value, as race-conditions may ruin your code! Do only use the
 * atomic ref() and unref() operations. This function is only provided to output
 * informational debugging output!
 * @return reference count
 */
unsigned int
RefCount::refcount()
{
  return refc;
}


} // end namespace fawkes
