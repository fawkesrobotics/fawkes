
/***************************************************************************
 *  refcounter.h - reference counting template class
 *
 *  Created: Fri Oct 27 13:59:03 2006
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

#ifndef __CORE_UTILS_REFCOUNTER_H_
#define __CORE_UTILS_REFCOUNTER_H_

#include <core/utils/refcount.h>
#include <core/exceptions/software.h>
#include <typeinfo>

template <class Type>
class RefCounter : public RefCount
{
 public:
  RefCounter(const Type &p);
  RefCounter(Type *p);
  RefCounter();
  ~RefCounter();

  Type &        operator *  () const;
  Type *        operator -> () const;

 private:
  Type *p;

};

/** @class RefCounter core/utils/refcounter.h
 * Template class for reference counting.
 * If you cannot or do not want to modify the class that needs reference counting
 * from RefCount you can use the RefCounter template class to add this functionality.
 * The RefCounter has two constructors. The one will copy the given instance (or copy
 * the pointer if it is a pointer type) and the other one will create a new instance
 * of the given type.
 *
 * Important: Use the non-pointer type of the class as argument. Otherwise a runtime
 * exception will be thrown when the constructor is run.
 *
 * Note that that is one glitch in this class which became necessary:
 * One thing we want is that we do not have to care about the object being freed when
 * is is unref()'ed last. This also implies that we want to free the memory of the
 * RefCounter class itself, not just of the protected pointer. So in fact this class
 * has to commit suicide if the time has come. This will miserably result in a horrible
 * desaster if you did not instantiate the class with the new operator on the heap but
 * just have a non-pointer member or scope variable of the RefCounter. In that case the
 * last unref() will free memory your program uses and will crash, sooner or later.
 * So be warned and always use pointer to the RefCounter. Use code like the following
 * to use the refcounter:
 * @code
 * RefCounter<ClassTypeToProtect *> *rc = new RefCounter<ClassTypeToProtect *>();
 * // do something with the class
 * rc->do_something();
 * // others may have referenced the var by now, but we don't need it anymore
 * give_rc_to_other(rc);
 * rc->unref();
 * // rc may not be used from here on!
 * @endcode
 * The mentioned function may look like this:
 * @code
 * void
 * give_rc_to_other(RefCounter<ClassTypeToProtect *> *rc)
 * {
 *   rc->ref();
 *   // do something with data...
 *   rc->unref();
 * }
 * @endcode
 *
 * Note that you must give the non-pointer typeof the class, it will be transformed to
 * an pointer type internally.
 *
 * @see RefCount
 * @ingroup FCL
 *
 * @author Tim Niemueller
 */

/** Constructor that copies the given protected element.
 * The reference count is 1 after using the constructor. A copy of the given instance
 * will be created.
 * @param p instance to protect
 */
template <class Type>
RefCounter<Type>::RefCounter(const Type &p)
{
  if (typeid(Type).__is_pointer_p()) {
    // this should merely be a compile time check...
    throw NonPointerTypeExpectedException("RefCounter<Type>::RefCounter(const Type &p)");
  }
  this->p = new Type(p);
}


/** Constructor that copies the given protected element.
 * The reference count is 1 after using the constructor. The RefCounter takes over
 * ownership of the instance.
 * @param p instance to protect
 */
template <class Type>
RefCounter<Type>::RefCounter(Type *p)
{
  if (typeid(Type).__is_pointer_p()) {
    // this should merely be a compile time check...
    throw NonPointerTypeExpectedException("RefCounter<Type>::RefCounter(Type *p)");
  }
  this->p = p;
}


/** Constructor that creates a new instance of given type. */
template <class Type>
RefCounter<Type>::RefCounter()
{
  if (typeid(Type).__is_pointer_p()) {
    // this should merely be a compile time check...
    throw NonPointerTypeExpectedException("RefCounter<Type>::RefCounter()");
  }
  p = new Type();
}


/** Destructor. */
template <class Type>
RefCounter<Type>::~RefCounter()
{
  delete p;
}


/** Get reference protected data.
 * Returns a reference to the internal protected instance.
 * @return pointer to instance.
 */
template <class Type>
Type &
RefCounter<Type>::operator * () const
{
  return const_cast<Type &>(*p);
}


/** Get reference protected data.
 * This is the same operation as the * operation, but if you use a pointer type
 * as template argument the code that results from usage of the -> operator is easier
 * to read.
 * @return pointer to instance.
 * @see operator*()
 */
template <class Type>
Type *
RefCounter<Type>::operator -> () const
{
  return const_cast<Type *>(p);
}


#endif
