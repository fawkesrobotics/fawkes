
/***************************************************************************
 *  refptr.h - reference counting shared smartpointer
 *
 *  Created: Sat Jan 24 12:29:41 2009
 *  Copyright  2002  The gtkmm Development Team
 *             2005  The cairomm Development Team
 *             2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_UTILS_REFPTR_H_
#define __CORE_UTILS_REFPTR_H_

#include <core/threading/mutex.h>

namespace fawkes {

/** RefPtr<> is a reference-counting shared smartpointer.
 *
 * Reference counting means that a shared reference count is incremented each
 * time a RefPtr is copied, and decremented each time a RefPtr is destroyed,
 * for instance when it leaves its scope. When the reference count reaches
 * zero, the contained object is deleted
 *
 * Fawkes uses RefPtr so that you don't need to remember
 * to delete the object explicitly, or know when a method expects you to delete 
 * the object that it returns.
 *
 * Note that RefPtr is thread-safe.
 *
 * @ingroup FCL
 */
template <class T_CppObject>
class RefPtr
{
 public:
  /** Default constructor
   *
   * Afterwards it will be null and use of -> will cause a segmentation fault.
   */
  inline RefPtr();
  
  /// Destructor - decrements reference count.
  inline ~RefPtr();

  /** Constructor that takes ownership.
   *
   * This takes ownership of @a cpp_object, so it will be deleted when the 
   * last RefPtr is deleted, for instance when it goes out of scope.
   * @param cpp_object C++ object to take ownership of
   */
  explicit inline RefPtr(T_CppObject* cpp_object);

  /** Copy constructor
   * This increments the shared reference count.
   * @param src refptr to copy
   */
  inline RefPtr(const RefPtr<T_CppObject>& src);

  /** Copy constructor (from different, but castable type).
   * Increments the reference count.
   * @param src refptr to copy
   */
  template <class T_CastFrom>
  inline RefPtr(const RefPtr<T_CastFrom>& src);

  /** Swap the contents of two RefPtr<>.
   * This method swaps the internal pointers to T_CppObject.  This can be
   * done safely without involving a reference/unreference cycle and is
   * therefore highly efficient.
   * @param other other instance to swap with.
   */
  inline void swap(RefPtr<T_CppObject>& other);

  /** Copy from another RefPtr.
   * @param src refptr to copy from
   * @return reference to this instance
   */
  inline RefPtr<T_CppObject>& operator=(const RefPtr<T_CppObject>& src);

  /** Copy from different, but castable type).
   * Increments the reference count.
   * @param src refptr to copy from
   * @return reference to this instance
   */
  template <class T_CastFrom>
  inline RefPtr<T_CppObject>& operator=(const RefPtr<T_CastFrom>& src);

  /** Assign object and claim ownership.
   * @param ptr pointer to object, this refptr will claim ownership of the src!
   * @return reference to this instance
   */
  inline RefPtr<T_CppObject>& operator=(T_CppObject *ptr);


  /** Tests whether the RefPtr<> point to the same underlying instance.
   * @param src refptr to compare to
   * @return true if both refptrs point to the same instance.
   */
  inline bool operator==(const RefPtr<T_CppObject>& src) const;
  
  /** Tests whether the RefPtr<> do not point to the same underlying instance.
   * @param src refptr to compare to
   * @return true if both refptrs do not point to the same instance.
   */
  inline bool operator!=(const RefPtr<T_CppObject>& src) const;

  /** Dereferencing.
   * Use the methods of the underlying instance like so:
   * <code>refptr->memberfun()</code>.
   * @return pointer to encapsulated object
   */
  inline T_CppObject* operator->() const;

  /** Get underlying pointer.
   * Use with care!
   * @return pointer to encapsulated object
   */
  inline T_CppObject* operator*() const;


  /** Test whether the RefPtr<> points to any underlying instance.
   *
   * Mimics usage of ordinary pointers:
   * @code
   *   if (ptr)
   *     do_something();
   * @endcode
   */
  inline operator bool() const;

  /// Set underlying instance to 0, decrementing reference count of existing instance appropriately.
  inline void clear();

  /** Reset pointer.
   * Set underlying instance to 0, decrementing reference count of
   * existing instance appropriately.
   */
  inline void reset();


  /** Dynamic cast to derived class.
   *
   * The RefPtr can't be cast with the usual notation so instead you can use
   * @code
   *   ptr_derived = RefPtr<Derived>::cast_dynamic(ptr_base);
   * @endcode
   * @param src source refptr to cast
   * @return refptr to object casted to given type
   */
  template <class T_CastFrom>
  static inline RefPtr<T_CppObject> cast_dynamic(const RefPtr<T_CastFrom>& src);

  /** Static cast to derived class.
   *
   * Like the dynamic cast; the notation is 
   * @code
   *   ptr_derived = RefPtr<Derived>::cast_static(ptr_base);
   * @endcode
   * @param src source refptr to cast
   * @return refptr to object casted to given type
   */
  template <class T_CastFrom>
  static inline RefPtr<T_CppObject> cast_static(const RefPtr<T_CastFrom>& src);

  /** Cast to non-const.
   *
   * The RefPtr can't be cast with the usual notation so instead you can use
   * @code
   *   ptr_unconst = RefPtr<UnConstType>::cast_const(ptr_const);
   * @endcode
   * @param src source refptr to cast
   * @return refptr to object casted to given type
   */
  template <class T_CastFrom>
  static inline RefPtr<T_CppObject> cast_const(const RefPtr<T_CastFrom>& src);

  /** For use only in the internal implementation of sharedptr.
   * @param cpp_object C++ object to wrap
   * @param refcount reference count
   * @param refmutex reference count mutex
   */
  explicit inline RefPtr(T_CppObject *cpp_object, int *refcount, Mutex *refmutex);

  /** For use only in the internal implementation of sharedptr.
   * Get reference count pointer.
   * Warning: This is for internal use only.  Do not manually modify the
   * reference count with this pointer.
   * @return pointer to refcount integer
   */
  inline int *  refcount_ptr() const { return __ref_count; }

  /** Get current reference count.
   * @return current number of owners referencing this RefPtr.
   */
  inline int use_count() const { return *__ref_count; }

  /** For use only in the internal implementation of sharedptr.
   * Get reference mutex.
   * @return pointer to refcount mutex
   */
  inline Mutex *  refmutex_ptr() const { return __ref_mutex; }

private:

  T_CppObject   *__cpp_object;
  mutable int   *__ref_count;
  mutable Mutex *__ref_mutex;

};


// RefPtr<>::operator->() comes first here since it's used by other methods.
// If it would come after them it wouldn't be inlined.

template <class T_CppObject> inline
T_CppObject* RefPtr<T_CppObject>::operator->() const
{
  return __cpp_object;
}


template <class T_CppObject> inline
T_CppObject* RefPtr<T_CppObject>::operator*() const
{
  return __cpp_object;
}

template <class T_CppObject> inline
RefPtr<T_CppObject>::RefPtr()
:
  __cpp_object(0),
  __ref_count(0),
  __ref_mutex(0)
{}

template <class T_CppObject> inline
RefPtr<T_CppObject>::~RefPtr()
{
  if(__ref_count && __ref_mutex)
  {
    __ref_mutex->lock();

    --(*__ref_count);

    if(*__ref_count == 0)
    {
      if(__cpp_object)
      {
        delete __cpp_object;
        __cpp_object = 0;
      }

      delete __ref_count;
      delete __ref_mutex;
      __ref_count = 0;
      __ref_mutex = 0;
    } else {
      __ref_mutex->unlock();
    }
  }
}


template <class T_CppObject> inline
RefPtr<T_CppObject>::RefPtr(T_CppObject* cpp_object)
:
  __cpp_object(cpp_object),
  __ref_count(0),
  __ref_mutex(0)
{
  if(cpp_object)
  {
    __ref_count = new int;
    __ref_mutex = new Mutex();
    *__ref_count = 1; //This will be decremented in the destructor.
  }
}

//Used by cast_*() implementations:
template <class T_CppObject> inline
  RefPtr<T_CppObject>::RefPtr(T_CppObject* cpp_object, int* refcount, Mutex *refmutex)
:
  __cpp_object(cpp_object),
  __ref_count(refcount),
  __ref_mutex(refmutex)
{
  if(__cpp_object && __ref_count && __ref_mutex) {
    __ref_mutex->lock();
    ++(*__ref_count);
    __ref_mutex->unlock();
  }
}

template <class T_CppObject> inline
RefPtr<T_CppObject>::RefPtr(const RefPtr<T_CppObject>& src)
:
  __cpp_object (src.__cpp_object),
  __ref_count(src.__ref_count),
  __ref_mutex(src.__ref_mutex)
{
  if(__cpp_object && __ref_count && __ref_mutex)
  {
    __ref_mutex->lock();
    ++(*__ref_count);
    __ref_mutex->unlock();
  }
}

// The templated ctor allows copy construction from any object that's
// castable.  Thus, it does downcasts:
//   base_ref = derived_ref
template <class T_CppObject>
  template <class T_CastFrom>
inline
RefPtr<T_CppObject>::RefPtr(const RefPtr<T_CastFrom>& src)
:
  // A different RefPtr<> will not allow us access to __cpp_object.  We need
  // to add a get_underlying() for this, but that would encourage incorrect
  // use, so we use the less well-known operator->() accessor:
  __cpp_object (src.operator->()),
  __ref_count(src.refcount_ptr()),
  __ref_mutex(src.refmutex_ptr())
{
  if(__cpp_object && __ref_count && __ref_mutex) {
    __ref_mutex->lock();
    ++(*__ref_count);
    __ref_mutex->unlock();
  }
}

template <class T_CppObject> inline
void
RefPtr<T_CppObject>::swap(RefPtr<T_CppObject>& other)
{
  T_CppObject *const temp = __cpp_object;
  int *temp_count         = __ref_count; 
  Mutex *temp_mutex       = __ref_mutex;

  __cpp_object = other.__cpp_object;
  __ref_count  = other.__ref_count;
  __ref_mutex  = other.__ref_mutex;

  other.__cpp_object = temp;
  other.__ref_count  = temp_count;
  other.__ref_mutex  = temp_mutex;
}

template <class T_CppObject> inline
RefPtr<T_CppObject>&
RefPtr<T_CppObject>::operator=(const RefPtr<T_CppObject>& src)
{
  // In case you haven't seen the swap() technique to implement copy
  // assignment before, here's what it does:
  //
  // 1) Create a temporary RefPtr<> instance via the copy ctor, thereby
  //    increasing the reference count of the source object.
  //
  // 2) Swap the internal object pointers of *this and the temporary
  //    RefPtr<>.  After this step, *this already contains the new pointer,
  //    and the old pointer is now managed by temp.
  //
  // 3) The destructor of temp is executed, thereby unreferencing the
  //    old object pointer.
  //
  // This technique is described in Herb Sutter's "Exceptional C++", and
  // has a number of advantages over conventional approaches:
  //
  // - Code reuse by calling the copy ctor.
  // - Strong exception safety for free.
  // - Self assignment is handled implicitely.
  // - Simplicity.
  // - It just works and is hard to get wrong; i.e. you can use it without
  //   even thinking about it to implement copy assignment whereever the
  //   object data is managed indirectly via a pointer, which is very common.

  RefPtr<T_CppObject> temp (src);
  this->swap(temp);
  return *this;
}

template <class T_CppObject> inline
RefPtr<T_CppObject>&
RefPtr<T_CppObject>::operator=(T_CppObject *ptr)
{
  RefPtr<T_CppObject> temp(ptr);
  this->swap(temp);
  return *this;
}


template <class T_CppObject>
  template <class T_CastFrom>
inline
RefPtr<T_CppObject>&
RefPtr<T_CppObject>::operator=(const RefPtr<T_CastFrom>& src)
{
  RefPtr<T_CppObject> temp (src);
  this->swap(temp);
  return *this;
}

template <class T_CppObject> inline
bool
RefPtr<T_CppObject>::operator==(const RefPtr<T_CppObject>& src) const
{
  return (__cpp_object == src.__cpp_object);
}

template <class T_CppObject> inline
bool
RefPtr<T_CppObject>::operator!=(const RefPtr<T_CppObject>& src) const
{
  return (__cpp_object != src.__cpp_object);
}

template <class T_CppObject> inline
RefPtr<T_CppObject>::operator bool() const
{
  return (__cpp_object != 0);
}

template <class T_CppObject> inline
void RefPtr<T_CppObject>::clear()
{
  RefPtr<T_CppObject> temp; // swap with an empty RefPtr<> to clear *this
  this->swap(temp);
}

template <class T_CppObject> inline
void RefPtr<T_CppObject>::reset()
{
  RefPtr<T_CppObject> temp; // swap with an empty RefPtr<> to clear *this
  this->swap(temp);
}

template <class T_CppObject>
  template <class T_CastFrom>
inline
RefPtr<T_CppObject>
RefPtr<T_CppObject>::cast_dynamic(const RefPtr<T_CastFrom>& src)
{
  T_CppObject *const cpp_object = dynamic_cast<T_CppObject*>(src.operator->());

  if(cpp_object) //Check whether dynamic_cast<> succeeded so we don't pass a null object with a used refcount:
    return RefPtr<T_CppObject>(cpp_object, src.refcount_ptr(), src.refmutex_ptr());
  else
    return RefPtr<T_CppObject>();
}

template <class T_CppObject>
  template <class T_CastFrom>
inline
RefPtr<T_CppObject>
RefPtr<T_CppObject>::cast_static(const RefPtr<T_CastFrom>& src)
{
  T_CppObject *const cpp_object = static_cast<T_CppObject*>(src.operator->());

  return RefPtr<T_CppObject>(cpp_object, src.refcount_ptr(), src.refmutex_ptr());
}

template <class T_CppObject>
  template <class T_CastFrom>
inline
RefPtr<T_CppObject>
RefPtr<T_CppObject>::cast_const(const RefPtr<T_CastFrom>& src)
{
  T_CppObject *const cpp_object = const_cast<T_CppObject*>(src.operator->());

  return RefPtr<T_CppObject>(cpp_object, src.refcount_ptr(), src.refmutex_ptr());
}


/** Swap refptr instances.
 * @param lrp "left" refptr
 * @param rrp "right" refptr
 * @relates fawkes::RefPtr
 */
template <class T_CppObject> inline
void
swap(RefPtr<T_CppObject>& lrp, RefPtr<T_CppObject>& rrp)
{
  lrp.swap(rrp);
}

} // end namespace fawkes


#endif
