
/***************************************************************************
 *  circual_buffer.h - Circular buffer
 *
 *  Created: Fri Aug 15 12:00:42 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef __CORE_UTILS_CIRCULAR_BUFFER_H_
#define __CORE_UTILS_CIRCULAR_BUFFER_H_

#include <deque>

namespace fawkes {

/** @class CircularBuffer <core/utils/circular_buffer.h>
 * Circular buffer with a fixed size.
 * This class provides a a circular buffer.
 * A circular buffer is a container with a fixed (maximum) size.
 * It automatically maintains its size by removing elements from the front,
 * if necessary. This implementation does not allow any element manipulation
 * other than push_back() and pop_front(). All returned references to elements
 * are constant.
 *
 * @ingroup FCL
 * @author Till Hofmann
 */
template <typename Type>
class CircularBuffer
{
 public:

  /** The size_type of the buffer */
  typedef size_t size_type;
  /** The CircularBuffer's iterator is a std::deque iterator */
  typedef typename std::deque<Type>::const_iterator const_iterator;
  /** iterator is also const, we don't want to manipulate any elements */
  typedef const_iterator iterator;

  /** Constructor.
   * @param n the maximum size of the buffer */
  CircularBuffer(size_type n)
    : __deque(),
      __max_size(n)
  {}

  /** Copy constructor.
   * @param other CircularBuffer to copy
   */
  CircularBuffer(const CircularBuffer<Type> &other)
    : __deque(other.get_deque()),
      __max_size(other.get_max_size())
  {}

  /** Destructor. */
  virtual ~CircularBuffer()
  {}

  /** Insert an element at the end of the buffer
   *  and delete the first element if necessary
   *  @param val the value to insert
   */
  virtual void push_back(const Type& val)
  {
    if (__deque.size() >= __max_size) {
      __deque.pop_front();
    }
    __deque.push_back(val);
  }

  /** Delete the first element */
  virtual void pop_front()
  {
    __deque.pop_front();
  }

  /** Get the maximum size of the buffer
   * @return the maximum size
   */
  virtual size_type get_max_size() const
  {
    return __max_size;
  }

  /** Get the deque used to store the elements
   * @return the deque
   */
  virtual std::deque<Type> get_deque() const
  {
    return __deque;
  }

  /** Element access
   * @param n position of the element
   * @return reference to the n-th element
   */
  virtual const Type & operator[](size_type n) const
  {
    return __deque[n];
  }

  /** Element access
   * @param n position of the element
   * @return reference to the n-th element
   */
  virtual const Type & at(size_type n) const
  {
    return __deque.at(n);
  }

  /** Access the first element in the buffer
   * @return reference to the first element
   */
  virtual const Type & front() const
  {
    return __deque.front();
  }

  /** Access the last element in the buffer
   * @return reference to the last element
   */
  virtual const Type & back() const
  {
    return __deque.back();
  }

  /** Get iterator to the beginning
   * @return iterator
   */
  virtual const_iterator begin() const
  {
    return __deque.begin();
  }

  /** Get iterator to the end
   * @return iterator
   */
  virtual const_iterator end() const
  {
    return __deque.end();
  }

  /** Get actual size of the buffer
   * @return number of elements in the buffer
   */
  virtual size_type size() const
  {
    return __deque.size();
  }

 protected:
  /** The deque used to store the data */
  std::deque<Type> __deque;
  /** The maximum size of the circular buffer */
  size_type __max_size;

};

} // end namespace fawkes

#endif
