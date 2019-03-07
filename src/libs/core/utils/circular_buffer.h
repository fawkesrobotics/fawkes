
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

#ifndef _CORE_UTILS_CIRCULAR_BUFFER_H_
#define _CORE_UTILS_CIRCULAR_BUFFER_H_

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
	CircularBuffer(size_type n) : deque_(), max_size_(n)
	{
	}

	/** Copy constructor.
   * @param other CircularBuffer to copy
   */
	CircularBuffer(const CircularBuffer<Type> &other)
	: deque_(other.get_deque()), max_size_(other.get_max_size())
	{
	}

	/** Destructor. */
	~CircularBuffer()
	{
	}

	/** Assignment operator.
   * @param other CircularBuffer to copy
   * @return reference to this instance
   */
	CircularBuffer<Type> &
	operator=(const CircularBuffer<Type> &other)
	{
		deque_    = other.get_deque();
		max_size_ = other.get_max_size();
		return *this;
	}

	/** Insert an element at the end of the buffer
   *  and delete the first element if necessary
   *  @param val the value to insert
   */
	void
	push_back(const Type &val)
	{
		if (deque_.size() >= max_size_) {
			deque_.pop_front();
		}
		deque_.push_back(val);
	}

	/** Delete the first element */
	void
	pop_front()
	{
		deque_.pop_front();
	}

	/** Get the maximum size of the buffer
   * @return the maximum size
   */
	size_type
	get_max_size() const
	{
		return max_size_;
	}

	/** Get the deque used to store the elements
   * @return the deque
   */
	std::deque<Type>
	get_deque() const
	{
		return deque_;
	}

	/** Element access
   * @param n position of the element
   * @return reference to the n-th element
   */
	const Type &operator[](size_type n) const
	{
		return deque_[n];
	}

	/** Element access
   * @param n position of the element
   * @return reference to the n-th element
   */
	const Type &
	at(size_type n) const
	{
		return deque_.at(n);
	}

	/** Access the first element in the buffer
   * @return reference to the first element
   */
	const Type &
	front() const
	{
		return deque_.front();
	}

	/** Access the last element in the buffer
   * @return reference to the last element
   */
	const Type &
	back() const
	{
		return deque_.back();
	}

	/** Get iterator to the beginning
   * @return iterator
   */
	const_iterator
	begin() const
	{
		return deque_.begin();
	}

	/** Get iterator to the end
   * @return iterator
   */
	const_iterator
	end() const
	{
		return deque_.end();
	}

	/** Get actual size of the buffer
   * @return number of elements in the buffer
   */
	size_type
	size() const
	{
		return deque_.size();
	}

protected:
	/** The deque used to store the data */
	std::deque<Type> deque_;
	/** The maximum size of the circular buffer */
	size_type max_size_;
};

} // end namespace fawkes

#endif
