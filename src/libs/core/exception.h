
/***************************************************************************
 *  exception.h - basic exception
 *
 *  Generated: Thu Feb 09 13:02:37 2006 (from FireVision)
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

#ifndef _CORE_EXCEPTION_H_
#define _CORE_EXCEPTION_H_

// needed for va_list
#include <cstdarg>
#include <exception>

namespace fawkes {

class Mutex;

class Exception : public std::exception
{
public:
	Exception(const char *format, ...) noexcept;
	Exception(int errnoval, const char *format, ...) noexcept;
	Exception(const Exception &exc) noexcept;
	virtual ~Exception() noexcept;

	virtual void raise();
	void         prepend(const char *format, ...) noexcept;
	void         append(const char *format, ...) noexcept;
	void         append_va(const char *format, va_list va) noexcept;
	void         append(const Exception &e) noexcept;
	void         print_trace() noexcept;
	void         print_backtrace() const noexcept;
	char *       generate_backtrace() const noexcept;

	int get_errno() noexcept;

	void        set_type_id(const char *id);
	const char *type_id() const;

	virtual const char *what() const noexcept;
	virtual const char *what_no_backtrace() const noexcept;

	Exception &operator=(const Exception &exc) noexcept;

protected:
	/** Internal exception message list */
	struct message_list_t
	{
		message_list_t *next; /**< pointer to next element, NULL if last element */
		char *          msg;  /**< pointer to message, may not be NULL, will be freed
			       *   in dtor */
	};

public:
	class iterator
	{
		friend Exception;

	private:
		iterator(message_list_t *message_list);

	public:
		iterator(const iterator &i);
		iterator();

		iterator &operator++();        // prefix
		iterator  operator++(int inc); // postfix

		bool operator==(const iterator &i) const;
		bool operator!=(const iterator &i) const;

		const char *operator*() const;
		iterator &  operator=(const iterator &i);

	private:
		message_list_t *mlist;
	};

	iterator begin() noexcept;
	iterator end() noexcept;

protected:
	Exception() noexcept;

	void append_nolock(const char *format, ...) noexcept;
	void append_nolock_va(const char *format, va_list va) noexcept;
	void append_nolock_nocopy(char *msg) noexcept;
	void prepend_nolock_va(const char *format, va_list va) noexcept;
	void copy_messages(const Exception &exc) noexcept;

	message_list_t *messages;
	message_list_t *messages_iterator;
	message_list_t *messages_end;
	Mutex *         messages_mutex;

	int _errno;

private:
	const char *type_id_;
};

} // end namespace fawkes

#endif
