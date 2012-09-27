
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

#ifndef __CORE_EXCEPTION_H_
#define __CORE_EXCEPTION_H_

// needed for va_list
#include <cstdarg>
#include <exception>

namespace fawkes {


class Mutex;

class Exception : public std::exception {
 public:

  Exception(const char *format, ...) throw();
  Exception(int errno, const char *format, ...) throw();
  Exception(const Exception &exc) throw();
  virtual ~Exception() throw();

  virtual void raise();
  void prepend(const char *format, ...) throw();
  void append(const char *format, ...) throw();
  void append_va(const char *format, va_list va) throw();
  void append(const Exception &e) throw();
  void print_trace() throw();
  void print_backtrace() const throw();
  char *  generate_backtrace() const throw();

  int get_errno() throw();

  void          set_type_id(const char *id);
  const char *  type_id() const;

  virtual const char* what() const throw();
  virtual const char* what_no_backtrace() const throw();

  Exception& operator=(const Exception &exc) throw();

 protected:
   /** Internal exception message list */
   struct message_list_t {
     message_list_t  *next;   /**< pointer to next element, NULL if last element */
     char            *msg;    /**< pointer to message, may not be NULL, will be freed
			       *   in dtor */
   };

 public:
  class iterator
  {
    friend class Exception;
   private:
    iterator(message_list_t *message_list);
   public:
    iterator(const iterator &i);
    iterator();

    iterator &    operator++ ();        // prefix
    iterator      operator++ (int inc); // postfix

    bool          operator== (const iterator & i) const;
    bool          operator!= (const iterator & i) const;

    const char *  operator*  () const;
    iterator &    operator=  (const iterator & i);

   private:
    message_list_t *mlist;
  };

  iterator begin() throw();
  iterator end() throw();

 protected:
  Exception() throw();

  void append_nolock(const char *format, ...) throw();
  void append_nolock_va(const char *format, va_list va) throw();
  void append_nolock_nocopy(char *msg) throw();
  void prepend_nolock_va(const char *format, va_list va) throw();
  void copy_messages(const Exception &exc) throw();

  message_list_t  *messages;
  message_list_t  *messages_iterator;
  message_list_t  *messages_end;
  Mutex           *messages_mutex;

  int              _errno;

 private:
  const char *__type_id;
};


} // end namespace fawkes

#endif
