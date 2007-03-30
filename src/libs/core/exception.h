
/***************************************************************************
 *  exception.h - basic exception
 *
 *  Generated: Thu Feb 09 13:02:37 2006 (from FireVision)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __CORE_EXCEPTION_H_
#define __CORE_EXCEPTION_H_

// needed for va_list
#include <cstdarg>

class Mutex;

class Exception {
 public:

  Exception(const char *msg);
  Exception(const char *msg, int errno);
  Exception(const Exception &exc);
  virtual ~Exception();

  virtual const char * c_str();
  virtual void raise();
  void append(const char *format, ...);
  void printTrace();

  int errno() const;

  Exception& operator=(const Exception &exc);

 protected:

  /** typedef struct for message list */
  typedef struct message_list_t {
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

  iterator begin();
  iterator end();

 protected:
  Exception();

  void append_nolock(const char *msg);
  void append_nolock(const char *format, va_list va);
  void append_nolock_nocopy(char *msg);
  void copy_messages(const Exception &exc);

  message_list_t  *messages;
  message_list_t  *messages_iterator;
  message_list_t  *messages_end;
  Mutex           *messages_mutex;

  int              _errno;
};



#endif
