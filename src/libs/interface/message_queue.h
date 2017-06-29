
/***************************************************************************
 *  message_queue.h - BlackBoard Interface message queue
 *
 *  Created: Tue Oct 17 19:05:33 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __INTERFACE_MESSAGE_QUEUE_H_
#define __INTERFACE_MESSAGE_QUEUE_H_

#include <core/exception.h>
#include <core/exceptions/software.h>

namespace fawkes {

class Message;
class Mutex;


class MessageAlreadyQueuedException : public Exception {
 public:
  MessageAlreadyQueuedException();
};


class MessageQueue
{
 private:
  // define our own list type since std::list is way too fat
  /** Message list, internal only
   */
  struct msg_list_t {
    msg_list_t    *next;	/**< pointer to next element in list */
    unsigned int   msg_id;	/**< message id */
    Message       *msg;		/**< pointer to message */
  };

 public:
  MessageQueue();
  virtual ~MessageQueue();

  class MessageIterator
  {
    friend MessageQueue;
   private:
    MessageIterator(msg_list_t *cur);
   public:
    MessageIterator();
    MessageIterator(const MessageIterator &it);
    MessageIterator & operator++ ();        // prefix
    MessageIterator   operator++ (int inc); // postfix
    MessageIterator & operator+  (unsigned int i);
    MessageIterator & operator+= (unsigned int i);
    bool              operator== (const MessageIterator & c) const;
    bool              operator!= (const MessageIterator & c) const;
    Message *         operator*  () const;
    Message *         operator-> () const;
    MessageIterator & operator=  (const MessageIterator & c);

    unsigned int      id() const;

    template <class MessageType>
      bool            is() const;

    template <class MessageType>
      MessageType *   get() const;

   private:
    msg_list_t *cur;
  };


  void         append(Message *msg);
  void         remove(const Message *msg);
  void         remove(const unsigned int msg_id);
  void         insert_after(const MessageIterator &it, Message *msg);

  unsigned int size() const;

  void         flush();
  bool         empty() const;

  void         lock();
  bool         try_lock();
  void         unlock();

  Message *    first();
  void         pop();

  MessageIterator begin();
  MessageIterator end();

 private:
  void remove(msg_list_t *l, msg_list_t *p);

  msg_list_t  *__list;
  msg_list_t  *__end_el;
  Mutex       *__mutex;
};


/** Check if message is of given type.
 * The current message is checked if it is of the type that the
 * template parameter determines. Use non-pointer template arguments!
 * @return true, if the current message is of the given type, false otherwise
 */
template <class MessageType>
bool
MessageQueue::MessageIterator::is() const
{
  MessageType *msg = dynamic_cast<MessageType *>(cur->msg);
  return ( msg != 0 );
}


/** Get current message of given type.
 * This will return the current message of the given template type. An TypeMismatchException
 * is thrown if the current message is not of the requested type.
 * @exception TypeMismatchException thrown, if current message is not of requested type.
 * @return current message of requested type
 */
template <class MessageType>
MessageType *
MessageQueue::MessageIterator::get() const
{
  MessageType *msg = dynamic_cast<MessageType *>(cur->msg);
  if ( msg == 0 ) {
    throw TypeMismatchException("Message types do not match (get)");
  }
  return msg;
}

} // end namespace fawkes

#endif
