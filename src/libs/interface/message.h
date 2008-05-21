
/***************************************************************************
 *  message.h - BlackBoard message
 *
 *  Generated: Sun Oct 08 00:08:10 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __INTERFACE_MESSAGE_H_
#define __INTERFACE_MESSAGE_H_

#include <core/utils/refcount.h>

#define __INTERFACE_MESSAGE_TYPE_SIZE 32

namespace fawkes {

class Mutex;
class Interface;

class Message : public RefCount
{
 friend class Interface;
 friend class MessageQueue;
 public:
 /** Message status.
  * A message has a processing status. This status can have one of the following
  * values and is global to all messages.
  */
 typedef enum {
   Undefined,		/**< status is undefined (message not queued) */
   Enqueued,		/**< message has been enqueued, but not yet been processed */ 
   InProgress,		/**< processing the message has started but is not finished, yet */
   Success,		/**< message has been successfully processed */
   Failure		/**< an error occured during message processing */
 } MessageStatus;

  Message(const char *type);
  Message(const Message *mesg);
  Message(const Message &mesg);
  virtual ~Message();

  Message &         operator=  (const Message & m);

  unsigned int      id() const;
  void              set_status(MessageStatus status);
  MessageStatus     status() const;
  void              set_sub_status(unsigned int sub_status);
  unsigned int      sub_status() const;
  unsigned int      sender_id() const;
  const char *      sender_thread_name() const;
  Interface *       interface() const;
  const char *      type() const;

  const void *      datachunk() const;
  unsigned int      datasize() const;

  void              set_from_chunk(const void *chunk);

  unsigned int      recipient() const;

  virtual Message * clone() const;

 private:

  void              set_interface(Interface *iface);
  void              set_id(unsigned int message_id);

  unsigned int  __message_id;

  unsigned int  recipient_interface_mem_serial;  
  unsigned int  sender_interface_instance_serial;  

  MessageStatus  _status;
  unsigned int   _substatus;

  char          *_type;
  char          *_sender_thread_name;
  unsigned int   _sender_id;

  Interface     *_transmit_via_iface;

 protected:
  void         *data_ptr;
  unsigned int  data_size;
};

} // end namespace fawkes

#endif
