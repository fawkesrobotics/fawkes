
/***************************************************************************
 *  message.h - BlackBoard message
 *
 *  Generated: Sun Oct 08 00:08:10 2006
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

#ifndef __MESSAGE_H_
#define __MESSAGE_H_

#include <core/utils/refcount.h>
#include <sys/types.h>

class Mutex;
class Interface;

class Message : public RefCount
{
 friend class Interface;
 friend class MessageQueue;
 friend class BlackBoardMessageManager;
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
  Message(Message *mesg);
  Message(Message &mesg);
  virtual ~Message();

  Message &         operator=  (const Message & m);

  void              set_status(MessageStatus status);
  MessageStatus     status() const;
  void              set_sub_status(unsigned int sub_status);
  unsigned int      sub_status() const;
  pthread_t         sender_id() const;
  const char *      sender() const;
  Interface *       interface() const;
  const char *      type() const;

 private:
  virtual void *        data();
  virtual unsigned int  datasize();

  void                  set_interface(Interface *iface);

  unsigned int  message_id;

  unsigned int  recipient_interface_mem_serial;  
  unsigned int  sender_interface_instance_serial;  

  MessageStatus  _status;
  unsigned int   _substatus;

  char          *_type;
  char          *_sender;
  pthread_t      _sender_id;

  Interface     *_transmit_via_iface;

 protected:
  void         *data_ptr;
  unsigned int  data_size;
};

#endif
