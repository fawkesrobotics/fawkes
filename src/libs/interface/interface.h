
/***************************************************************************
 *  interface.h - BlackBoard Interface
 *
 *  Generated: Mon Oct 09 18:34:11 2006
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

#ifndef __INTERFACE_H_
#define __INTERFACE_H_

#include <interface/message.h>
#include <interface/message_queue.h>
#include <core/exception.h>

#define __INTERFACE_TYPE_SIZE 32
#define __INTERFACE_ID_SIZE 32

class RefCountRWLock;
class InterfaceMediator;
class MessageMediator;

class InterfaceWriteDeniedException : public Exception
{
 public:
  InterfaceWriteDeniedException(const char *type, const char *id);
};

class InterfaceInvalidMessageException : public Exception
{
 public:
  InterfaceInvalidMessageException(const char *msg_type, const char *interface_type);
};

class Interface
{
 friend class BlackBoardInterfaceManager;
 friend class BlackBoardMessageManager;

 public:
  virtual ~Interface();

  bool          oftype(const char *interface_type) const;
  unsigned int  datasize() const;
  const char *  type() const;
  const char *  id() const;
  unsigned int  serial() const;
  bool          operator== (Interface &comp) const;

  void          read();
  void          write();

  bool          hasWriter() const;

  unsigned int  msgq_enqueue(Message *message);
  void          msgq_remove(Message *message);
  void          msgq_remove(unsigned int message_id);
  unsigned int  msgq_size();
  void          msgq_flush();
  void          msgq_lock();
  bool          msgq_tryLock();
  void          msgq_unlock();
  void          msgq_pop();
  Message *     msgq_first();
  bool          msgq_empty();

  template <class MessageType>
    bool           msgq_first_is();

  /** Check if first message has desired type.
   * @return true, if message has desired type, false otherwise
   */
  template <class MessageType>
    MessageType *  msgq_first();

  MessageQueue::MessageIterator  msgq_begin();
  MessageQueue::MessageIterator  msgq_end();

 protected:
  Interface();
  virtual bool  messageValid(const Message *message) const = 0;

  void         *data_ptr;
  unsigned int  data_size;

 private:
  unsigned int       msgq_append(Message *message);

  char               __type[__INTERFACE_TYPE_SIZE + 1];
  char               __id[__INTERFACE_ID_SIZE + 1];
  unsigned int       __instance_serial;

  void *             __mem_data_ptr;
  void *             __mem_real_ptr;
  unsigned int       __mem_serial;
  bool               __write_access;

  RefCountRWLock    *__rwlock;

  InterfaceMediator *__interface_mediator;
  MessageMediator   *__message_mediator;
  MessageQueue      *__message_queue;


  typedef struct imsg_list_t {
    imsg_list_t  *next;		/**< pointer to next element in list */
    Message       msg;		/**< pointer to message */
  };

};


template <class MessageType>
MessageType *
Interface::msgq_first()
{
  return dynamic_cast<MessageType *>(__message_queue->first());
}


/** Check if first message has desired type.
 * @return true, if message has desired type, false otherwise
 */
template <class MessageType>
bool
Interface::msgq_first_is()
{
  return (dynamic_cast<MessageType *>(__message_queue->first()) != 0);
}


typedef Interface *  (* InterfaceFactoryFunc)  (void);
typedef void         (* InterfaceDestroyFunc)  (Interface *);

/** Friend for interface generator function. */
#define INTERFACE_MGMT_FRIENDS(interface_class)				\
  friend Interface * private_new##interface_class();			\
  friend void private_delete##interface_class(interface_class *interface);

/** Interface generator function for this plugin.
 * @return an instance of the desired interface
 */
#define INTERFACE_GENERATOR(interface_class)			\
  Interface *							\
  private_new##interface_class()				\
  {								\
    return new interface_class();				\
  }


/** Interface delete function for this plugin.
 * @return an instance of the desired interface
 */
#define INTERFACE_DELETER(interface_class)			\
  void								\
  private_delete##interface_class(interface_class *interface)	\
  {								\
    delete interface;						\
  }


/** Interface factory function.
 * @return an instance of the desired interface
 */
#define INTERFACE_FACTORY(interface_class)			\
  extern "C"							\
  Interface *							\
  new##interface_class()						\
  {								\
    return private_new##interface_class();			\
  }


/** Interface destruction function.
 * @param interface The interface that is to be destroyed.
 */
#define INTERFACE_DESTROY(interface_class)		\
  extern "C"						\
  void							\
  delete##interface_class (interface_class *interface)	\
  {							\
    private_delete##interface_class(interface);		\
  }

/** Export interface.
 * This will create appropriate interface factory and destroy functions.
 */
#define EXPORT_INTERFACE(interface_class) \
  INTERFACE_GENERATOR(interface_class)	  \
  INTERFACE_DELETER(interface_class)	  \
  INTERFACE_FACTORY(interface_class)	  \
  INTERFACE_DESTROY(interface_class)

#endif
