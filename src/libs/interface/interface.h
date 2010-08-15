
/***************************************************************************
 *  interface.h - BlackBoard Interface
 *
 *  Created: Mon Oct 09 18:34:11 2006
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

#ifndef __INTERFACE_H_
#define __INTERFACE_H_

#include <interface/message.h>
#include <interface/message_queue.h>
#include <core/exception.h>

#include <cstddef>
#include <list>
#define __STD_LIMIT_MACROS
#include <stdint.h>

#define __INTERFACE_TYPE_SIZE   32
#define __INTERFACE_ID_SIZE     32
// We use MD5 as interface hash
#define __INTERFACE_HASH_SIZE   16
//  UID is:                                   type  ::   id
#define __INTERFACE_UID_SIZE __INTERFACE_TYPE_SIZE + 2 + __INTERFACE_ID_SIZE

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RefCountRWLock;
class InterfaceMediator;
class MessageMediator;
class Time;
class Clock;

class InterfaceWriteDeniedException : public Exception
{
 public:
  InterfaceWriteDeniedException(const char *type, const char *id, const char *msg);
};

class InterfaceMessageEnqueueException : public Exception
{
 public:
  InterfaceMessageEnqueueException(const char *type, const char *id);
};

class InterfaceInvalidMessageException : public Exception
{
 public:
  InterfaceInvalidMessageException(const Interface *interface, const Message *message);
};


class InterfaceInvalidException : public Exception
{
 public:
  InterfaceInvalidException(const Interface *interface, const char *method);
};

class Interface
{
 friend class BlackBoardInterfaceManager;
 friend class BlackBoardInstanceFactory;
 friend class BlackBoardMessageManager;
 friend class BlackBoardInterfaceProxy;

 public:
  virtual ~Interface();

  bool                    oftype(const char *interface_type) const;
  const void *            datachunk() const;
  unsigned int            datasize() const;
  const char *            type() const;
  const char *            id() const;
  const char *            uid() const;
  unsigned short          serial() const;
  unsigned int            mem_serial() const;
  bool                    operator== (Interface &comp) const;
  const unsigned char *   hash() const;
  size_t                  hash_size() const;
  const char *            hash_printable() const;
  bool                    is_writer() const;
  void                    set_validity(bool valid);
  bool                    is_valid() const;

  void                    set_from_chunk(void *chunk);

  virtual Message *       create_message(const char *type) const = 0;
  virtual void            copy_values(const Interface *interface) = 0;
  virtual const char *    enum_tostring(const char *enumtype, int val) const = 0;

  void          read();
  void          write();

  bool          has_writer() const;
  unsigned int  num_readers() const;

  bool          changed() const;
  const Time *  timestamp() const;
  void          set_auto_timestamping(bool enabled);
  void          set_timestamp(const Time *t = NULL);
  void          set_clock(Clock *clock);

  std::list<const char *> get_message_types();

  unsigned int  msgq_enqueue(Message *message);
  unsigned int  msgq_enqueue_copy(Message *message);
  void          msgq_remove(Message *message);
  void          msgq_remove(unsigned int message_id);
  unsigned int  msgq_size();
  void          msgq_flush();
  void          msgq_lock();
  bool          msgq_try_lock();
  void          msgq_unlock();
  void          msgq_pop();
  Message *     msgq_first();
  bool          msgq_empty();

  /** Check if first message has desired type.
   * @return true, if message has desired type, false otherwise
   */
  template <class MessageType>
    bool           msgq_first_is();

  /** Get first message casted to the desired type.
   * @return message casted to desired type
   * @exception TypeMismatchException thrown if message is not of desired type
   */
  template <class MessageType>
    MessageType *  msgq_first();

  /** Get first message casted to the desired type.
   * @param msg reference to pointer to message of desired type, upon successful
   * return points to the message. 
   * @return message casted to desired type (same as msg parameter)
   * @exception TypeMismatchException thrown if message is not of desired type
   */
  template <class MessageType>
    MessageType *  msgq_first(MessageType *&msg);

  MessageQueue::MessageIterator  msgq_begin();
  MessageQueue::MessageIterator  msgq_end();

  /* Introspection */

  /** Message info list */
  struct interface_messageinfo_t {
    const char              *type;   /**< the type of the message */
    interface_messageinfo_t *next;   /**< the next field, NULL if last */
  };

  InterfaceFieldIterator fields();
  InterfaceFieldIterator fields_end();

  unsigned int num_fields();

  /* Convenience */
  static void parse_uid(const char *uid, char **type, char **id);

 protected:
  Interface();
  virtual bool  message_valid(const Message *message) const = 0;

  void set_hash(unsigned char *ihash);
  void add_fieldinfo(interface_fieldtype_t type, const char *name,
		     size_t length, void *value, const char *enumtype = 0);
  void add_messageinfo(const char *name);

  void         *data_ptr;
  unsigned int  data_size;
  bool          data_changed;

  /** Timestamp data, must be present and first entries for each interface
   * data structs! This leans on timeval struct. */
  typedef struct {
    int64_t timestamp_sec;	/**< time in seconds since Unix epoch */
    int64_t timestamp_usec;	/**< additional time microseconds */
  } interface_data_ts_t;
  interface_data_ts_t  *data_ts;

 private:
  void msgq_append(Message *message);
  void set_type_id(const char *type, const char *id);
  void set_instance_serial(unsigned short instance_serial);
  void set_mediators(InterfaceMediator *iface_mediator,
				   MessageMediator *msg_mediator);
  void set_memory(unsigned int serial, void *real_ptr, void *data_ptr);
  void set_readwrite(bool write_access, RefCountRWLock *rwlock);

  inline unsigned int next_msg_id()
  {
    return (__instance_serial << 16) | ++__next_message_id;
  }

  char               __type[__INTERFACE_TYPE_SIZE + 1];
  char               __id[__INTERFACE_ID_SIZE + 1];
  char               __uid[__INTERFACE_UID_SIZE + 1];
  unsigned char      __hash[__INTERFACE_HASH_SIZE];
  char               __hash_printable[__INTERFACE_HASH_SIZE * 2 + 1];

  unsigned short     __instance_serial;
  bool               __valid;

  void *             __mem_data_ptr;
  void *             __mem_real_ptr;
  unsigned int       __mem_serial;
  bool               __write_access;

  RefCountRWLock    *__rwlock;

  InterfaceMediator *__interface_mediator;
  MessageMediator   *__message_mediator;
  MessageQueue      *__message_queue;
  unsigned short     __next_message_id;

  interface_fieldinfo_t   *__fieldinfo_list;
  interface_messageinfo_t *__messageinfo_list;

  unsigned int       __num_fields;

  Clock             *__clock;
  Time              *__timestamp;
  Time              *__local_read_timestamp;
  bool               __auto_timestamping;
};


template <class MessageType>
MessageType *
Interface::msgq_first()
{
  MessageType *m = dynamic_cast<MessageType *>(__message_queue->first());
  if (m) {
    return m;
  } else {
    throw TypeMismatchException("Message is not of desired type");
  }
}


template <class MessageType>
MessageType *
Interface::msgq_first(MessageType *&msg)
{
  msg = this->msgq_first<MessageType>();
  return msg;
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


/** Interface destructor function for the shared library.
 * Do not use directly. Use EXPORT_INTERFACE macro.
 * @param interface Interface to destroy
 */
typedef void         (* InterfaceDestroyFunc)  (Interface *interface);

/** Interface generator function for the shared library
 * Do not use directly. Use EXPORT_INTERFACE macro.
 */
typedef Interface *  (* InterfaceFactoryFunc)  (void);


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
  interface_factory()						\
  {								\
    return private_new##interface_class();			\
  }


/** Interface destruction function.
 * @param interface The interface that is to be destroyed.
 */
#define INTERFACE_DESTROY(interface_class)		\
  extern "C"						\
  void							\
  interface_destroy(interface_class *interface)		\
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

} // end namespace fawkes

#endif
