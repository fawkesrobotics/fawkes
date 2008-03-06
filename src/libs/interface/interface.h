
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

#include <cstddef>

#define __INTERFACE_TYPE_SIZE   32
#define __INTERFACE_ID_SIZE     32
// We use MD5 as interface hash
#define __INTERFACE_HASH_SIZE   16
//  UID is:                                   type  ::   id
#define __INTERFACE_UID_SIZE __INTERFACE_TYPE_SIZE + 2 + __INTERFACE_ID_SIZE

class RefCountRWLock;
class InterfaceMediator;
class MessageMediator;

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
  unsigned int            serial() const;
  unsigned int            mem_serial() const;
  bool                    operator== (Interface &comp) const;
  const unsigned char *   hash() const;
  size_t                  hash_size() const;
  const char *            hash_printable() const;
  bool                    is_writer() const;

  void                    set_from_chunk(void *chunk);

  virtual Message *       create_message(const char *type) const = 0;

  void          read();
  void          write();

  bool          has_writer() const;
  unsigned int  num_readers() const;


  unsigned int  msgq_enqueue(Message *message);
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

  template <class MessageType>
    bool           msgq_first_is();

  /** Check if first message has desired type.
   * @return true, if message has desired type, false otherwise
   */
  template <class MessageType>
    MessageType *  msgq_first();

  MessageQueue::MessageIterator  msgq_begin();
  MessageQueue::MessageIterator  msgq_end();

  /* Introspection */

  /** Interface field type*/
  typedef enum {
    IFT_BOOL,		/**< boolean field */
    IFT_INT,		/**< integer field */
    IFT_UINT,		/**< unsigned integer field */
    IFT_LONGINT,	/**< long int field */
    IFT_LONGUINT,	/**< unsigned long int field */
    IFT_FLOAT,		/**< float field */
    IFT_STRING		/**< string field */
  } interface_fieldtype_t;

  /** Interface field info list */
  struct interface_fieldinfo_t {
    interface_fieldtype_t    type;	/**< type of this field */
    const char              *name;	/**< Name of this field */
    void                    *value;	/**< Current value of this field */
    interface_fieldinfo_t   *next;	/**< next field, NULL if last */
  };

  class FieldIterator
  {
    friend class Interface;
   public:
    FieldIterator();
    FieldIterator(const FieldIterator &fit);
    ~FieldIterator();
    FieldIterator &        operator++ ();        // prefix
    FieldIterator          operator++ (int inc); // postfix
    FieldIterator &        operator+  (unsigned int i);
    FieldIterator &        operator+= (unsigned int i);
    bool                   operator== (const FieldIterator & s) const;
    bool                   operator!= (const FieldIterator & s) const;
    const void *           operator*  () const;
    FieldIterator &        operator=  (const FieldIterator & shmit);

    interface_fieldtype_t  get_type() const;
    const char *           get_name() const;
    const void *           get_value() const;
    bool                   get_bool() const;
    int                    get_int() const;
    unsigned int           get_uint() const;
    long int               get_longint() const;
    unsigned long int      get_longuint() const;
    float                  get_float() const;
    const char *           get_string() const;

   protected:
    FieldIterator(const interface_fieldinfo_t *info_list);

   private:
    const interface_fieldinfo_t   *__infol;
  };

  FieldIterator fields();
  FieldIterator fields_end();

 protected:
  Interface();
  virtual bool  message_valid(const Message *message) const = 0;

  void set_hash(unsigned char ihash[__INTERFACE_HASH_SIZE]);
  void add_fieldinfo(interface_fieldtype_t type, const char *name, void *value);

  void         *data_ptr;
  unsigned int  data_size;

 private:
  unsigned int       msgq_append(Message *message);
  void               set_type_id(const char *type, const char *id);
  void               set_instance_serial(unsigned int instance_serial);
  void               set_mediators(InterfaceMediator *iface_mediator,
				   MessageMediator *msg_mediator);
  void               set_memory(unsigned int serial, void *real_ptr, void *data_ptr);
  void               set_readwrite(bool write_access, RefCountRWLock *rwlock);

  char               __type[__INTERFACE_TYPE_SIZE + 1];
  char               __id[__INTERFACE_ID_SIZE + 1];
  char               __uid[__INTERFACE_UID_SIZE + 1];
  unsigned char      __hash[__INTERFACE_HASH_SIZE];
  char               __hash_printable[__INTERFACE_HASH_SIZE * 2 + 1];

  unsigned int       __instance_serial;

  void *             __mem_data_ptr;
  void *             __mem_real_ptr;
  unsigned int       __mem_serial;
  bool               __write_access;

  RefCountRWLock    *__rwlock;

  InterfaceMediator *__interface_mediator;
  MessageMediator   *__message_mediator;
  MessageQueue      *__message_queue;

  interface_fieldinfo_t  *__info_list;

  struct imsg_list_t {
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
