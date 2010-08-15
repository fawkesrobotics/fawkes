
/***************************************************************************
 *  message.h - BlackBoard message
 *
 *  Created: Sun Oct 08 00:08:10 2006
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <interface/field_iterator.h>
#include <interface/types.h>
#include <core/utils/refcount.h>

#define __INTERFACE_MESSAGE_TYPE_SIZE 32

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;
class Interface;
class InterfaceFieldIterator;
class Time;

class Message : public RefCount
{
 friend class Interface;
 public:
  Message(const char *type);
  Message(const Message *mesg);
  Message(const Message &mesg);
  virtual ~Message();

  Message &         operator=  (const Message & m);

  unsigned int      id() const;
  void              set_id(unsigned int message_id);
  void              mark_enqueued();
  bool              enqueued() const;
  const Time *      time_enqueued() const;

  unsigned int      sender_id() const;
  const char *      sender_thread_name() const;
  Interface *       interface() const;
  const char *      type() const;

  InterfaceFieldIterator     fields();
  InterfaceFieldIterator     fields_end();

  unsigned int      num_fields() const;

  const void *      datachunk() const;
  unsigned int      datasize() const;

  unsigned int      hops() const;
  void              set_hops(unsigned int hops);

  void              set_from_chunk(const void *chunk);

  unsigned int      recipient() const;

  virtual Message * clone() const;

  /** Check if message has desired type.
   * @return true, if message has desired type, false otherwise
   */
  template <class MessageType>
    bool           is_of_type();

 private: // fields
  unsigned int  __message_id;
  unsigned int  __hops;
  bool          __enqueued;
  Time         *__time_enqueued;

  unsigned int  recipient_interface_mem_serial;  
  unsigned int  sender_interface_instance_serial;  

  char          *_type;
  char          *_sender_thread_name;
  unsigned int   _sender_id;

  Interface     *_transmit_via_iface;

  interface_fieldinfo_t  *__fieldinfo_list;

  unsigned int __num_fields;

 private: // methods
  void              set_interface(Interface *iface);

 protected:
  void add_fieldinfo(interface_fieldtype_t type, const char *name,
		     size_t length, void *value, const char *enumtype = 0);

  void         *data_ptr;
  unsigned int  data_size;

  /** Timestamp data, must be present and first entries for each interface
   * data structs! This leans on timeval struct. */
  typedef struct {
    int64_t timestamp_sec;	/**< time in seconds since Unix epoch */
    int64_t timestamp_usec;	/**< additional time microseconds */
  } message_data_ts_t;
  message_data_ts_t  *data_ts;	/**< data timestamp aliasing pointer */
};

template <class MessageType>
bool
Message::is_of_type()
{
  return (dynamic_cast<MessageType *>(this) != 0);
}


} // end namespace fawkes

#endif
