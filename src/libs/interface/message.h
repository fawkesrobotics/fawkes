
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

#ifndef _INTERFACE_MESSAGE_H_
#define _INTERFACE_MESSAGE_H_

#include <core/exceptions/software.h>
#include <core/utils/refcount.h>
#include <interface/field_iterator.h>
#include <interface/types.h>

#include <cstring>
#include <type_traits>

#define INTERFACE_MESSAGE_TYPE_SIZE_ 64

namespace fawkes {

class Mutex;
class Interface;
class InterfaceFieldIterator;
class Time;

class Message : public RefCount
{
	friend Interface;

public:
	Message(const char *type);
	Message(const Message *mesg);
	Message(const Message &mesg);
	virtual ~Message();

	Message &operator=(const Message &m);

	unsigned int id() const;
	void         set_id(unsigned int message_id);
	void         mark_enqueued();
	bool         enqueued() const;
	const Time * time_enqueued() const;

	unsigned int sender_id() const;
	const char * sender_thread_name() const;
	Interface *  interface() const;
	const char * type() const;

	InterfaceFieldIterator fields();
	InterfaceFieldIterator fields_end();

	unsigned int num_fields() const;

	const void * datachunk() const;
	unsigned int datasize() const;

	unsigned int hops() const;
	void         set_hops(unsigned int hops);

	void set_from_chunk(const void *chunk);

	unsigned int recipient() const;

	virtual Message *clone() const;

	/** Check if message has desired type.
   * @return true, if message has desired type, false otherwise
   */
	template <class MessageType>
	bool is_of_type();

	/** Cast message to given type if possible.
   * Check with is_of_type() first if the message has the requested type.
   * @return message casted to requested type
   * @throw TypeMismatchException thrown if the message is not of the requested type
   */
	template <class MessageType>
	MessageType *as_type();

private: // fields
	unsigned int message_id_;
	unsigned int hops_;
	bool         enqueued_;
	Time *       time_enqueued_;

	unsigned int recipient_interface_mem_serial;
	unsigned int sender_interface_instance_serial;

	char *       _type;
	char *       _sender_thread_name;
	unsigned int _sender_id;

	Interface *_transmit_via_iface;

	interface_fieldinfo_t *fieldinfo_list_;

	unsigned int num_fields_;

private: // methods
	void set_interface(Interface *iface);

protected:
	void add_fieldinfo(interface_fieldtype_t       type,
	                   const char *                name,
	                   size_t                      length,
	                   void *                      value,
	                   const char *                enumtype = 0,
	                   const interface_enum_map_t *enum_map = 0);

	void *       data_ptr;
	unsigned int data_size;

	/** Timestamp data, must be present and first entries for each interface
   * data structs! This leans on timeval struct. */
	typedef struct
	{
		int64_t timestamp_sec;  /**< time in seconds since Unix epoch */
		int64_t timestamp_usec; /**< additional time microseconds */
	} message_data_ts_t;
	message_data_ts_t *data_ts; /**< data timestamp aliasing pointer */
};

template <class MessageType>
bool
Message::is_of_type()
{
	return (dynamic_cast<MessageType *>(this) != 0);
}

template <class MessageType>
MessageType *
Message::as_type()
{
	MessageType *m = dynamic_cast<MessageType *>(this);
	if (m) {
		return m;
	} else {
		throw fawkes::TypeMismatchException("Message is not of requested type");
	}
}

/** Set a field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, class DataT>
bool
change_field(FieldT &field, const DataT &value)
{
	bool rv = field != value;
	field   = value;
	return rv;
}

/** Set a string field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size>
bool
change_field(FieldT (&field)[Size], const char *value)
{
	bool change = ::strncmp(field, value, Size);
	::strncpy(field, value, Size - 1);
	field[Size - 1] = 0;
	return change;
}

/** Set an array field and return whether it changed
 * @param field The interface field to change
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size, class DataT>
typename std::enable_if<!std::is_same<FieldT, char>::value, bool>::type
change_field(FieldT (&field)[Size], const DataT *value)
{
	bool change = ::memcmp(field, value, Size);
	::memcpy(field, value, sizeof(FieldT) * Size);
	return change;
}

/** Set an array field value at a certain index and return whether it changed
 * @param field The interface field to change
 * @param index Index into the array field
 * @param value The new value
 * @return Whether the new value is different from the old
 */
template <class FieldT, std::size_t Size, class DataT>
bool
change_field(FieldT (&field)[Size], unsigned int index, const DataT &value)
{
	if (index >= Size)
		throw Exception("Index value %u is out of bounds (0..%u)", index, Size - 1);
	bool change  = field[index] != value;
	field[index] = value;
	return change;
}

} // end namespace fawkes

#endif
