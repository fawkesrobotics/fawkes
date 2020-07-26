
/***************************************************************************
 *  net_messages.h - BlackBoard Network Messages
 *
 *  Created: Sat Mar 01 16:08:13 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef _BLACKBOARD_NET_MESSAGES_H_
#define _BLACKBOARD_NET_MESSAGES_H_

#include <interface/interface.h>
#include <netcomm/utils/dynamic_buffer.h>

#include <stdint.h>

namespace fawkes {

#pragma pack(push, 4)

/** BlackBoard network message types */
typedef enum {
	MSG_BB_LIST_ALL = 0,
	MSG_BB_INTERFACE_LIST,
	MSG_BB_OPEN_FOR_READING,
	MSG_BB_OPEN_FOR_WRITING,
	MSG_BB_OPEN_SUCCESS,
	MSG_BB_OPEN_FAILURE,
	MSG_BB_CLOSE,
	MSG_BB_WRITE,
	MSG_BB_INTERFACE_MESSAGE,
	MSG_BB_DATA_REFRESHED,
	MSG_BB_DATA_CHANGED,
	MSG_BB_READER_ADDED,
	MSG_BB_READER_REMOVED,
	MSG_BB_WRITER_ADDED,
	MSG_BB_WRITER_REMOVED,
	MSG_BB_INTERFACE_CREATED,
	MSG_BB_INTERFACE_DESTROYED,
	MSG_BB_LIST
} blackboard_msgid_t;

/** Error codes */
typedef enum {
	BB_ERR_UNKNOWN_ERR,   /**< Unknown error occured. Check log. */
	BB_ERR_UNKNOWN_TYPE,  /**< Requested interface type is unknown. */
	BB_ERR_HASH_MISMATCH, /**< The hashes of the interfaces do not match. Make sure that
			 * both sides are using the exact same version of the interface. */
	BB_ERR_WRITER_EXISTS  /**< You tried to open an interface for writing but there is already
			 * a writing instance for this interface. */
} blackboard_neterror_t;

/** Message to transport a list of interfaces. */
typedef struct
{
	dynamic_list_t interface_list; /**< dynamic buffer list with interface info */
} bb_ilist_msg_t;

/** Message to request constrained interface list. */
typedef struct
{
	char type_pattern[INTERFACE_TYPE_SIZE_]; /**< type pattern */
	char id_pattern[INTERFACE_ID_SIZE_];     /**< ID pattern */
} bb_ilistreq_msg_t;

/** Message to identify an interface on open. */
typedef struct
{
	char          type[INTERFACE_TYPE_SIZE_]; /**< interface type name */
	char          id[INTERFACE_ID_SIZE_];     /**< interface instance ID */
	unsigned char hash[INTERFACE_HASH_SIZE_]; /**< interface version hash */
} bb_iopen_msg_t;

/** Message for interface info. */
typedef struct
{
	char          type[INTERFACE_TYPE_SIZE_]; /**< interface type name */
	char          id[INTERFACE_ID_SIZE_];     /**< interface instance ID */
	unsigned char hash[INTERFACE_HASH_SIZE_]; /**< interface version hash */
	uint32_t      serial;                     /**< instance serial to uniquely identify
						 * this instance (big endian) */
	uint32_t      writer_readers;             /**< combined writer reader
						 * information. First bit (any endian) is
						   1 if writer exists, 0 otherwise. The
						   remaining 31 bits encode the number
						   of readers as big endian number. */
	int64_t       timestamp_sec;              /**< data or write timestamp, sec part */
	int64_t       timestamp_usec;             /**< data or write timestamp, usec part */
} bb_iinfo_msg_t;

/** Message for interface events.
 * This message is used for MSG_BB_INTERFACE_CREATED and MSG_BB_INTERFACE_REMOVED.
 */
typedef struct
{
	char type[INTERFACE_TYPE_SIZE_]; /**< interface type name */
	char id[INTERFACE_ID_SIZE_];     /**< interface instance ID */
} bb_ievent_msg_t;

/** Message to identify an interface instance.
 * This message is used for MSG_BB_CLOSE, MSG_BB_READER_ADDED, MSG_BB_READER_REMOVED,
 * MSG_BB_WRITER_ADDED, and MSG_BB_READER_REMOVED.
 */
typedef struct
{
	uint32_t serial; /**< instance serial to unique identify this instance */
} bb_iserial_msg_t;

/** Message to identify an two interface instances.
 * This message is used for MSG_BB_READER_ADDED, MSG_BB_READER_REMOVED,
 * MSG_BB_WRITER_ADDED, and MSG_BB_READER_REMOVED.
 */
typedef struct
{
	uint32_t serial;       /**< instance serial to unique identify own instance */
	uint32_t event_serial; /**< instance serial to unique identify instance that
				 * caused the event. */
} bb_ieventserial_msg_t;

/** Interface open success
 * The serial denotes a unique instance of an interface within the (remote)
 * BlackBoard.
 * This message struct is always followed by a data chunk that is of the
 * size data_size. It contains the current content of the interface.
 */
typedef struct
{
	uint32_t serial;         /**< instance serial to unique identify this instance */
	uint32_t writer_readers; /**< combined writer reader information. First
				 * bit (any endian) is 1 if writer exists, 0 otherwise.
				 * The remaining 31 bits encode the number of readers
				 * as big endian number. */
	uint32_t data_size;      /**< size in bytes of the following data. */
} bb_iopensucc_msg_t;

/** Message to send update data. */
typedef struct
{
	uint32_t error_code; /**< Error code. @see blackboard_neterror_t */
} bb_iopenfail_msg_t;

/** Interface data message.
 * The serial denotes a unique instance of an interface within the (remote)
 * BlackBoard.
 * This message struct is always followed by a data chunk that is of the
 * size data_size. It contains the current content of the interface.
 * This message is sent for MSG_BB_WRITE and MSG_BB_DATA_CHANGED.
 */
typedef struct
{
	uint32_t serial;    /**< instance serial to unique identify this instance */
	uint32_t data_size; /**< size in bytes of the following data. */
} bb_idata_msg_t;

/** Interface message.
 * This type is used to transport interface messages. This struct is always followed
 * by a data chunk of the size data_size that transports the message data.
 */
typedef struct
{
	uint32_t serial;                                 /**< interface instance serial */
	char     msg_type[INTERFACE_MESSAGE_TYPE_SIZE_]; /**< message type */
	uint32_t msgid;                                  /**< message ID */
	uint32_t hops;      /**< number of hops this message already passed */
	uint32_t data_size; /**< data for message */
} bb_imessage_msg_t;

#pragma pack(pop)

} // end namespace fawkes

#endif
