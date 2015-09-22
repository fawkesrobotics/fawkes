
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

#ifndef __BLACKBOARD_NET_MESSAGES_H_
#define __BLACKBOARD_NET_MESSAGES_H_

#include <stdint.h>
#include <netcomm/utils/dynamic_buffer.h>
#include <interface/interface.h>

namespace fawkes {

#pragma pack(push,4)

/** BlackBoard network message types */
typedef enum {
  MSG_BB_LIST_ALL		=  0,
  MSG_BB_INTERFACE_LIST		=  1,
  MSG_BB_OPEN_FOR_READING	=  2,
  MSG_BB_OPEN_FOR_WRITING	=  3,
  MSG_BB_OPEN_SUCCESS		=  4,
  MSG_BB_OPEN_FAILURE		=  5,
  MSG_BB_CLOSE			=  6,
  MSG_BB_WRITE			=  7,
  MSG_BB_INTERFACE_MESSAGE	=  8,
  MSG_BB_DATA_CHANGED		=  9,
  MSG_BB_READER_ADDED		= 10,
  MSG_BB_READER_REMOVED		= 11,
  MSG_BB_WRITER_ADDED		= 12,
  MSG_BB_WRITER_REMOVED		= 13,
  MSG_BB_INTERFACE_CREATED	= 14,
  MSG_BB_INTERFACE_DESTROYED	= 15,
  MSG_BB_LIST			= 16
} blackboard_msgid_t;

/** Error codes */
typedef enum {
  BB_ERR_UNKNOWN_ERR,	/**< Unknown error occured. Check log. */
  BB_ERR_UNKNOWN_TYPE,	/**< Requested interface type is unknown. */
  BB_ERR_HASH_MISMATCH,	/**< The hashes of the interfaces do not match. Make sure that
			 * both sides are using the exact same version of the interface. */
  BB_ERR_WRITER_EXISTS	/**< You tried to open an interface for writing but there is already
			 * a writing instance for this interface. */
} blackboard_neterror_t;

/** Message to transport a list of interfaces. */
typedef struct {
  dynamic_list_t interface_list;	/**< dynamic buffer list with interface info */
} bb_ilist_msg_t;

/** Message to request constrained interface list. */
typedef struct {
  char          type_pattern[__INTERFACE_TYPE_SIZE];	/**< type pattern */
  char          id_pattern[__INTERFACE_ID_SIZE];	/**< ID pattern */
} bb_ilistreq_msg_t;

/** Message to identify an interface on open. */
typedef struct {
  char          type[__INTERFACE_TYPE_SIZE];	/**< interface type name */
  char          id[__INTERFACE_ID_SIZE];	/**< interface instance ID */
  unsigned char hash[__INTERFACE_HASH_SIZE];	/**< interface version hash */
} bb_iopen_msg_t;


/** Message for interface info. */
typedef struct {
  char          type[__INTERFACE_TYPE_SIZE];	/**< interface type name */
  char          id[__INTERFACE_ID_SIZE];	/**< interface instance ID */
  unsigned char hash[__INTERFACE_HASH_SIZE];	/**< interface version hash */
  uint32_t      serial;				/**< instance serial to uniquely identify
						 * this instance (big endian) */
  uint32_t      writer_readers;			/**< combined writer reader
						 * information. First bit (any endian) is
						   1 if writer exists, 0 otherwise. The
						   remaining 31 bits encode the number
						   of readers as big endian number. */
  int64_t       timestamp_sec;			/**< data or write timestamp, sec part */
  int64_t       timestamp_usec;			/**< data or write timestamp, usec part */
} bb_iinfo_msg_t;


/** Message for interface events.
 * This message is used for MSG_BB_INTERFACE_CREATED and MSG_BB_INTERFACE_REMOVED.
 */
typedef struct {
  char          type[__INTERFACE_TYPE_SIZE];	/**< interface type name */
  char          id[__INTERFACE_ID_SIZE];	/**< interface instance ID */
} bb_ievent_msg_t;


/** Message to identify an interface instance.
 * This message is used for MSG_BB_CLOSE, MSG_BB_READER_ADDED, MSG_BB_READER_REMOVED,
 * MSG_BB_WRITER_ADDED, and MSG_BB_READER_REMOVED.
 */
typedef struct {
  uint32_t  serial;	/**< instance serial to unique identify this instance */
} bb_iserial_msg_t;


/** Message to identify an two interface instances.
 * This message is used for MSG_BB_READER_ADDED, MSG_BB_READER_REMOVED,
 * MSG_BB_WRITER_ADDED, and MSG_BB_READER_REMOVED.
 */
typedef struct {
  uint32_t  serial;		/**< instance serial to unique identify own instance */
  uint32_t  event_serial;	/**< instance serial to unique identify instance that
				 * caused the event. */
} bb_ieventserial_msg_t;


/** Interface open success
 * The serial denotes a unique instance of an interface within the (remote)
 * BlackBoard.
 * This message struct is always followed by a data chunk that is of the
 * size data_size. It contains the current content of the interface.
 */
typedef struct {
  uint32_t serial;		/**< instance serial to unique identify this instance */
  uint32_t writer_readers;	/**< combined writer reader information. First
				 * bit (any endian) is 1 if writer exists, 0 otherwise.
				 * The remaining 31 bits encode the number of readers
				 * as big endian number. */
  uint32_t data_size;		/**< size in bytes of the following data. */
} bb_iopensucc_msg_t;


/** Message to send update data. */
typedef struct {
  uint32_t  error_code;	/**< Error code. @see blackboard_neterror_t */
} bb_iopenfail_msg_t;


/** Interface data message.
 * The serial denotes a unique instance of an interface within the (remote)
 * BlackBoard.
 * This message struct is always followed by a data chunk that is of the
 * size data_size. It contains the current content of the interface.
 * This message is sent for MSG_BB_WRITE and MSG_BB_DATA_CHANGED.
 */
typedef struct {
  uint32_t serial;	/**< instance serial to unique identify this instance */
  uint32_t data_size;	/**< size in bytes of the following data. */
} bb_idata_msg_t;


/** Interface message.
 * This type is used to transport interface messages. This struct is always followed
 * by a data chunk of the size data_size that transports the message data.
 */
 typedef struct {
  uint32_t serial;					/**< interface instance serial */
  char     msg_type[__INTERFACE_MESSAGE_TYPE_SIZE];	/**< message type */
  uint32_t msgid;					/**< message ID */
  uint32_t hops;					/**< number of hops this message already passed */
  uint32_t data_size;					/**< data for message */
} bb_imessage_msg_t;

#pragma pack(pop)

} // end namespace fawkes

#endif
