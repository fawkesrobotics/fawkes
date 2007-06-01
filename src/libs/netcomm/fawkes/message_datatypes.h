
/***************************************************************************
 *  message_datatypes.h - Fawkes network message data types
 *
 *  Created: Fri Jun 01 14:25:27 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_FAWKES_MESSAGE_DATATYPES_H_
#define __NETCOMM_FAWKES_MESSAGE_DATATYPES_H_

#include <stdint.h>

/** Dynamic list type.
 * Use this element in your message struct if you want to add a dynamic list.
 * This is meant to be used in conjunction with DynamicBuffer.
 */
typedef struct {
  uint16_t  size;		/**< total size of list buffer */
  uint16_t  num_elements;	/**< number of elements in list */
} dynamic_list_t;

/** Fawkes network message header.
 * Header that is prepended to all following messages.
 */
typedef struct {
  unsigned short int  cid;		/**< component id */
  unsigned short int  msg_id;		/**< message id */
  unsigned int        payload_size;	/**< payload size in bytes */
} fawkes_message_header_t;


/** Message as stored in local queues.
 * A message takes a header and a pointer to the data that
 * has the size mentioned in header.payload_size that is to be
 * sent over the network.
 */
typedef struct {
  fawkes_message_header_t  header;	/**< message header */
  void                    *payload;	/**< message payload */
} fawkes_message_t;


/** Fawkes transfer header.
 * This header is prepended to a collection of messages that is sent
 * at once.
 */
typedef struct {
  unsigned int  size;	/**< size of the following payload. */
} fawkes_transfer_header_t;

#endif
