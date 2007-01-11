
/***************************************************************************
 *  message.h - Fawkes network message
 *
 *  Created: Mon Nov 20 18:00:09 2006
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

#ifndef __NETCOMM_FAWKES_MESSAGE_H_
#define __NETCOMM_FAWKES_MESSAGE_H_

#include <core/utils/refcount.h>
#include <core/exceptions/software.h>

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


class FawkesNetworkMessage : public RefCount
{
 public:
  FawkesNetworkMessage(unsigned int clid, fawkes_message_t &msg);
  FawkesNetworkMessage(fawkes_message_t &msg);
  FawkesNetworkMessage(unsigned int clid,
		       unsigned short int cid, unsigned short int msg_id,
		       void *payload, unsigned int payload_size);
  FawkesNetworkMessage(unsigned int clid,
		       unsigned short int cid, unsigned short int msg_id);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
		       void *payload, unsigned int payload_size);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
		       unsigned int payload_size);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id);
  FawkesNetworkMessage();

  virtual ~FawkesNetworkMessage();

  unsigned int       clid() const;
  unsigned short int cid() const;
  unsigned short int msgid() const;
  unsigned int       payload_size() const;
  void *             payload() const;
  const fawkes_message_t & fmsg() const;

  /** Get correctly casted payload.
   * Use this method to cast the payload to a specific type. The size is
   * check as a sanity check and a TypeMismatchException is thrown if the
   * size does not match.
   * @return casted message
   * @exception TypeMismatchException payload size does not match requested type
   */
  template <typename MT>
    MT *
    msg() const
    {
      if ( payload_size() != sizeof(MT) ) {
	throw TypeMismatchException("FawkesNetworkMessage: message has incorrect size for this type");
      }
      return (MT *)(_msg.payload);
    }

  void setClientID(unsigned int clid);
  void setComponentID(unsigned short int cid);
  void setMessageID(unsigned short int msg_id);
  void setPayload(void *payload, unsigned int payload_size);
  void set(fawkes_message_t &msg);

 private:
  void init_cid_msgid(unsigned short int cid, unsigned short int msg_id);
  void init_payload(unsigned int payload_size);

  unsigned int _clid;
  fawkes_message_t _msg;
};

#endif
