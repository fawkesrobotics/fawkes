
/***************************************************************************
 *  message.h - Fawkes network message
 *
 *  Created: Mon Nov 20 18:00:09 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_FAWKES_MESSAGE_H_
#define __NETCOMM_FAWKES_MESSAGE_H_

#include <core/utils/refcount.h>
#include <core/exceptions/software.h>

#include <cstddef>

namespace fawkes {

#pragma pack(push,4)

/** Fawkes network message header.
 * Header that is prepended to all following messages.
 */
typedef struct {
  unsigned short int  cid;		/**< component id */
  unsigned short int  msg_id;		/**< message id */
  unsigned int        payload_size;	/**< payload size in bytes */
} fawkes_message_header_t;

#pragma pack(pop)

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


class FawkesNetworkMessageTooBigException : public Exception
{
 public:
  FawkesNetworkMessageTooBigException(size_t message_size);
};

class FawkesNetworkMessageContent;

class FawkesNetworkMessage : public RefCount
{
 public:
  FawkesNetworkMessage(unsigned int clid, fawkes_message_t &msg);
  FawkesNetworkMessage(fawkes_message_t &msg);
  FawkesNetworkMessage(unsigned int clid,
		       unsigned short int cid, unsigned short int msg_id,
		       void *payload, size_t payload_size);
  FawkesNetworkMessage(unsigned int clid,
		       unsigned short int cid, unsigned short int msg_id);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
		       void *payload, size_t payload_size);
  FawkesNetworkMessage(unsigned int clid,
		       unsigned short int cid, unsigned short int msg_id,
		       FawkesNetworkMessageContent *content);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
		       FawkesNetworkMessageContent *content);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id,
		       size_t payload_size);
  FawkesNetworkMessage(unsigned short int cid, unsigned short int msg_id);
  FawkesNetworkMessage();

  virtual ~FawkesNetworkMessage();

  unsigned int       clid() const;
  unsigned short int cid() const;
  unsigned short int msgid() const;
  size_t             payload_size() const;
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

  /** Get correctly casted payload.
   * Use this method to cast the payload to a specific type. The size is
   * check as a sanity check and a TypeMismatchException is thrown if the
   * size does not match. The size of the received message must be greater or
   * equal to the size of the message type. Useful if message contains a variable
   * length string.
   * @return casted message
   * @exception TypeMismatchException payload size does not match requested type
   */
  template <typename MT>
    MT *
    msgge() const
    {
      if ( payload_size() < sizeof(MT) ) {
	throw TypeMismatchException("FawkesNetworkMessage: message has incorrect size for this type");
      }
      return (MT *)(_msg.payload);
    }

  /** Get correctly parsed output.
   * Use this method to cast the payload to a specific complex type. You can use this
   * routine to parse complex messages that are derived from FawkesNetworkMessageContent.
   * Note that the class must provide a constructor that takes four parameters: The
   * component ID, message ID, a pointer to the payload and the payload size. From this
   * the class shall parse  the output and throw an exception if that for whatever
   * reason fails.
   * @return casted message
   * @exception TypeMismatchException payload size does not match requested type
   */
  template <typename MT>
    MT *
    msgc() const
    {
      try {
	MT *m = new MT(cid(), msgid(), _msg.payload, payload_size());
	return m;
      } catch (Exception &e) {
	throw;
      } catch (...) {
	throw Exception("Unknown exception caught while parsing complex network message");
      }
    }

  void set_client_id(unsigned int clid);
  void set_component_id(unsigned short int cid);
  void set_message_id(unsigned short int msg_id);
  void set_payload(void *payload, size_t payload_size);
  void set(fawkes_message_t &msg);
  void set_content(FawkesNetworkMessageContent *content);

  void pack();

 private:
  void init_cid_msgid(unsigned short int cid, unsigned short int msg_id);
  void init_payload(size_t payload_size);

  unsigned int _clid;
  fawkes_message_t _msg;

  FawkesNetworkMessageContent *_content;
};

} // end namespace fawkes

#endif
