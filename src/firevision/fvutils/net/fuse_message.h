
/***************************************************************************
 *  fuse_message.h - FireVision Remote Control Protocol Message Type
 *
 *  Created: Wed Nov 07 12:56:18 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_H_

#include <core/utils/refcount.h>
#include <core/exceptions/software.h>
#include <fvutils/net/fuse.h>
#include <sys/types.h>

//class FuseComplexMessageContent;

class FuseNetworkMessage : public RefCount
{
 public:
  FuseNetworkMessage();
  FuseNetworkMessage(FUSE_message_t *msg);
  //FuseNetworkMessage(FuseComplexMessageContent *content);
  FuseNetworkMessage(FUSE_message_type_t type, void *payload, size_t payload_size,
		     bool copy_payload = false);
  FuseNetworkMessage(FUSE_message_type_t type);
  virtual ~FuseNetworkMessage();

  virtual void pack();

  uint32_t  type() const;
  size_t    payload_size() const;
  void *    payload() const;

  const FUSE_message_t &  fmsg() const;

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

  /** Get correctly parsed output.
   * Use this method to cast the payload to a specific complex type. You can use this
   * routine to parse complex messages that are derived from FuseComplexMessageContent.
   * Note that the class must provide a constructor that takes three parameters: The
   * message type, a pointer to the payload and the payload size. From this
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
	MT *m = new MT(type(), _msg.payload, payload_size());
	return m;
      } catch (Exception &e) {
	throw;
      } catch (...) {
	throw Exception("Unknown exception caught while parsing complex network message");
      }
    }

  void set_payload(void *payload, size_t payload_size);
  void set(FUSE_message_t &msg);
  //void set_content(FuseComplexMessageContent *content);

 protected:
  void copy_payload(size_t offset, void *buf, size_t len);

  /** Internal message. Fill in derivatives. */
  FUSE_message_t _msg;
};

#endif
