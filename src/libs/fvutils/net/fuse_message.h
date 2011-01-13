
/***************************************************************************
 *  fuse_message.h - FireVision Remote Control Protocol Message Type
 *
 *  Created: Wed Nov 07 12:56:18 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_H_
#define __FIREVISION_FVUTILS_NET_FUSE_MESSAGE_H_

#include <core/utils/refcount.h>
#include <core/exceptions/software.h>
#include <fvutils/net/fuse.h>
#include <sys/types.h>
#include <cstdlib>
#include <cstring>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseMessageContent;

class FuseNetworkMessage : public fawkes::RefCount
{
 public:
  FuseNetworkMessage();
  FuseNetworkMessage(FUSE_message_t *msg);
  FuseNetworkMessage(FUSE_message_type_t type, void *payload, size_t payload_size,
		     bool copy_payload = false);
  FuseNetworkMessage(FUSE_message_type_t type, FuseMessageContent *content);
  FuseNetworkMessage(FUSE_message_type_t type);
  ~FuseNetworkMessage();

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
	throw fawkes::TypeMismatchException("FawkesNetworkMessage: message has incorrect size for this type");
      }
      return (MT *)(_msg.payload);
    }


  /** Get copy of correctly casted payload.
   * Use this method to cast the payload to a specific type. The size is
   * check as a sanity check and a TypeMismatchException is thrown if the
   * size does not match.
   * @return copy of casted message
   * @exception TypeMismatchException payload size does not match requested type
   */
  template <typename MT>
    MT *
    msg_copy() const
    {
      if ( payload_size() != sizeof(MT) ) {
	throw fawkes::TypeMismatchException("FawkesNetworkMessage: message has incorrect size for this type");
      }
      void *tmp = malloc(sizeof(MT));
      memcpy(tmp, _msg.payload, sizeof(MT));
      return (MT *)tmp;
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
      } catch (fawkes::Exception &e) {
	throw;
      } catch (...) {
	throw fawkes::Exception("Unknown exception caught while parsing complex network message");
      }
    }

  void pack();

  void set_payload(void *payload, size_t payload_size);
  void set(FUSE_message_t &msg);
  //void set_content(FuseComplexMessageContent *content);

 protected:
  /** Internal message. Fill in derivatives. */
  FUSE_message_t _msg;

 private:
  FuseMessageContent *__content;
};

} // end namespace firevision

#endif
