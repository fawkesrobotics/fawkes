
/***************************************************************************
 *  queue_entry.h - Protobuf stream protocol - send queue entry
 *
 *  Created: Fri Feb 01 22:07:14 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PROTOBUF_COMM_QUEUE_ENTRY_H_
#define __PROTOBUF_COMM_QUEUE_ENTRY_H_

#include <boost/asio.hpp>
#include <array>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Outgoing queue entry. */
struct QueueEntry {
public:
  /** Constructor. */
  QueueEntry()
  {
    frame_header.header_version = PB_FRAME_V2;
    frame_header.cipher         = PB_ENCRYPTION_NONE;
  };
  std::string  serialized_message;	///< serialized protobuf message
  frame_header_t    frame_header;	///< Frame header (network byte order), never encrypted
  frame_header_v1_t frame_header_v1;	///< Frame header (network byte order), never encrypted
  message_header_t message_header;		///< Frame header (network byte order)
  std::array<boost::asio::const_buffer, 3> buffers;	///< outgoing buffers
  std::string   encrypted_message;	///< encrypted buffer if encryption is used
};


} // end namespace protobuf_comm

#endif
