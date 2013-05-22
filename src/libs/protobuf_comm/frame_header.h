
/***************************************************************************
 *  frame_header.h - Basic framing header or each message
 *
 *  Created: Mon Jan 21 12:05:03 2013
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

#ifndef __PROTOBUF_COMM_FRAME_HEADER_H_
#define __PROTOBUF_COMM_FRAME_HEADER_H_

#include <cstdint>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#pragma pack(push,4)

/** Network message framing header.
 * Header that is prepended to all messages.
 * The component ID can be used to route a message to a particular
 * software component. The component then can use the message type to
 * determine how the message must be parse the payload. It is appended
 * immediately following the header. The payload size does not include
 * the size of the header.
 * All numbers are given in network byte order (big endian).
 * @author Tim Niemueller
 */
typedef struct {
  /** component id */
  uint16_t  component_id;
  /** message type */
  uint16_t  msg_type;
  /** payload size in bytes */
  uint32_t  payload_size;
} frame_header_t;

#pragma pack(pop)

} // end namespace protobuf_comm

#endif
