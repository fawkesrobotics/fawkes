
/***************************************************************************
 *  message_register.h - Protobuf stream protocol - message register
 *
 *  Created: Fri Feb 01 15:43:36 2013
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

#ifndef __PROTOBUF_COMM_MESSAGE_REGISTER_H_
#define __PROTOBUF_COMM_MESSAGE_REGISTER_H_

#include <protobuf_comm/frame_header.h>

#include <type_traits>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>

#include <map>
#include <cstdint>
#include <stdexcept>
#include <memory>
#include <limits>
#include <mutex>

namespace google {
  namespace protobuf {
    namespace compiler {
      class Importer;
      class DiskSourceTree;
    }
  }
}

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MessageRegister : boost::noncopyable
{
 public:
  MessageRegister();
  MessageRegister(std::vector<std::string> &proto_path);
  ~MessageRegister();

  void add_message_type(std::string msg_type);

  /** Add a new message type.
   * The template parameter must be a sub-class of google::protobuf::Message.
   * An instance is spawned and kept internally to spawn more on incoming messages.
   * @param component_id ID of component this message type belongs to
   * @param msg_type message type
   */
  template <class MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type
  add_message_type(uint16_t component_id, uint16_t msg_type)
  {
    KeyType key(component_id, msg_type);
    if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(component_id) + ":" +
	std::to_string(msg_type) + " already registered";
      throw std::runtime_error(msg);
    }
    MT *m = new MT();
    message_by_comp_type_[key] = m;
    message_by_typename_[m->GetDescriptor()->full_name()] = m;
  }

  /** Add a new message type.
   * The template parameter must be a sub-class of google::protobuf::Message.
   * An instance is spawned and kept internally to spawn more on incoming messages.
   */
  template <class MT>
  typename std::enable_if<std::is_base_of<google::protobuf::Message, MT>::value, void>::type
  add_message_type()
  {
    MT m;
    const google::protobuf::Descriptor *desc = m.GetDescriptor();
    KeyType key = key_from_desc(desc);
    if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" +
	std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    MT *new_m = new MT();
    message_by_comp_type_[key] = new_m;
    message_by_typename_[new_m->GetTypeName()] = new_m;
  }

  void remove_message_type(uint16_t component_id, uint16_t msg_type);

  std::shared_ptr<google::protobuf::Message>
  new_message_for(uint16_t component_id, uint16_t msg_type);

  std::shared_ptr<google::protobuf::Message>
  new_message_for(std::string &full_name);

  void serialize(uint16_t component_id, uint16_t msg_type,
		 google::protobuf::Message &msg,
		 frame_header_t &frame_header,
		 message_header_t &message_header, 
		 std::string &data);
  std::shared_ptr<google::protobuf::Message>
  deserialize(frame_header_t &frame_header,
	      message_header_t &message_header,
	      void *data);

  /** Mapping from message type to load error message. */
  typedef std::multimap<std::string, std::string> LoadFailMap;

  /** Get failure messages from loading.
   * If the proto path constructor is used this function returns a list
   * of loading errors after construction.
   * @return map of loading failures
   */
  const LoadFailMap &  load_failures() const
  { return failed_to_load_types_; }

 private: // members
  typedef std::pair<uint16_t, uint16_t> KeyType;
  typedef std::map<KeyType, google::protobuf::Message *> TypeMap;
  typedef std::map<std::string, google::protobuf::Message *> TypeNameMap;

  KeyType key_from_desc(const google::protobuf::Descriptor *desc);
  google::protobuf::Message * create_msg(std::string &msg_type);

  std::mutex maps_mutex_;
  TypeMap message_by_comp_type_;
  TypeNameMap message_by_typename_;

  google::protobuf::compiler::DiskSourceTree  *pb_srctree_;
  google::protobuf::compiler::Importer        *pb_importer_;
  google::protobuf::MessageFactory            *pb_factory_;
  std::multimap<std::string, std::string> failed_to_load_types_;
};

} // end namespace protobuf_comm


#endif
