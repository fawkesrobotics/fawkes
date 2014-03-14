
/***************************************************************************
 *  message_register.cpp - Protobuf stream protocol - message register
 *
 *  Created: Fri Feb 01 15:48:36 2013
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

#include <protobuf_comm/message_register.h>

#include <google/protobuf/compiler/importer.h>
#include <google/protobuf/dynamic_message.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <dirent.h>
#include <fnmatch.h>

namespace protobuf_comm {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class MessageRegister <protobuf_comm/message_register.h>
 * Register to map msg type numbers to Protobuf messages.
 * The register is used to automatically parse incoming messages to the
 * appropriate type. In your application, you need to register any
 * message you want to read. All unknown messages are silently dropped.
 * @author Tim Niemueller
 */

/** Constructor. */
MessageRegister::MessageRegister()
{
  pb_srctree_  = NULL;
  pb_importer_ = NULL;
  pb_factory_  = NULL;
}


/** Constructor.
 * @param proto_path file paths to search for proto files. All message types
 * within these files will automatically be registered and available for dynamic
 * message creation.
 */
MessageRegister::MessageRegister(std::vector<std::string> &proto_path)
{
  pb_srctree_ = new google::protobuf::compiler::DiskSourceTree();
  for (size_t i = 0; i < proto_path.size(); ++i) {
    pb_srctree_->MapPath("", proto_path[i]);
  }
  pb_importer_ = new google::protobuf::compiler::Importer(pb_srctree_, NULL);
  pb_factory_  = new google::protobuf::DynamicMessageFactory(pb_importer_->pool());

  for (size_t i = 0; i < proto_path.size(); ++i) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(proto_path[i].c_str())) != NULL) {
      while ((ent = readdir(dir)) != NULL) {
	if (fnmatch("*.proto", ent->d_name, FNM_PATHNAME) != FNM_NOMATCH) {
	  //printf ("%s\n", ent->d_name);
	  const google::protobuf::FileDescriptor *fd =
	    pb_importer_->Import(ent->d_name);
	  for (int i = 0; i < fd->message_type_count(); ++i) {
	    const google::protobuf::Descriptor *desc = fd->message_type(i);
	    //printf("  Type: %s\n", desc->full_name().c_str());
	    if (! desc->FindEnumTypeByName("CompType")) continue;

	    try {
	      add_message_type(desc->full_name());
	    } catch (std::logic_error &e) {
	      // cannot open for some reason
	      failed_to_load_types_.insert(std::make_pair(desc->full_name(), e.what()));
	    }
	  }
	}
      }
      closedir (dir);
    }
  }
}

/** Destructor. */
MessageRegister::~MessageRegister()
{
  TypeMap::iterator m;
  for (m = message_by_comp_type_.begin(); m != message_by_comp_type_.end(); ++m) {
    delete m->second;
  }
  delete pb_factory_;
  delete pb_importer_;
  delete pb_srctree_;
}


google::protobuf::Message *
MessageRegister::create_msg(std::string &msg_type)
{
  const google::protobuf::DescriptorPool *pool =
    google::protobuf::DescriptorPool::generated_pool();
  google::protobuf::MessageFactory *factory =
    google::protobuf::MessageFactory::generated_factory();

  const google::protobuf::Descriptor *desc = pool->FindMessageTypeByName(msg_type);
  if (desc) {
    return factory->GetPrototype(desc)->New();
  } else if (pb_importer_) {
    pool = pb_importer_->pool();
    factory = pb_factory_;

    const google::protobuf::Descriptor *cdesc = pool->FindMessageTypeByName(msg_type);
    if (cdesc) {
      return factory->GetPrototype(cdesc)->New();
    }
  }
  return NULL;
}

/** Add a message type from generated pool.
 * This will check all message libraries for a type of the given name
 * and if found registers it.
 * @param msg_type the full name of the message type to add, i.e. including
 * a package name if the message type has one. The message must have been
 * registered with either the generated messages pool or with the pool
 * associated with the proto paths passed to the constructor.
 */
void
MessageRegister::add_message_type(std::string msg_type)
{
  google::protobuf::Message *m = create_msg(msg_type);
  if (m) {
    KeyType key = key_from_desc(m->GetDescriptor());
    std::lock_guard<std::mutex> lock(maps_mutex_);
    if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
      std::string msg = "Message type " + std::to_string(key.first) + ":" +
	std::to_string(key.second) + " already registered";
      throw std::runtime_error(msg);
    }
    //printf("Registering %s (%u:%u)\n", msg_type.c_str(), key.first, key.second);
    message_by_comp_type_[key] = m;
    message_by_typename_[m->GetTypeName()] = m;
  } else {
    throw std::runtime_error("Unknown message type");
  }
}


/** Remove the given message type.
 * @param component_id ID of component this message type belongs to
 * @param msg_type message type
 */
void
MessageRegister::remove_message_type(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);
  std::lock_guard<std::mutex> lock(maps_mutex_);
  if (message_by_comp_type_.find(key) != message_by_comp_type_.end()) {
    message_by_typename_.erase(message_by_comp_type_[key]->GetDescriptor()->full_name());
    message_by_comp_type_.erase(key);
  }
}


MessageRegister::KeyType
MessageRegister::key_from_desc(const google::protobuf::Descriptor *desc)
{
  const google::protobuf::EnumDescriptor *enumdesc = desc->FindEnumTypeByName("CompType");
  if (! enumdesc) {
    throw std::logic_error("Message does not have CompType enum");
  }
  const google::protobuf::EnumValueDescriptor *compdesc =
    enumdesc->FindValueByName("COMP_ID");
  const google::protobuf::EnumValueDescriptor *msgtdesc =
    enumdesc->FindValueByName("MSG_TYPE");
  if (! compdesc || ! msgtdesc) {
    throw std::logic_error("Message CompType enum hs no COMP_ID or MSG_TYPE value");
  }
  int comp_id = compdesc->number();
  int msg_type = msgtdesc->number();
  if (comp_id < 0 || comp_id > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid COMP_ID");
  }
  if (msg_type < 0 || msg_type > std::numeric_limits<uint16_t>::max()) {
    throw std::logic_error("Message has invalid MSG_TYPE");
  }
  return KeyType(comp_id, msg_type);
}

/** Create a new message instance.
 * @param component_id ID of component this message type belongs to
 * @param msg_type message type
 * @return new instance of a protobuf message that has been registered
 * for the given message type.
 */
std::shared_ptr<google::protobuf::Message>
MessageRegister::new_message_for(uint16_t component_id, uint16_t msg_type)
{
  KeyType key(component_id, msg_type);

  std::lock_guard<std::mutex> lock(maps_mutex_);
  if (message_by_comp_type_.find(key) == message_by_comp_type_.end()) {
    std::string msg = "Message type " + std::to_string(component_id) + ":" +
      std::to_string(msg_type) + " not registered";
    throw std::runtime_error(msg);
  }

  google::protobuf::Message *m = message_by_comp_type_[key]->New();
  return std::shared_ptr<google::protobuf::Message>(m);
}


/** Create a new message instance.
 * @param full_name full message type name, i.e. the message type name
 * possibly with a package name prefix.
 * @return new instance of a protobuf message that has been registered
 * for the given message type.
 */
std::shared_ptr<google::protobuf::Message>
MessageRegister::new_message_for(std::string &full_name)
{
  std::lock_guard<std::mutex> lock(maps_mutex_);
  if (message_by_typename_.find(full_name) == message_by_typename_.end()) {
    google::protobuf::Message *m = create_msg(full_name);
    if (m) {
      return std::shared_ptr<google::protobuf::Message>(m);
    } else {
      throw std::runtime_error("Message type not registered");
    }
  } else {
    google::protobuf::Message *m = message_by_typename_[full_name]->New();
    return std::shared_ptr<google::protobuf::Message>(m);
  }
}


/** Serialize a message.
 * @param component_id ID of component this message type belongs to
 * @param msg_type message type
 * @param msg message to seialize
 * @param frame_header upon return, the frame header is filled out according to
 * the given information and message.
 * @param message_header upon return, the frame header is filled out according to
 * the given information and message.
 * @param data upon return, contains the serialized message
 */ 
void
MessageRegister::serialize(uint16_t component_id, uint16_t msg_type,
			   google::protobuf::Message &msg,
			   frame_header_t &frame_header,
			   message_header_t &message_header, 
			   std::string &data)
{
  if (msg.SerializeToString(&data)) {
    message_header.component_id = htons(component_id);
    message_header.msg_type     = htons(msg_type);
    frame_header.payload_size   = htonl(sizeof(message_header) + data.size());
  } else {
    throw std::runtime_error("Cannot serialize message");
  }
}


/** Deserialize message.
 * @param frame_header incoming message's frame header
 * @param message_header incoming message's message header
 * @param data incoming message's data buffer
 * @return new instance of a protobuf message type that has been registered
 * for the given type.
 * @exception std::runtime_error thrown if anything goes wrong when
 * deserializing the message, e.g. if no protobuf message has been registered
 * for the given component ID and message type.
 */
std::shared_ptr<google::protobuf::Message>
MessageRegister::deserialize(frame_header_t &frame_header, message_header_t &message_header, void *data)
{
  uint16_t comp_id   = ntohs(message_header.component_id);
  uint16_t msg_type  = ntohs(message_header.msg_type);
  size_t   data_size = ntohl(frame_header.payload_size) - sizeof(message_header);

  std::shared_ptr<google::protobuf::Message> m =
    new_message_for(comp_id, msg_type);
  if (! m->ParseFromArray(data, data_size)) {
    throw std::runtime_error("Failed to parse message");
  }

  return m;
}

} // end namespace protobuf_comm
