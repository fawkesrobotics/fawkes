
/***************************************************************************
 *  communicator.cpp - protobuf network communication for CLIPS
 *
 *  Created: Tue Apr 16 13:51:14 2013
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

#include <protobuf_clips/communicator.h>

#include <core/threading/mutex_locker.h>
#include <logging/logger.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/server.h>
#include <protobuf_comm/peer.h>

#include <google/protobuf/descriptor.h>

#include <boost/format.hpp>

using namespace google::protobuf;
using namespace protobuf_comm;

namespace protobuf_clips {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ClipsProtobufCommunicator <protobuf_clips/communicator.h>
 * CLIPS protobuf integration class.
 * This class adds functionality related to protobuf to a given CLIPS
 * environment. It supports the creation of communication channels
 * through protobuf_comm. An instance maintains its own message register
 * shared among server, peer, and clients.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param env CLIPS environment to which to provide the protobuf functionality
 * @param env_mutex mutex to lock when operating on the CLIPS environment.
 * @param logger optional logger for informational output
 */
ClipsProtobufCommunicator::ClipsProtobufCommunicator(CLIPS::Environment *env,
						     fawkes::Mutex &env_mutex,
						     fawkes::Logger *logger)
  : clips_(env), clips_mutex_(env_mutex), logger_(logger), server_(NULL),
    next_client_id_(0)
{
  message_register_ = new MessageRegister();
  setup_clips();
}

/** Constructor.
 * @param env CLIPS environment to which to provide the protobuf functionality
 * @param env_mutex mutex to lock when operating on the CLIPS environment.
 * @param proto_path proto path passed to a newly instantiated message register
 * @param logger optional logger for informational output
 */
ClipsProtobufCommunicator::ClipsProtobufCommunicator(CLIPS::Environment *env,
						     fawkes::Mutex &env_mutex,
						     std::vector<std::string> &proto_path,
						     fawkes::Logger *logger)
  : clips_(env), clips_mutex_(env_mutex), logger_(logger), server_(NULL),
    next_client_id_(0)
{
  message_register_ = new MessageRegister(proto_path);
  setup_clips();
}


/** Destructor. */
ClipsProtobufCommunicator::~ClipsProtobufCommunicator()
{
  {
    fawkes::MutexLocker lock(&clips_mutex_);

    for (auto f : functions_) {
      clips_->remove_function(f);
    }
    functions_.clear();
  }

  for (auto c : clients_) {
    delete c.second;
  }
  clients_.clear();

  delete message_register_;
  delete server_;
}


#define ADD_FUNCTION(n, s)						\
  clips_->add_function(n, s);						\
  functions_.push_back(n);


/** Setup CLIPS environment. */
void
ClipsProtobufCommunicator::setup_clips()
{
  fawkes::MutexLocker lock(&clips_mutex_);

  ADD_FUNCTION("pb-register-type", (sigc::slot<CLIPS::Value, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_register_type))));
  ADD_FUNCTION("pb-field-names", (sigc::slot<CLIPS::Values, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_names))));
  ADD_FUNCTION("pb-field-type", (sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_type))));
  ADD_FUNCTION("pb-has-field", (sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_has_field))));
  ADD_FUNCTION("pb-field-label", (sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_label))));
  ADD_FUNCTION("pb-field-value", (sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_value))));
  ADD_FUNCTION("pb-field-list", (sigc::slot<CLIPS::Values, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_list))));
  ADD_FUNCTION("pb-field-is-list", (sigc::slot<CLIPS::Value, void *, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_field_is_list))));
  ADD_FUNCTION("pb-create", (sigc::slot<CLIPS::Value, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_create))));
  ADD_FUNCTION("pb-destroy", (sigc::slot<void, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_destroy))));
  ADD_FUNCTION("pb-ref", (sigc::slot<CLIPS::Value, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_ref))));
  ADD_FUNCTION("pb-set-field", (sigc::slot<void, void *, std::string, CLIPS::Value>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_set_field))));
  ADD_FUNCTION("pb-add-list", (sigc::slot<void, void *, std::string, CLIPS::Value>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_add_list))));
  ADD_FUNCTION("pb-send", (sigc::slot<void, long int, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_send))));
  ADD_FUNCTION("pb-tostring", (sigc::slot<std::string, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_tostring))));
  ADD_FUNCTION("pb-server-enable", (sigc::slot<void, int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::enable_server))));
  ADD_FUNCTION("pb-server-disable", (sigc::slot<void>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::disable_server))));
  ADD_FUNCTION("pb-peer-create", (sigc::slot<long int, std::string, int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_create))));
  ADD_FUNCTION("pb-peer-create-local", (sigc::slot<long int, std::string, int, int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_create_local))));
  ADD_FUNCTION("pb-peer-create-crypto",
	       (sigc::slot<long int, std::string, int, std::string, std::string>
		 (sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_create_crypto))));
  ADD_FUNCTION("pb-peer-create-local-crypto",
	       (sigc::slot<long int, std::string, int, int, std::string, std::string>
		 (sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_create_local_crypto))));
  ADD_FUNCTION("pb-peer-destroy", (sigc::slot<void, long int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_destroy))));
  ADD_FUNCTION("pb-peer-setup-crypto", (sigc::slot<void, long int, std::string, std::string>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_peer_setup_crypto))));
  ADD_FUNCTION("pb-broadcast", (sigc::slot<void, long int, void *>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_broadcast))));
  ADD_FUNCTION("pb-connect", (sigc::slot<long int, std::string, int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_client_connect))));
  ADD_FUNCTION("pb-disconnect", (sigc::slot<void, long int>(sigc::mem_fun(*this, &ClipsProtobufCommunicator::clips_pb_disconnect))));
}

/** Enable protobuf stream server.
 * @param port TCP port to listen on for connections
 */
void
ClipsProtobufCommunicator::enable_server(int port)
{
  if ((port > 0) && ! server_) {
    server_ = new protobuf_comm::ProtobufStreamServer(port, message_register_);

    server_->signal_connected()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_server_client_connected, this, _1, _2));
    server_->signal_disconnected()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_server_client_disconnected, this, _1, _2));
    server_->signal_received()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_server_client_msg, this, _1, _2, _3, _4));
    server_->signal_receive_failed()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_server_client_fail, this, _1, _2, _3, _4));
  }

}


/** Disable protobu stream server. */
void
ClipsProtobufCommunicator::disable_server()
{
  delete server_;
  server_ = NULL;
}


/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int
ClipsProtobufCommunicator::clips_pb_peer_create_local_crypto(std::string address, int send_port, int recv_port,
							     std::string crypto_key, std::string cipher)
{
  if (recv_port <= 0)  recv_port = send_port;

  if (send_port > 0) {
    protobuf_comm::ProtobufBroadcastPeer *peer =
      new protobuf_comm::ProtobufBroadcastPeer(address, send_port, recv_port,
					       message_register_, crypto_key, cipher);

    long int peer_id;
    {
      fawkes::MutexLocker lock(&map_mutex_);
      peer_id = ++next_client_id_;
      peers_[peer_id] = peer;
    }

    peer->signal_received()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_peer_msg, this, peer_id, _1, _2, _3, _4));
    peer->signal_recv_error()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_peer_recv_error, this, peer_id, _1, _2));
    peer->signal_send_error()
      .connect(boost::bind(&ClipsProtobufCommunicator::handle_peer_send_error, this, peer_id, _1));

    return peer_id;
  } else {
    return 0;
  }
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int
ClipsProtobufCommunicator::clips_pb_peer_create_crypto(std::string address, int port,
						       std::string crypto_key, std::string cipher)
{
  return clips_pb_peer_create_local_crypto(address, port, port, crypto_key, cipher);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @return peer identifier
 */
long int
ClipsProtobufCommunicator::clips_pb_peer_create(std::string address, int port)
{
  return clips_pb_peer_create_local_crypto(address, port, port);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @return peer identifier
 */
long int
ClipsProtobufCommunicator::clips_pb_peer_create_local(std::string address, int send_port,
						      int recv_port)
{
  return clips_pb_peer_create_local_crypto(address, send_port, recv_port);
}


/** Disable peer.
 * @param peer_id ID of the peer to destroy
 */
void
ClipsProtobufCommunicator::clips_pb_peer_destroy(long int peer_id)
{
  if (peers_.find(peer_id) != peers_.end()) {
    delete peers_[peer_id];
    peers_.erase(peer_id);
  }
}


/** Setup crypto for peer. 
 * @param peer_id ID of the peer to destroy
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 */
void
ClipsProtobufCommunicator::clips_pb_peer_setup_crypto(long int peer_id,
						      std::string crypto_key, std::string cipher)
{
  if (peers_.find(peer_id) != peers_.end()) {
    peers_[peer_id]->setup_crypto(crypto_key, cipher);
  }
}


/** Register a new message type.
 * @param full_name full name of type to register
 * @return true if the type was successfully registered, false otherwise
 */
CLIPS::Value
ClipsProtobufCommunicator::clips_pb_register_type(std::string full_name)
{
  try {
    message_register_->add_message_type(full_name);
    return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
  } catch (std::runtime_error &e) {
    if (logger_) {
      logger_->log_error("CLIPS-Protobuf", "Registering type %s failed: %s",
			 full_name.c_str(), e.what());
    }
    return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  }
}



CLIPS::Value
ClipsProtobufCommunicator::clips_pb_create(std::string full_name)
{
  try {
    std::shared_ptr<google::protobuf::Message> m =
      message_register_->new_message_for(full_name);
    return CLIPS::Value(new std::shared_ptr<google::protobuf::Message>(m));
  } catch (std::runtime_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Cannot create message of type %s: %s",
			full_name.c_str(), e.what());
    }
    return CLIPS::Value(new std::shared_ptr<google::protobuf::Message>());
  }
}


CLIPS::Value
ClipsProtobufCommunicator::clips_pb_ref(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return new std::shared_ptr<google::protobuf::Message>();

  return CLIPS::Value(new std::shared_ptr<google::protobuf::Message>(*m));
}


void
ClipsProtobufCommunicator::clips_pb_destroy(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return;

  delete m;
}


CLIPS::Values
ClipsProtobufCommunicator::clips_pb_field_names(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return CLIPS::Values();

  const Descriptor *desc = (*m)->GetDescriptor();
  const int field_count  = desc->field_count();
  CLIPS::Values field_names(field_count);
  for (int i = 0; i < field_count; ++i) {
    field_names[i].set(desc->field(i)->name(), true);
  }
  return field_names;
}

CLIPS::Value
ClipsProtobufCommunicator::clips_pb_field_type(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    return CLIPS::Value("DOES-NOT-EXIST", CLIPS::TYPE_SYMBOL);
  }
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:   return CLIPS::Value("DOUBLE", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_FLOAT:    return CLIPS::Value("FLOAT", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_INT64:    return CLIPS::Value("INT64", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_UINT64:   return CLIPS::Value("UINT64", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_INT32:    return CLIPS::Value("INT32", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_FIXED64:  return CLIPS::Value("FIXED64", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_FIXED32:  return CLIPS::Value("FIXED32", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_BOOL:     return CLIPS::Value("BOOL", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_STRING:   return CLIPS::Value("STRING", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_MESSAGE:  return CLIPS::Value("MESSAGE", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_BYTES:    return CLIPS::Value("BYTES", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_UINT32:   return CLIPS::Value("UINT32", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_ENUM:     return CLIPS::Value("ENUM", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_SFIXED32: return CLIPS::Value("SFIXED32", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_SFIXED64: return CLIPS::Value("SFIXED64", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_SINT32:   return CLIPS::Value("SINT32", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_SINT64:   return CLIPS::Value("SINT64", CLIPS::TYPE_SYMBOL);
  default: return CLIPS::Value("UNKNOWN", CLIPS::TYPE_SYMBOL);
  }
}

CLIPS::Value
ClipsProtobufCommunicator::clips_pb_has_field(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return false;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return false;

  const Reflection *refl       = (*m)->GetReflection();

  if (field->is_repeated()) {
    return CLIPS::Value((refl->FieldSize(**m, field) > 0) ? "TRUE" : "FALSE",
			CLIPS::TYPE_SYMBOL);
  } else if (field->is_optional()) {
    return CLIPS::Value(refl->HasField(**m, field) ? "TRUE" : "FALSE",
			CLIPS::TYPE_SYMBOL);
  } else {
    return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
  }
}


CLIPS::Value
ClipsProtobufCommunicator::clips_pb_field_label(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    return CLIPS::Value("DOES-NOT-EXIST", CLIPS::TYPE_SYMBOL);
  }
  switch (field->label()) {
  case FieldDescriptor::LABEL_OPTIONAL: return CLIPS::Value("OPTIONAL", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::LABEL_REQUIRED: return CLIPS::Value("REQUIRED", CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::LABEL_REPEATED: return CLIPS::Value("REPEATED", CLIPS::TYPE_SYMBOL);
  default:                              return CLIPS::Value("UNKNOWN", CLIPS::TYPE_SYMBOL);
  }
}

CLIPS::Value
ClipsProtobufCommunicator::clips_pb_field_value(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
		if (logger_) {
			logger_->log_warn("CLIPS-Protobuf", "Invalid message when setting %s", field_name.c_str());
		}
		return CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL);
	}

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Field %s of %s does not exist",
			field_name.c_str(), (*m)->GetTypeName().c_str());
    }
    return CLIPS::Value("DOES-NOT-EXIST", CLIPS::TYPE_SYMBOL);
  }
  const Reflection *refl       = (*m)->GetReflection();
  if (field->type() != FieldDescriptor::TYPE_MESSAGE && ! refl->HasField(**m, field)) {
    if (logger_) {
        logger_->log_warn("CLIPS-Protobuf", "Field %s of %s not set",
			  field_name.c_str(), (*m)->GetTypeName().c_str());
    }
    return CLIPS::Value("NOT-SET", CLIPS::TYPE_SYMBOL);
  }
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:   return CLIPS::Value(refl->GetDouble(**m, field));
  case FieldDescriptor::TYPE_FLOAT:    return CLIPS::Value(refl->GetFloat(**m, field));
  case FieldDescriptor::TYPE_INT64:    return CLIPS::Value(refl->GetInt64(**m, field));
  case FieldDescriptor::TYPE_UINT64:
    return CLIPS::Value((long int)refl->GetUInt64(**m, field));
  case FieldDescriptor::TYPE_INT32:    return CLIPS::Value(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_FIXED64:
    return CLIPS::Value((long int)refl->GetUInt64(**m, field));
  case FieldDescriptor::TYPE_FIXED32:  return CLIPS::Value(refl->GetUInt32(**m, field));
  case FieldDescriptor::TYPE_BOOL:
    //Booleans are represented as Symbols in CLIPS
    if(refl->GetBool(**m, field)){
      return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
    }
    else{
      return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
    }
  case FieldDescriptor::TYPE_STRING:   return CLIPS::Value(refl->GetString(**m, field));
  case FieldDescriptor::TYPE_MESSAGE:
    {
      const google::protobuf::Message &mfield = refl->GetMessage(**m, field);
      google::protobuf::Message *mcopy = mfield.New();
      mcopy->CopyFrom(mfield);
      void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
      return CLIPS::Value(ptr);
    }
  case FieldDescriptor::TYPE_BYTES:    return CLIPS::Value((char *)"bytes");
  case FieldDescriptor::TYPE_UINT32:   return CLIPS::Value(refl->GetUInt32(**m, field));
  case FieldDescriptor::TYPE_ENUM:
    return CLIPS::Value(refl->GetEnum(**m, field)->name(), CLIPS::TYPE_SYMBOL);
  case FieldDescriptor::TYPE_SFIXED32: return CLIPS::Value(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_SFIXED64: return CLIPS::Value(refl->GetInt64(**m, field));
  case FieldDescriptor::TYPE_SINT32:   return CLIPS::Value(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_SINT64:   return CLIPS::Value(refl->GetInt64(**m, field));
  default:
    throw std::logic_error("Unknown protobuf field type encountered");
  }
}


void
ClipsProtobufCommunicator::clips_pb_set_field(void *msgptr, std::string field_name, CLIPS::Value value)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) return;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Could not find field %s", field_name.c_str());
    }
    return;
  }
  const Reflection *refl       = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      refl->SetDouble(m->get(), field, value.as_float()); break;
    case FieldDescriptor::TYPE_FLOAT:
      refl->SetFloat(m->get(), field, value.as_float());  break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      refl->SetInt64(m->get(), field, value.as_integer());  break;
    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
      refl->SetUInt64(m->get(), field, value.as_integer()); break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
      refl->SetInt32(m->get(), field, value.as_integer()); break;
    case FieldDescriptor::TYPE_BOOL:
      refl->SetBool(m->get(), field, (value == "TRUE"));
      break;
    case FieldDescriptor::TYPE_STRING:
      refl->SetString(m->get(), field, value.as_string()); break;
    case FieldDescriptor::TYPE_MESSAGE:
      {
	std::shared_ptr<google::protobuf::Message> *mfrom =
	  static_cast<std::shared_ptr<google::protobuf::Message> *>(value.as_address());
	Message *mut_msg = refl->MutableMessage(m->get(), field);
	mut_msg->CopyFrom(**mfrom);
	delete mfrom;
      }
      break;
    case FieldDescriptor::TYPE_BYTES:    break;
    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      refl->SetUInt32(m->get(), field, value.as_integer()); break;
    case FieldDescriptor::TYPE_ENUM:
      {
	const EnumDescriptor *enumdesc = field->enum_type();
	const EnumValueDescriptor *enumval = enumdesc->FindValueByName(value);
	if (enumval) {
	  refl->SetEnum(m->get(), field, enumval);
	} else {
	  if (logger_) {
	    logger_->log_warn("CLIPS-Protobuf", "%s: cannot set invalid "
			      "enum value '%s' on '%s'",
			      (*m)->GetTypeName().c_str(),
			      value.as_string().c_str(), field_name.c_str());
	  }
	}
      }
      break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Failed to set field %s of %s: %s "
			"(type %d, as string %s)",
			field_name.c_str(), (*m)->GetTypeName().c_str(), e.what(),
			value.type(), to_string(value).c_str());
    }
  }
}


void
ClipsProtobufCommunicator::clips_pb_add_list(void *msgptr, std::string field_name, CLIPS::Value value)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) return;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Could not find field %s",
			field_name.c_str());
    }
    return;
  }
  const Reflection *refl       = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:   refl->AddDouble(m->get(), field, value); break;
    case FieldDescriptor::TYPE_FLOAT:    refl->AddFloat(m->get(), field, value);  break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      refl->AddInt64(m->get(), field, value);  break;
    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
      refl->AddUInt64(m->get(), field, (long int)value); break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
  case FieldDescriptor::TYPE_INT32:
      refl->AddInt32(m->get(), field, value); break;
    case FieldDescriptor::TYPE_BOOL:
      refl->AddBool(m->get(), field, (value == "TRUE"));
      break;
    case FieldDescriptor::TYPE_STRING:   refl->AddString(m->get(), field, value); break;
    case FieldDescriptor::TYPE_MESSAGE:
      {
	std::shared_ptr<google::protobuf::Message> *mfrom =
	  static_cast<std::shared_ptr<google::protobuf::Message> *>(value.as_address());
	Message *new_msg = refl->AddMessage(m->get(), field);
	new_msg->CopyFrom(**mfrom);
	delete mfrom;
      }
      break;
    case FieldDescriptor::TYPE_BYTES:    break;
    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      refl->AddUInt32(m->get(), field, value); break;
    case FieldDescriptor::TYPE_ENUM:
      {
	const EnumDescriptor *enumdesc = field->enum_type();
	const EnumValueDescriptor *enumval = enumdesc->FindValueByName(value);
	if (enumval)  refl->AddEnum(m->get(), field, enumval);
      }
      break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Failed to add field %s of %s: %s",
			field_name.c_str(), (*m)->GetTypeName().c_str(), e.what());
    }
  }
}


long int
ClipsProtobufCommunicator::clips_pb_client_connect(std::string host, int port)
{
  if (port <= 0) return false;

  ProtobufStreamClient *client = new ProtobufStreamClient(message_register_);

  long int client_id;
  {
    fawkes::MutexLocker lock(&map_mutex_);
    client_id = ++next_client_id_;
    clients_[client_id] = client;
  }

  client->signal_connected().connect(
    boost::bind(&ClipsProtobufCommunicator::handle_client_connected, this, client_id));
  client->signal_disconnected().connect(
    boost::bind(&ClipsProtobufCommunicator::handle_client_disconnected,
		this, client_id, boost::asio::placeholders::error));
  client->signal_received().connect(
    boost::bind(&ClipsProtobufCommunicator::handle_client_msg, this, client_id, _1, _2, _3));
  client->signal_receive_failed().connect(
    boost::bind(&ClipsProtobufCommunicator::handle_client_receive_fail, this, client_id, _1, _2, _3));

  client->async_connect(host.c_str(), port);
  return CLIPS::Value(client_id);
}


void
ClipsProtobufCommunicator::clips_pb_send(long int client_id, void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Cannot send to %li: invalid message", client_id);
    }
    return;
  }

  try {
    fawkes::MutexLocker lock(&map_mutex_);

    if (server_ && server_clients_.find(client_id) != server_clients_.end()) {
      //printf("***** SENDING via SERVER\n");
      server_->send(server_clients_[client_id], *m);
      sig_server_sent_(server_clients_[client_id], *m);
    } else if (clients_.find(client_id) != clients_.end()) {
      //printf("***** SENDING via CLIENT\n");
      clients_[client_id]->send(*m);
      std::pair<std::string, unsigned short> &client_endpoint = client_endpoints_[client_id];
      sig_client_sent_(client_endpoint.first, client_endpoint.second, *m);
    } else if (peers_.find(client_id) != peers_.end()) {
      //printf("***** SENDING via CLIENT\n");
      peers_[client_id]->send(*m);
      sig_peer_sent_(client_id, *m);
    } else {
      //printf("Client ID %li is unknown, cannot send message of type %s\n",
      //     client_id, (*m)->GetTypeName().c_str());
    }
  } catch (google::protobuf::FatalException &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Profobuf", "Failed to send message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what());
    }
  } catch (fawkes::Exception &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Failed to send message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what_no_backtrace());
    }
  } catch (std::runtime_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Failed to send message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what());
    }
  }
}

std::string
ClipsProtobufCommunicator::clips_pb_tostring(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Cannot convert message to string: invalid message");
    }
    return "";
  }

  return (*m)->DebugString();
}


void
ClipsProtobufCommunicator::clips_pb_broadcast(long int peer_id, void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf", "Cannot send broadcast: invalid message");
    }
    return;
  }

  fawkes::MutexLocker lock(&map_mutex_);
  if (peers_.find(peer_id) == peers_.end())  return;

  //logger_->log_info("CLIPS-Protobuf", "Broadcasting %s", (*m)->GetTypeName().c_str());
  try {
    peers_[peer_id]->send(*m);
  } catch (google::protobuf::FatalException &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Failed to broadcast message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what());
    }
  } catch (fawkes::Exception &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Failed to broadcast message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what_no_backtrace());
    }
  } catch (std::runtime_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Failed to broadcast message of type %s: %s",
			(*m)->GetTypeName().c_str(), e.what());
    }
  }

  sig_peer_sent_(peer_id, *m);
}


void
ClipsProtobufCommunicator::clips_pb_disconnect(long int client_id)
{
  //logger_->log_info("CLIPS-Protobuf", "Disconnecting client %li", client_id);

  try {
    fawkes::MutexLocker lock(&map_mutex_);

    if (server_clients_.find(client_id) != server_clients_.end()) {
      protobuf_comm::ProtobufStreamServer::ClientID srv_client = server_clients_[client_id];
      server_->disconnect(srv_client);
      server_clients_.erase(client_id);
      rev_server_clients_.erase(srv_client);
    } else if (clients_.find(client_id) != clients_.end()) {
      delete clients_[client_id];
      clients_.erase(client_id);
    }
  } catch (std::runtime_error &e) {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Failed to disconnect from client %li: %s",
			client_id, e.what());
    }
  }
}

CLIPS::Values
ClipsProtobufCommunicator::clips_pb_field_list(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) return CLIPS::Values(1, CLIPS::Value("INVALID-MESSAGE", CLIPS::TYPE_SYMBOL));

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    return CLIPS::Values(1, CLIPS::Value("DOES-NOT-EXIST", CLIPS::TYPE_SYMBOL));
  }
  if (field->label() == FieldDescriptor::LABEL_REQUIRED ||
      field->label() == FieldDescriptor::LABEL_OPTIONAL)
  {
    CLIPS::Values rv(1, clips_pb_field_value(msgptr, field_name));
    return rv;
  }

  const Reflection *refl       = (*m)->GetReflection();
  int field_size = refl->FieldSize(**m, field);
  CLIPS::Values rv(field_size);
  for (int i = 0; i < field_size; ++i) {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      rv[i] = CLIPS::Value(refl->GetRepeatedDouble(**m, field, i));
      break;
    case FieldDescriptor::TYPE_FLOAT:
      rv[i] = CLIPS::Value(refl->GetRepeatedFloat(**m, field, i));
      break;
      break;
    case FieldDescriptor::TYPE_UINT64:
    case FieldDescriptor::TYPE_FIXED64:
      rv[i] = CLIPS::Value((long int)refl->GetRepeatedUInt64(**m, field, i));
      break;
    case FieldDescriptor::TYPE_UINT32:
    case FieldDescriptor::TYPE_FIXED32:
      rv[i] = CLIPS::Value(refl->GetRepeatedUInt32(**m, field, i));
      break;
    case FieldDescriptor::TYPE_BOOL:
      //Booleans are represented as Symbols in CLIPS
      if(refl->GetRepeatedBool(**m, field, i)){
	rv[i] = CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
      }
      else{
	rv[i] = CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
      }
      break;
    case FieldDescriptor::TYPE_STRING:
      rv[i] = CLIPS::Value(refl->GetRepeatedString(**m, field, i));
      break;
    case FieldDescriptor::TYPE_MESSAGE:
      {
	const google::protobuf::Message &msg = refl->GetRepeatedMessage(**m, field, i);
	google::protobuf::Message *mcopy = msg.New();
	mcopy->CopyFrom(msg);
	void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
	rv[i] = CLIPS::Value(ptr);
      }
      break;
    case FieldDescriptor::TYPE_BYTES:
      rv[i] = CLIPS::Value((char *)"BYTES", CLIPS::TYPE_SYMBOL);
      break;
    case FieldDescriptor::TYPE_ENUM:
      rv[i] = CLIPS::Value(refl->GetRepeatedEnum(**m, field, i)->name(), CLIPS::TYPE_SYMBOL);
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_INT32:
    case FieldDescriptor::TYPE_SINT32:
      rv[i] = CLIPS::Value(refl->GetRepeatedInt32(**m, field, i));
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      rv[i] = CLIPS::Value(refl->GetRepeatedInt64(**m, field, i));
      break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  }

  return rv;
}


CLIPS::Value
ClipsProtobufCommunicator::clips_pb_field_is_list(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
  return CLIPS::Value(field->is_repeated() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}


void
ClipsProtobufCommunicator::clips_assert_message(std::pair<std::string, unsigned short> &endpoint,
						uint16_t comp_id, uint16_t msg_type,
						std::shared_ptr<google::protobuf::Message> &msg,
						ClipsProtobufCommunicator::ClientType ct,
						unsigned int client_id)
{
  CLIPS::Template::pointer temp = clips_->get_template("protobuf-msg");
  if (temp) {
    struct timeval tv;
    gettimeofday(&tv, 0);
    void *ptr = new std::shared_ptr<google::protobuf::Message>(msg);
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(*clips_, temp);
    fact->set_slot("type", msg->GetTypeName());
    fact->set_slot("comp-id", comp_id);
    fact->set_slot("msg-type", msg_type);
    fact->set_slot("rcvd-via",
		   CLIPS::Value((ct == CT_PEER) ? "BROADCAST" : "STREAM", CLIPS::TYPE_SYMBOL));
    CLIPS::Values rcvd_at(2, CLIPS::Value(CLIPS::TYPE_INTEGER));
    rcvd_at[0] = tv.tv_sec;
    rcvd_at[1] = tv.tv_usec;
    fact->set_slot("rcvd-at", rcvd_at);
    CLIPS::Values host_port(2, CLIPS::Value(CLIPS::TYPE_STRING));
    host_port[0] = endpoint.first;
    host_port[1] = CLIPS::Value(endpoint.second);
    fact->set_slot("rcvd-from", host_port);
    fact->set_slot("client-type",
      CLIPS::Value(ct == CT_CLIENT ? "CLIENT" :
		   (ct == CT_SERVER ? "SERVER" : "PEER"), CLIPS::TYPE_SYMBOL));
    fact->set_slot("client-id", client_id);
    fact->set_slot("ptr", CLIPS::Value(ptr));
    CLIPS::Fact::pointer new_fact = clips_->assert_fact(fact);

    if (! new_fact) {
      if (logger_) {
	      logger_->log_warn("CLIPS-Protobuf", "Asserting protobuf-msg fact failed");
      }
      delete static_cast<std::shared_ptr<google::protobuf::Message> *>(ptr);
    }
  } else {
    if (logger_) {
      logger_->log_warn("CLIPS-Protobuf",
			"Did not get template, did you load protobuf.clp?");
    }
  }
}

void
ClipsProtobufCommunicator::handle_server_client_connected(ProtobufStreamServer::ClientID client,
							  boost::asio::ip::tcp::endpoint &endpoint)
{

  long int client_id = -1;
  {
    fawkes::MutexLocker lock(&map_mutex_);
    client_id = ++next_client_id_;
    client_endpoints_[client_id] =
      std::make_pair(endpoint.address().to_string(), endpoint.port());
    server_clients_[client_id] = client;
    rev_server_clients_[client] = client_id;
  }

  fawkes::MutexLocker lock(&clips_mutex_);
  clips_->assert_fact_f("(protobuf-server-client-connected %li %s %u)", client_id,
			endpoint.address().to_string().c_str(), endpoint.port());
}


void
ClipsProtobufCommunicator::handle_server_client_disconnected(ProtobufStreamServer::ClientID client,
							     const boost::system::error_code &error)
{
  long int client_id = -1;
  {
    fawkes::MutexLocker lock(&map_mutex_);
    RevServerClientMap::iterator c;
    if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
      client_id = c->second;
      rev_server_clients_.erase(c);
      server_clients_.erase(client_id);
    }
  }

  if (client_id >= 0) {
    fawkes::MutexLocker lock(&clips_mutex_);
    clips_->assert_fact_f("(protobuf-server-client-disconnected %li)", client_id);
  }
}


/** Handle message that came from a client.
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
ClipsProtobufCommunicator::handle_server_client_msg(ProtobufStreamServer::ClientID client,
						    uint16_t component_id, uint16_t msg_type,
						    std::shared_ptr<google::protobuf::Message> msg)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  fawkes::MutexLocker lock2(&map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    clips_assert_message(client_endpoints_[c->second],
			 component_id, msg_type, msg, CT_SERVER, c->second);
  }
}

/** Handle server reception failure
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message string
 */
void
ClipsProtobufCommunicator::handle_server_client_fail(ProtobufStreamServer::ClientID client,
						     uint16_t component_id, uint16_t msg_type,
						     std::string msg)
{
  fawkes::MutexLocker lock(&map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    fawkes::MutexLocker lock(&clips_mutex_);
    clips_->assert_fact_f("(protobuf-server-receive-failed (comp-id %u) (msg-type %u) "
			  "(rcvd-via STREAM) (client-id %li) (message \"%s\") "
			  "(rcvd-from (\"%s\" %u)))",
			  component_id, msg_type, c->second, msg.c_str(),
			  client_endpoints_[c->second].first.c_str(),
			  client_endpoints_[c->second].second);
  }
}


/** Handle message that came from a peer/robot
 * @param endpoint the endpoint from which the message was received
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
ClipsProtobufCommunicator::handle_peer_msg(long int peer_id,
					   boost::asio::ip::udp::endpoint &endpoint,
					   uint16_t component_id, uint16_t msg_type,
					   std::shared_ptr<google::protobuf::Message> msg)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  std::pair<std::string, unsigned short> endpp =
    std::make_pair(endpoint.address().to_string(), endpoint.port());
  clips_assert_message(endpp, component_id, msg_type, msg, CT_PEER, peer_id);
}


/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
ClipsProtobufCommunicator::handle_peer_recv_error(long int peer_id,
						  boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
  if (logger_) {
    logger_->log_warn("CLIPS-Protobuf",
		      "Failed to receive peer message from %s:%u: %s",
		      endpoint.address().to_string().c_str(), endpoint.port(),
		      msg.c_str());
  }
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
ClipsProtobufCommunicator::handle_peer_send_error(long int peer_id, std::string msg)
{
  if (logger_) {
    logger_->log_warn("CLIPS-Protobuf",
		      "Failed to send peer message: %s", msg.c_str());
  }
}


void
ClipsProtobufCommunicator::handle_client_connected(long int client_id)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  clips_->assert_fact_f("(protobuf-client-connected %li)", client_id);
}

void
ClipsProtobufCommunicator::handle_client_disconnected(long int client_id,
						      const boost::system::error_code &error)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  clips_->assert_fact_f("(protobuf-client-disconnected %li)", client_id);
}

void
ClipsProtobufCommunicator::handle_client_msg(long int client_id,
					     uint16_t comp_id, uint16_t msg_type,
					     std::shared_ptr<google::protobuf::Message> msg)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  std::pair<std::string, unsigned short> endpp = std::make_pair(std::string(), 0);
  clips_assert_message(endpp, comp_id, msg_type, msg, CT_CLIENT, client_id);
}


void
ClipsProtobufCommunicator::handle_client_receive_fail(long int client_id,
						      uint16_t comp_id, uint16_t msg_type, std::string msg)
{
  fawkes::MutexLocker lock(&clips_mutex_);
  clips_->assert_fact_f("(protobuf-receive-failed (client-id %li) (rcvd-via STREAM) "
			"(comp-id %u) (msg-type %u) (message \"%s\"))",
			client_id, comp_id, msg_type, msg.c_str());
}

std::string
ClipsProtobufCommunicator::to_string(const CLIPS::Value &v)
{
  switch (v.type()) {
  case CLIPS::TYPE_UNKNOWN: return "Unknown Type";
  case CLIPS::TYPE_FLOAT:   return std::to_string(v.as_float());
  case CLIPS::TYPE_INTEGER: return std::to_string(v.as_integer());
  case CLIPS::TYPE_SYMBOL:
  case CLIPS::TYPE_INSTANCE_NAME:
  case CLIPS::TYPE_STRING:  return v.as_string();
  case CLIPS::TYPE_INSTANCE_ADDRESS:
  case CLIPS::TYPE_EXTERNAL_ADDRESS:
    return boost::str(boost::format("%p") % v.as_address());
  }
  return "Implicit unknown type";
}

} // end namespace protobuf_clips
