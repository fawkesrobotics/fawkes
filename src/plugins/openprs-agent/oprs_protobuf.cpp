
/***************************************************************************
 *  oprs_protobuf.cpp - protobuf network communication for OpenPRS
 *
 *  Created: Tue Sep 02 16:53:26 2014 (based on CLIPS version)
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
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

#include "oprs_protobuf.h"

#include <core/threading/mutex_locker.h>
#include <core/exception.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/server.h>
#include <protobuf_comm/peer.h>

#include <oprs_f-pub.h>
#include <google/protobuf/descriptor.h>

#include <algorithm>

using namespace google::protobuf;
using namespace protobuf_comm;

namespace oprs_protobuf {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenPRSProtobuf "oprs_protobuf.h"
 * OpenPRS protobuf integration class.
 * This class adds functionality related to protobuf to OpenPRS.
 * It supports the creation of communication channels through protobuf_comm.
 * An instance maintains its own message register shared among server, peer,
 * and clients.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param proto_path proto path passed to a newly instantiated message register
 */
OpenPRSProtobuf::OpenPRSProtobuf(std::vector<std::string> &proto_path)
  : server_(NULL), next_client_id_(0)
{
  message_register_ = new MessageRegister(proto_path);
}


/** Destructor. */
OpenPRSProtobuf::~OpenPRSProtobuf()
{
  for (auto c : clients_) {
    delete c.second;
  }
  clients_.clear();

  delete message_register_;
  delete server_;
}


/** Enable protobuf stream server.
 * @param port TCP port to listen on for connections
 */
void
OpenPRSProtobuf::oprs_pb_enable_server(int port)
{
  if ((port > 0) && ! server_) {
    server_ = new protobuf_comm::ProtobufStreamServer(port, message_register_);

    server_->signal_connected()
      .connect(boost::bind(&OpenPRSProtobuf::handle_server_client_connected, this, _1, _2));
    server_->signal_disconnected()
      .connect(boost::bind(&OpenPRSProtobuf::handle_server_client_disconnected, this, _1, _2));
    server_->signal_received()
      .connect(boost::bind(&OpenPRSProtobuf::handle_server_client_msg, this, _1, _2, _3, _4));
    server_->signal_receive_failed()
      .connect(boost::bind(&OpenPRSProtobuf::handle_server_client_fail, this, _1, _2, _3, _4));
  }

}


/** Disable protobuf stream server. */
void
OpenPRSProtobuf::oprs_pb_disable_server()
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
Term *
OpenPRSProtobuf::oprs_pb_peer_create_local_crypto(std::string address, int send_port, int recv_port,
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
      .connect(boost::bind(&OpenPRSProtobuf::handle_peer_msg, this, peer_id, _1, _2, _3, _4));
    peer->signal_recv_error()
      .connect(boost::bind(&OpenPRSProtobuf::handle_peer_recv_error, this, peer_id, _1, _2));
    peer->signal_send_error()
      .connect(boost::bind(&OpenPRSProtobuf::handle_peer_send_error, this, peer_id, _1));

    return build_long_long(peer_id);
  } else {
    return build_long_long(0);
  }
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
Term *
OpenPRSProtobuf::oprs_pb_peer_create_crypto(std::string address, int port,
						       std::string crypto_key, std::string cipher)
{
  return oprs_pb_peer_create_local_crypto(address, port, port, crypto_key, cipher);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @return peer identifier
 */
Term *
OpenPRSProtobuf::oprs_pb_peer_create(std::string address, int port)
{
  return oprs_pb_peer_create_local_crypto(address, port, port);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @return peer identifier
 */
Term *
OpenPRSProtobuf::oprs_pb_peer_create_local(std::string address, int send_port,
						      int recv_port)
{
  return oprs_pb_peer_create_local_crypto(address, send_port, recv_port);
}


/** Disable peer.
 * @param peer_id ID of the peer to destroy
 */
void
OpenPRSProtobuf::oprs_pb_peer_destroy(long int peer_id)
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
OpenPRSProtobuf::oprs_pb_peer_setup_crypto(long int peer_id,
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
bool
OpenPRSProtobuf::oprs_pb_register_type(std::string full_name)
{
  try {
    message_register_->add_message_type(full_name);
    return true;
  } catch (std::runtime_error &e) {
    //logger_->log_error("RefBox", "Registering type %s failed: %s", full_name.c_str(), e.what());
    return false;
  }
}



/** Create a new message of given type.
 * @param full_name name of message type (fully qualified, i.e. including package name)
 * @return shared pointer to new mesage
 * @exception std::runtime_error thrown if creating the message failed
 */
std::shared_ptr<google::protobuf::Message> *
OpenPRSProtobuf::oprs_create_msg(std::string full_name)
{
  std::shared_ptr<google::protobuf::Message> m =
    message_register_->new_message_for(full_name);
  return new std::shared_ptr<google::protobuf::Message>(m);
}


/** Create new reference to message.
 * @param msgptr message to create reference for
 * @return new message reference pointing to the very same message as @p msgptr
 */
Term *
OpenPRSProtobuf::oprs_pb_ref(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_pointer(new std::shared_ptr<google::protobuf::Message>());

  return build_pointer(new std::shared_ptr<google::protobuf::Message>(*m));
}


/** Destroy given message (reference).
 * This will decrement the reference count to the message and delete it.
 * The message itself is deleted if the reference counter reaches zero.
 * @param msgptr message (reference) to delete, any access to this message
 * afterwards is illegal.
 * @return T
 */
Term *
OpenPRSProtobuf::oprs_pb_destroy(void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_nil();

  delete m;
  return build_t();
}


/** Get field names of message.
 * @param msgptr user pointer to message
 * @return term containing lisp list of field names
 */
Term *
OpenPRSProtobuf::oprs_pb_field_names(void *msgptr)
{
  TermList tl = sl_make_slist();

  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_term_l_list_from_c_list(tl);


  const Descriptor *desc = (*m)->GetDescriptor();
  const int field_count  = desc->field_count();
  for (int i = 0; i < field_count; ++i) {
    tl = build_term_list(tl, build_string(desc->field(i)->name().c_str()));
  }
  return build_term_l_list_from_c_list(tl);
}


/** Get type if a specific field.
 * @param msgptr message for which to get the field type
 * @param field_name name of the field
 * @return term with a symbol for the type
 */
Term *
OpenPRSProtobuf::oprs_pb_field_type(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_id(declare_atom("INVALID-MESSAGE"));

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    return build_id(declare_atom("DOES-NOT-EXIST"));
  }
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:   return build_id(declare_atom("DOUBLE"));
  case FieldDescriptor::TYPE_FLOAT:    return build_id(declare_atom("FLOAT"));
  case FieldDescriptor::TYPE_INT64:    return build_id(declare_atom("INT64"));
  case FieldDescriptor::TYPE_UINT64:   return build_id(declare_atom("UINT64"));
  case FieldDescriptor::TYPE_INT32:    return build_id(declare_atom("INT32"));
  case FieldDescriptor::TYPE_FIXED64:  return build_id(declare_atom("FIXED64"));
  case FieldDescriptor::TYPE_FIXED32:  return build_id(declare_atom("FIXED32"));
  case FieldDescriptor::TYPE_BOOL:     return build_id(declare_atom("BOOL"));
  case FieldDescriptor::TYPE_STRING:   return build_id(declare_atom("STRING"));
  case FieldDescriptor::TYPE_MESSAGE:  return build_id(declare_atom("MESSAGE"));
  case FieldDescriptor::TYPE_BYTES:    return build_id(declare_atom("BYTES"));
  case FieldDescriptor::TYPE_UINT32:   return build_id(declare_atom("UINT32"));
  case FieldDescriptor::TYPE_ENUM:     return build_id(declare_atom("ENUM"));
  case FieldDescriptor::TYPE_SFIXED32: return build_id(declare_atom("SFIXED32"));
  case FieldDescriptor::TYPE_SFIXED64: return build_id(declare_atom("SFIXED64"));
  case FieldDescriptor::TYPE_SINT32:   return build_id(declare_atom("SINT32"));
  case FieldDescriptor::TYPE_SINT64:   return build_id(declare_atom("SINT64"));
  default:  return build_id(declare_atom("UNKNOWN"));
  }
}


/** Check if message has a specific field.
 * This is relevant in particular for optional fields.
 * @param msgptr message
 * @param field_name name of the field
 * @return true if the field is present, false otherwise
 */
bool
OpenPRSProtobuf::oprs_pb_has_field(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return false;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return false;

  const Reflection *refl       = (*m)->GetReflection();

  if (field->is_repeated()) {
    return (refl->FieldSize(**m, field) > 0);
  } else {
    return refl->HasField(**m, field);
  }
}


/** Get a fields label.
 * @param msgptr message for which to get the field type
 * @param field_name name of the field
 * @return Term with Symbol, one of INVALID-MESSAGE, DOES-NOT-EXIST, OPTIONAL, REPEATED, UNKNOWN
 */
Term *
OpenPRSProtobuf::oprs_pb_field_label(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_id(declare_atom("INVALID-MESSAGE"));

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return build_id(declare_atom("DOES-NOT-EXIST"));
  switch (field->label()) {
  case FieldDescriptor::LABEL_OPTIONAL: return build_id(declare_atom("OPTIONAL"));
  case FieldDescriptor::LABEL_REQUIRED: return build_id(declare_atom("REQUIRED"));
  case FieldDescriptor::LABEL_REPEATED: return build_id(declare_atom("REPEATED"));
  default:                              return build_id(declare_atom("UNKNOWN"));
  }
}

/** Get properly typed field value.
 * @param msgptr message for which to get the field type
 * @param field_name name of the field
 * @return Term with value of proper type
 */
Term *
OpenPRSProtobuf::oprs_pb_field_value(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return build_id(declare_atom("INVALID-MESSAGE"));

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return build_id(declare_atom("DOES-NOT-EXIST"));
  const Reflection *refl       = (*m)->GetReflection();
  if (field->type() != FieldDescriptor::TYPE_MESSAGE && ! refl->HasField(**m, field)) {
    //logger_->log_warn("RefBox", "Field %s of %s not set",
    //	   field_name.c_str(), (*m)->GetTypeName().c_str());
    return build_id(declare_atom("NOT-SET"));
  }
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:   return build_float(refl->GetDouble(**m, field));
  case FieldDescriptor::TYPE_FLOAT:    return build_float(refl->GetFloat(**m, field));
  case FieldDescriptor::TYPE_INT64:    return build_long_long(refl->GetInt64(**m, field));
  case FieldDescriptor::TYPE_UINT64:
    return build_long_long((long int)refl->GetUInt64(**m, field));
  case FieldDescriptor::TYPE_INT32:    return build_integer(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_FIXED64:
    return build_long_long((long int)refl->GetUInt64(**m, field));
  case FieldDescriptor::TYPE_FIXED32:  return build_long_long(refl->GetUInt32(**m, field));
  case FieldDescriptor::TYPE_BOOL:     return refl->GetBool(**m, field) ? build_t() : build_nil();
  case FieldDescriptor::TYPE_STRING:   return build_string(refl->GetString(**m, field).c_str());
  case FieldDescriptor::TYPE_MESSAGE:
    {
      const google::protobuf::Message &mfield = refl->GetMessage(**m, field);
      google::protobuf::Message *mcopy = mfield.New();
      mcopy->CopyFrom(mfield);
      void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
      return build_pointer(ptr);
    }
  case FieldDescriptor::TYPE_BYTES:    return build_string((char *)"bytes");
  case FieldDescriptor::TYPE_UINT32:   return build_long_long(refl->GetUInt32(**m, field));
  case FieldDescriptor::TYPE_ENUM:
    return build_id(declare_atom(refl->GetEnum(**m, field)->name().c_str()));
  case FieldDescriptor::TYPE_SFIXED32: return build_integer(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_SFIXED64: return build_long_long(refl->GetInt64(**m, field));
  case FieldDescriptor::TYPE_SINT32:   return build_integer(refl->GetInt32(**m, field));
  case FieldDescriptor::TYPE_SINT64:   return build_long_long(refl->GetInt64(**m, field));
  default:
    throw std::logic_error("Unknown protobuf field type encountered");
  }
}


/** Set a field.
 * @param msgptr message for which to get the field type
 * @param field_name name of the field
 * @param value term which must contain a single properly typed value.
 */
void
OpenPRSProtobuf::oprs_pb_set_field(void *msgptr, std::string field_name, Term *value)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) return;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    //logger_->log_warn("RefBox", "Could not find field %s", field_name.c_str());
    return;
  }
  const Reflection *refl       = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      if (value->type == TT_FLOAT) {
	refl->SetDouble(m->get(), field, *(value->u.doubleptr));
      } else {
	throw std::logic_error(std::string("Invalid type, required float for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_FLOAT: 
      if (value->type == TT_FLOAT) {
	refl->SetFloat(m->get(), field, *(value->u.doubleptr));
      } else {
	throw std::logic_error(std::string("Invalid type, required float for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      if (value->type == INTEGER) {
	refl->SetInt64(m->get(), field, value->u.intval);
      } else if (value->type == LONG_LONG) {
	refl->SetInt64(m->get(), field, value->u.llintval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer or long long for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
      if (value->type == INTEGER) {
	refl->SetUInt64(m->get(), field, value->u.intval);
      } else if (value->type == LONG_LONG) {
	refl->SetUInt64(m->get(), field, value->u.llintval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer or long long for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
      if (value->type == INTEGER) {
	refl->SetInt32(m->get(), field, value->u.intval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_BOOL:
      if (value->type == TT_ATOM) {
	if (value->u.id == lisp_t_sym || value->u.id == nil_sym) {
	  refl->SetBool(m->get(), field, (value->u.id == lisp_t_sym));
	} else {
	  throw std::logic_error(std::string("Invalid value, allowed are T or NIL for field ") +
				 (*m)->GetTypeName() + field_name);
	}
      } else {
	throw std::logic_error(std::string("Invalid type, required symbol for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_STRING:
      if (value->type == STRING) {
	refl->SetString(m->get(), field, value->u.string);
      } else {
	throw std::logic_error(std::string("Invalid type, required string for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_MESSAGE:
      if (value->type == U_POINTER) {
	std::shared_ptr<google::protobuf::Message> *mfrom =
	  static_cast<std::shared_ptr<google::protobuf::Message> *>(value->u.u_pointer);
	Message *mut_msg = refl->MutableMessage(m->get(), field);
	mut_msg->CopyFrom(**mfrom);
	delete mfrom;
      } else {
	throw std::logic_error(std::string("Invalid type, required user pointer for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_BYTES:    break;
    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      if (value->type == INTEGER) {
	refl->SetUInt32(m->get(), field, value->u.intval);
      } else if (value->type == LONG_LONG) {
	refl->SetUInt32(m->get(), field, value->u.llintval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer or long long for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_ENUM:
      {
	const char *sym_name = NULL;
	if (value->type == TT_ATOM) {
	  sym_name = value->u.id;
	} else if (value->type == STRING) {
	  sym_name = value->u.string;
	} else {
	  throw std::logic_error(std::string("Invalid type, required symbol or string for ") +
				 (*m)->GetTypeName() + field_name);
	}

	const EnumDescriptor *enumdesc = field->enum_type();
	const EnumValueDescriptor *enumval = enumdesc->FindValueByName(sym_name);
	if (enumval) {
	  refl->SetEnum(m->get(), field, enumval);
	} else {
	  std::string sym_str(sym_name);
	  std::transform(sym_str.begin(), sym_str.end(), sym_str.begin(), std::ptr_fun<int, int>(std::toupper));

	  const EnumValueDescriptor *enumval = enumdesc->FindValueByName(sym_str);

	  if (enumval) {
	    refl->SetEnum(m->get(), field, enumval);
	  } else {
	    fprintf(stderr, "%s: cannot set invalid enum value '%s' (neither '%s') on '%s'",
		    (*m)->GetTypeName().c_str(), sym_name, sym_str.c_str(), field_name.c_str());
	  }
	}
      }
      break;

    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    //logger_->log_warn("RefBox", "Failed to set field %s of %s: %s", field_name.c_str(),
    //	   (*m)->GetTypeName().c_str(), e.what());
  }
}


/** Add value to a repeated field.
 * @param msgptr message
 * @param field_name name of the field
 * @param value term which must contain a single properly typed value.
 */
void
OpenPRSProtobuf::oprs_pb_add_list(void *msgptr, std::string field_name, Term *value)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m || *m)) return;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    //logger_->log_warn("RefBox", "Could not find field %s", field_name.c_str());
    return;
  }
  const Reflection *refl       = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      if (value->type == TT_FLOAT) {
	refl->AddDouble(m->get(), field, *(value->u.doubleptr));
      } else {
	throw std::logic_error(std::string("Invalid type, required float for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_FLOAT: 
      if (value->type == TT_FLOAT) {
	refl->AddFloat(m->get(), field, *(value->u.doubleptr));
      } else {
	throw std::logic_error(std::string("Invalid type, required float for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;

    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      if (value->type == INTEGER) {
	refl->AddInt64(m->get(), field, value->u.intval);
      } else if (value->type == LONG_LONG) {
	refl->AddInt64(m->get(), field, value->u.llintval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer or long long for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;

    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
      if (value->type == INTEGER) {
	refl->AddInt32(m->get(), field, value->u.intval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_BOOL:
      if (value->type == TT_ATOM) {
	if (value->u.id == lisp_t_sym || value->u.id == nil_sym) {
	  refl->AddBool(m->get(), field, (value->u.id == lisp_t_sym));
	} else {
	  throw std::logic_error(std::string("Invalid value, allowed are T or NIL for field ") +
				 (*m)->GetTypeName() + field_name);
	}
      } else {
	throw std::logic_error(std::string("Invalid type, required symbol for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_STRING:
      if (value->type == STRING) {
	refl->AddString(m->get(), field, value->u.string);
      } else {
	throw std::logic_error(std::string("Invalid type, required string for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;
    case FieldDescriptor::TYPE_MESSAGE:
      if (value->type == U_POINTER) {
	std::shared_ptr<google::protobuf::Message> *mfrom =
	  static_cast<std::shared_ptr<google::protobuf::Message> *>(value->u.u_pointer);
	Message *mut_msg = refl->AddMessage(m->get(), field);
	mut_msg->CopyFrom(**mfrom);
	delete mfrom;
      } else {
	throw std::logic_error(std::string("Invalid type, required user pointer for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;

    case FieldDescriptor::TYPE_BYTES:    break;

    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      if (value->type == INTEGER) {
	refl->AddUInt32(m->get(), field, value->u.intval);
      } else if (value->type == LONG_LONG) {
	refl->AddUInt32(m->get(), field, value->u.llintval);
      } else {
	throw std::logic_error(std::string("Invalid type, required integer or long long for ") +
			       (*m)->GetTypeName() + field_name);
      }
      break;

    case FieldDescriptor::TYPE_ENUM:
      {
	const char *sym_name = NULL;
	if (value->type == TT_ATOM) {
	  sym_name = value->u.id;
	} else if (value->type == STRING) {
	  sym_name = value->u.string;
	} else {
	  throw std::logic_error(std::string("Invalid type, required symbol or string for ") +
				 (*m)->GetTypeName() + field_name);
	}
	const EnumDescriptor *enumdesc = field->enum_type();
	const EnumValueDescriptor *enumval = enumdesc->FindValueByName(sym_name);
	if (enumval) {
	  refl->AddEnum(m->get(), field, enumval);
	} else {
	  //logger_->log_warn("RefBox", "%s: cannot set invalid enum value '%s' on '%s'",
	  //	 (*m)->GetTypeName().c_str(), value.as_string().c_str(), field_name.c_str());
	}
      }
      break;

    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    //logger_->log_warn("RefBox", "Failed to add field %s of %s: %s", field_name.c_str(),
    //	   (*m)->GetTypeName().c_str(), e.what());
  }
}


/** Connect as a client to the given server.
 * Note that this will perform an asynchronous connect. A
 * (protobuf-client-connected) or (protobuf-client-disconnected) fact
 * is asserted during (pb-process) in the case of success or failure.
 * @param host host to connect to
 * @param port TCP port to connect to
 * @return Term with a long long of the client ID
 */
Term *
OpenPRSProtobuf::oprs_pb_client_connect(std::string host, int port)
{
  if (port <= 0) return build_nil();

  ProtobufStreamClient *client = new ProtobufStreamClient(message_register_);

  long int client_id;
  {
    fawkes::MutexLocker lock(&map_mutex_);
    client_id = ++next_client_id_;
    clients_[client_id] = client;
  }

  client->signal_connected().connect(
    boost::bind(&OpenPRSProtobuf::handle_client_connected, this, client_id));
  client->signal_disconnected().connect(
    boost::bind(&OpenPRSProtobuf::handle_client_disconnected,
		this, client_id, boost::asio::placeholders::error));
  client->signal_received().connect(
    boost::bind(&OpenPRSProtobuf::handle_client_msg, this, client_id, _1, _2, _3));
  client->signal_receive_failed().connect(
    boost::bind(&OpenPRSProtobuf::handle_client_receive_fail, this, client_id, _1, _2, _3));

  client->async_connect(host.c_str(), port);
  return build_long_long(client_id);
}


/** Send message to a specific client.
 * @param client_id ID of the client, this can be a server client ID, a client
 * ID, or a peer ID (message will then be broadcasted).
 * @param msgptr message to send
 */
void
OpenPRSProtobuf::oprs_pb_send(long int client_id, void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m || *m)) {
    //logger_->log_warn("RefBox", "Cannot send to %li: invalid message", client_id);
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
    //logger_->log_warn("RefBox", "Failed to send message of type %s: %s",
    //     (*m)->GetTypeName().c_str(), e.what());
  } catch (std::runtime_error &e) {
    //logger_->log_warn("RefBox", "Failed to send message of type %s: %s",
    //     (*m)->GetTypeName().c_str(), e.what());
  }
}


/** Broadcast a message through a peer.
 * @param peer_id ID broadcast peer to send through
 * @param msgptr message to send
 */
void
OpenPRSProtobuf::oprs_pb_broadcast(long int peer_id, void *msgptr)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m || *m)) {
    fprintf(stderr, "Cannot send broadcast: invalid message");
    return;
  }

  fawkes::MutexLocker lock(&map_mutex_);
  if (peers_.find(peer_id) == peers_.end())  return;

  try {
    peers_[peer_id]->send(*m);
  } catch (google::protobuf::FatalException &e) {
    fprintf(stderr, "pb-broadcast: failed to broadcast message of type %s: %s\n",
	    (*m)->GetTypeName().c_str(), e.what());
  }

  sig_peer_sent_(peer_id, *m);
}


/** Disconnect a given client.
 * @param client_id ID of client to disconnect, can be a server client ID or a client ID
 */
void
OpenPRSProtobuf::oprs_pb_disconnect(long int client_id)
{
  //logger_->log_info("RefBox", "Disconnecting client %li", client_id);

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
    throw fawkes::Exception("Failed to disconnect from client %li: %s", client_id, e.what());
  }
}


/** Get list of values of a given message field.
 * @param msgptr message
 * @param field_name field to retrieve
 * @return term which contains a Lisp list with properly typed values, or a symbol in
 * case of an error
 */
Term *
OpenPRSProtobuf::oprs_pb_field_list(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m || *m)) return build_id(declare_atom("INVALID-MESSAGE"));

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field)  return build_id(declare_atom("DOES-NOT-EXIST"));

  TermList tl = sl_make_slist();

  if (field->label() == FieldDescriptor::LABEL_REQUIRED ||
      field->label() == FieldDescriptor::LABEL_OPTIONAL)
  {
    tl = build_term_list(tl, oprs_pb_field_value(msgptr, field_name));
    return build_term_l_list_from_c_list(tl);
  }

  const Reflection *refl       = (*m)->GetReflection();
  int field_size = refl->FieldSize(**m, field);
  for (int i = 0; i < field_size; ++i) {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      tl = build_term_list(tl, build_float(refl->GetRepeatedDouble(**m, field, i)));
      break;
    case FieldDescriptor::TYPE_FLOAT:
      tl = build_term_list(tl, build_float(refl->GetRepeatedFloat(**m, field, i)));
      break;
      break;
    case FieldDescriptor::TYPE_UINT64:
    case FieldDescriptor::TYPE_FIXED64:
      tl = build_term_list(tl, build_long_long(refl->GetRepeatedUInt64(**m, field, i)));
      break;
    case FieldDescriptor::TYPE_UINT32:
    case FieldDescriptor::TYPE_FIXED32:
      tl = build_term_list(tl, build_long_long(refl->GetRepeatedUInt32(**m, field, i)));
      break;
    case FieldDescriptor::TYPE_BOOL:
      tl = build_term_list(tl, refl->GetRepeatedBool(**m, field, i) ? build_t() : build_nil());
      break;
    case FieldDescriptor::TYPE_STRING:
      tl = build_term_list(tl, build_string(refl->GetRepeatedString(**m, field, i).c_str()));
      break;
    case FieldDescriptor::TYPE_MESSAGE:
      {
	const google::protobuf::Message &msg = refl->GetRepeatedMessage(**m, field, i);
	google::protobuf::Message *mcopy = msg.New();
	mcopy->CopyFrom(msg);
	void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
	tl = build_term_list(tl, build_pointer(ptr));
      }
      break;
    case FieldDescriptor::TYPE_BYTES:
      tl = build_term_list(tl, build_string((char *)"bytes"));
      break;
    case FieldDescriptor::TYPE_ENUM:
      tl = build_term_list(tl, build_id(declare_atom(refl->GetRepeatedEnum(**m, field, i)->name().c_str())));
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_INT32:
    case FieldDescriptor::TYPE_SINT32:
      tl = build_term_list(tl, build_integer(refl->GetRepeatedInt32(**m, field, i)));
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      tl = build_term_list(tl, build_long_long(refl->GetRepeatedInt64(**m, field, i)));
      break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  }

  return build_term_l_list_from_c_list(tl);
}


/** Check if a given field is a list (repeated field).
 * @param msgptr message
 * @param field_name name of the field
 * @return true if the field is a list, false otherwise
 */
bool
OpenPRSProtobuf::oprs_pb_field_is_list(void *msgptr, std::string field_name)
{
  std::shared_ptr<google::protobuf::Message> *m =
    static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m || *m)) return false;

  const Descriptor *desc       = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (! field) {
    return false;
  }
  return (field->label() == FieldDescriptor::LABEL_REPEATED);
}


/** Process all pending events.
 * This will process events and assert appropriate facts.
 */
void
OpenPRSProtobuf::oprs_pb_process()
{
  { fawkes::MutexLocker lock(q_server_client_.mutex());
    while (! q_server_client_.empty()) {
      auto &sc = q_server_client_.front();
      oprs_assert_server_client_event(std::get<0>(sc), std::get<1>(sc),
				      std::get<2>(sc), std::get<3>(sc));
      q_server_client_.pop();
    }
  }

  { fawkes::MutexLocker lock(q_client_.mutex());
    while (! q_client_.empty()) {
      auto &c = q_client_.front();
      oprs_assert_client_event(std::get<0>(c), std::get<1>(c));
      q_client_.pop();
    }
  }

  { fawkes::MutexLocker lock(q_msgs_.mutex());
    while (! q_msgs_.empty()) {
      auto &m = q_msgs_.front();
      oprs_assert_message(std::get<0>(m), std::get<1>(m), std::get<2>(m),
			  std::get<3>(m), std::get<4>(m), std::get<5>(m), std::get<6>(m));
      q_msgs_.pop();
    }
  }
}


/** Check if there are pending events.
 * @return true if there are pending events, false otherwise
 */
bool
OpenPRSProtobuf::oprs_pb_events_pending()
{
  fawkes::MutexLocker lock1(q_server_client_.mutex());
  fawkes::MutexLocker lock2(q_client_.mutex());
  fawkes::MutexLocker lock3(q_msgs_.mutex());

  return (! (q_server_client_.empty() && q_client_.empty() && q_msgs_.empty()));
}


void
OpenPRSProtobuf::oprs_assert_server_client_event(long int client_id,
						 std::string &host, unsigned short port, bool connect)
{
  TermList tl = sl_make_slist();
  tl = build_term_list(tl, build_long_long(client_id));
  if (connect) {
    tl = build_term_list(tl, build_string(host.c_str()));
    tl = build_term_list(tl, build_integer(port));
    add_external_fact((char *)"protobuf-server-client-connected", tl);
  } else {
    add_external_fact((char *)"protobuf-server-client-disconnected", tl);
  }
}


void
OpenPRSProtobuf::oprs_assert_client_event(long int client_id, bool connect)
{
  TermList tl = sl_make_slist();
  tl = build_term_list(tl, build_long_long(client_id));
  if (connect) {
    add_external_fact((char *)"protobuf-client-connected", tl);
  } else {
    add_external_fact((char *)"protobuf-client-disconnected", tl);
  }
}

void
OpenPRSProtobuf::oprs_assert_message(std::string &endpoint_host, unsigned short endpoint_port,
				     uint16_t comp_id, uint16_t msg_type,
				     std::shared_ptr<google::protobuf::Message> &msg,
				     OpenPRSProtobuf::ClientType ct,
				     unsigned int client_id)
{
  TermList tl = sl_make_slist();

  struct timeval tv;
  gettimeofday(&tv, 0);
  void *ptr = new std::shared_ptr<google::protobuf::Message>(msg);
  //tl = build_term_list(tl, build_string((char *)"type"));
  tl = build_term_list(tl, build_string(msg->GetTypeName().c_str()));
  //tl = build_term_list(tl, build_string((char *)"comp-id"));
  tl = build_term_list(tl, build_integer(comp_id));
  //tl = build_term_list(tl, build_string((char *)"msg-type"));
  tl = build_term_list(tl, build_integer(msg_type));
  //tl = build_term_list(tl, build_string((char *)"rcvd-via"));
  tl = build_term_list(tl, build_string((client_id == 0) ? "BROADCAST" : "STREAM"));
  //tl = build_term_list(tl, build_string((char *)"rcvd-at"));
  tl = build_term_list(tl, build_long_long(tv.tv_sec));
  tl = build_term_list(tl, build_long_long(tv.tv_usec));
  //tl = build_term_list(tl, build_string((char *)"rcvd-from"));
  tl = build_term_list(tl, build_string(endpoint_host.c_str()));
  tl = build_term_list(tl, build_integer(endpoint_port));
  //tl = build_term_list(tl, build_string((char *)"client-type"));
  tl = build_term_list(tl, build_string(ct == CT_CLIENT ? "CLIENT" :
					(ct == CT_SERVER ? "SERVER" : "PEER")));
  //tl = build_term_list(tl, build_string((char *)"client-id"));
  tl = build_term_list(tl, build_integer(client_id));
  //tl = build_term_list(tl, build_string((char *)"ptr"));
  tl = build_term_list(tl, build_pointer(ptr));

  add_external_fact((char *)"protobuf-msg", tl);
}

void
OpenPRSProtobuf::handle_server_client_connected(ProtobufStreamServer::ClientID client,
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

  q_server_client_.push_locked(
    std::make_tuple(client_id, endpoint.address().to_string(), endpoint.port(), true));
}


void
OpenPRSProtobuf::handle_server_client_disconnected(ProtobufStreamServer::ClientID client,
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
    q_server_client_.push_locked(std::make_tuple(client_id, "", 0, false));
  }
}


/** Handle message that came from a client.
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
OpenPRSProtobuf::handle_server_client_msg(ProtobufStreamServer::ClientID client,
						    uint16_t component_id, uint16_t msg_type,
						    std::shared_ptr<google::protobuf::Message> msg)
{
  fawkes::MutexLocker lock(&map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    q_msgs_.push_locked(std::make_tuple(client_endpoints_[c->second].first,
					client_endpoints_[c->second].second,
					component_id, msg_type, msg, CT_SERVER, c->second));
  }
}

/** Handle server reception failure
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message string
 */
void
OpenPRSProtobuf::handle_server_client_fail(ProtobufStreamServer::ClientID client,
						     uint16_t component_id, uint16_t msg_type,
						     std::string msg)
{
  fawkes::MutexLocker lock(&map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    /*
    fawkes::MutexLocker lock(&oprs_mutex_);
    oprs_->assert_fact_f("(protobuf-server-receive-failed (comp-id %u) (msg-type %u) "
			  "(rcvd-via STREAM) (client-id %li) (message \"%s\") "
			  "(rcvd-from (\"%s\" %u)))",
			  component_id, msg_type, c->second, msg.c_str(),
			  client_endpoints_[c->second].first.c_str(),
			  client_endpoints_[c->second].second);
    */
  }
}


/** Handle message that came from a peer/robot
 * @param endpoint the endpoint from which the message was received
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
OpenPRSProtobuf::handle_peer_msg(long int peer_id,
					   boost::asio::ip::udp::endpoint &endpoint,
					   uint16_t component_id, uint16_t msg_type,
					   std::shared_ptr<google::protobuf::Message> msg)
{
  q_msgs_.push_locked(std::make_tuple(endpoint.address().to_string(), endpoint.port(),
				      component_id, msg_type, msg, CT_PEER, peer_id));
}


/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
OpenPRSProtobuf::handle_peer_recv_error(long int peer_id,
					boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
  fprintf(stderr, "Failed to receive peer message from %s:%u: %s\n",
	  endpoint.address().to_string().c_str(), endpoint.port(), msg.c_str());
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
OpenPRSProtobuf::handle_peer_send_error(long int peer_id, std::string msg)
{
  //logger_->log_warn("RefBox", "Failed to send peer message: %s", msg.c_str());
}


void
OpenPRSProtobuf::handle_client_connected(long int client_id)
{
  q_client_.push_locked(std::make_tuple(client_id, true));
}

void
OpenPRSProtobuf::handle_client_disconnected(long int client_id,
					    const boost::system::error_code &error)
{
  q_client_.push_locked(std::make_tuple(client_id, false));
}

void
OpenPRSProtobuf::handle_client_msg(long int client_id,
				   uint16_t comp_id, uint16_t msg_type,
				   std::shared_ptr<google::protobuf::Message> msg)
{
  q_msgs_.push_locked(std::make_tuple("", 0, comp_id, msg_type, msg, CT_CLIENT, client_id));
}


void
OpenPRSProtobuf::handle_client_receive_fail(long int client_id,
						      uint16_t comp_id, uint16_t msg_type, std::string msg)
{
  /*
  oprs_->assert_fact_f("(protobuf-receive-failed (client-id %li) (rcvd-via STREAM) "
			"(comp-id %u) (msg-type %u) (message \"%s\"))",
			client_id, comp_id, msg_type, msg.c_str());
  */
}

} // end namespace protobuf_clips
