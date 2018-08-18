
/***************************************************************************
 *  protobuf_adapter.cpp - PLEXIL adapter for protobuf_comm
 *
 *  Created: Thu Aug 16 11:06:55 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "protobuf_adapter.h"

#include "utils.h"

#include <utils/misc/string_split.h>

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>
#include <InterfaceManager.hh>
#include <StateCacheEntry.hh>

#include <algorithm>
#include <chrono>

using namespace fawkes;
using namespace protobuf_comm;
using namespace google::protobuf;


/** @class ProtobufCommPlexilAdapter "protobuf_adapter.h"
 * Plexil adapter to provide access to protobuf_comm.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
ProtobufCommPlexilAdapter::ProtobufCommPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
ProtobufCommPlexilAdapter::ProtobufCommPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                                     pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
ProtobufCommPlexilAdapter::~ProtobufCommPlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
ProtobufCommPlexilAdapter::initialize()
{
	logger_  = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));
	config_  = reinterpret_cast<fawkes::Configuration *>(m_execInterface.getProperty("::Fawkes::Config"));
	clock_   = reinterpret_cast<fawkes::Clock *>(m_execInterface.getProperty("::Fawkes::Clock"));

	std::string cfg_prefix;
	try {
		std::string cfg_spec = config_->get_string("/plexil/spec");
		cfg_prefix = "/plexil/" + cfg_spec + "/";
	} catch (fawkes::Exception &e) {
		logger_->log_error("PlexilProtobuf", "Failed to read config: %s", e.what_no_backtrace());
		return false;
	}

	std::string cfg_proto_dir;

	pugi::xml_node config = getXml();
	pugi::xml_attribute xml_attr = config.attribute("protos");
	if (xml_attr) {
		cfg_proto_dir = xml_attr.value();
	} else {
		for (const auto &c : config.children()) {
			if (strcmp(c.name(), "Parameter") == 0) {
				pugi::xml_attribute xml_key_attr = c.attribute("key");
				if (xml_key_attr && strcmp(xml_key_attr.value(), "protos") == 0) {
					cfg_proto_dir = c.text().get();
				}
			}
		}
	}
	replace_tokens(cfg_proto_dir);

	std::vector<std::string> cfg_proto_dirs{cfg_proto_dir};
	logger_->log_info("PlexilProtobuf", "Protobuf message spec dir: %s", cfg_proto_dirs[0].c_str());
	
	msg_counter_ = 0;
	next_client_id_ = 0;

	message_register_ = std::make_shared<MessageRegister>(cfg_proto_dirs);

	namespace p = std::placeholders;
	commands_ = {
	  {"ReceiveCommand",  std::bind(&ProtobufCommPlexilAdapter::proc_receive_command, this, p::_1)},
	  {"GetParameter",    std::bind(&ProtobufCommPlexilAdapter::proc_get_param_command, this, p::_1)},
	  {"SendReturnValue", std::bind(&ProtobufCommPlexilAdapter::proc_send_rv_command, this, p::_1)},
	  {"pb_create",       std::bind(&ProtobufCommPlexilAdapter::pb_create, this, p::_1)},
	  {"pb_destroy",      std::bind(&ProtobufCommPlexilAdapter::pb_destroy, this, p::_1)},
	  {"pb_set_value",    std::bind(&ProtobufCommPlexilAdapter::pb_set_value, this, p::_1)},
	  {"pb_get_int",      std::bind(&ProtobufCommPlexilAdapter::pb_get_value, this, p::_1, PLEXIL::INTEGER_TYPE)},
	  {"pb_get_real",     std::bind(&ProtobufCommPlexilAdapter::pb_get_value, this, p::_1, PLEXIL::REAL_TYPE)},
	  {"pb_get_bool",     std::bind(&ProtobufCommPlexilAdapter::pb_get_value, this, p::_1, PLEXIL::BOOLEAN_TYPE)},
	  {"pb_get_string",   std::bind(&ProtobufCommPlexilAdapter::pb_get_value, this, p::_1, PLEXIL::STRING_TYPE)},
	  {"pb_get_length",   std::bind(&ProtobufCommPlexilAdapter::pb_get_length, this, p::_1)},
	  {"pb_broadcast",    std::bind(&ProtobufCommPlexilAdapter::pb_broadcast, this, p::_1)},
	  {"pb_tostring",     std::bind(&ProtobufCommPlexilAdapter::pb_tostring, this, p::_1)},
	  {"pb_peer_create",  std::bind(&ProtobufCommPlexilAdapter::pb_peer_create, this, p::_1)},
	  {"pb_peer_destroy", std::bind(&ProtobufCommPlexilAdapter::pb_get_length, this, p::_1)},
	  {"pb_peer_create_local",
	   std::bind(&ProtobufCommPlexilAdapter::pb_peer_create_local, this, p::_1)},
	  {"pb_peer_create_crypto",
	   std::bind(&ProtobufCommPlexilAdapter::pb_peer_create_crypto, this, p::_1)},
	  {"pb_peer_create_local_crypto",
	   std::bind(&ProtobufCommPlexilAdapter::pb_peer_create_local_crypto, this, p::_1, nullptr)},
	  {"pb_peer_setup_crypto",
	   std::bind(&ProtobufCommPlexilAdapter::pb_peer_setup_crypto, this, p::_1)},
	};

	for (const auto &c: commands_) {
		PLEXIL::g_configuration->registerCommandInterface(c.first, this);
	}

	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
ProtobufCommPlexilAdapter::start()
{
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
ProtobufCommPlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
ProtobufCommPlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
ProtobufCommPlexilAdapter::shutdown()
{
	return true;
}

/** Perform given command.
 * @param cmd command to execute
 */
void
ProtobufCommPlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	std::string const &name = cmd->getName();

	//logger_->log_info("PlexilProtobuf", "Processing %s", name.c_str());

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("ProtobufCommAdapter:executeCommand: called for unknown"
		     " command " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
}

void
ProtobufCommPlexilAdapter::proc_receive_command(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (args.size() != 1) {
		warn("ProtobufCommAdapter: The ReceiveCommand"
		     " command requires exactly one argument");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (args.front().valueType() != PLEXIL::STRING_TYPE) {
		warn("ProtobufCommAdapter: The argument to the ReceiveCommand"
		     " command, " << args.front() << ", is not a string");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string msg_type;
	if (!args.front().getValue(msg_type)) {
		warn("ProtobufCommAdapter: message type is unknown");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	add_recipient(msg_type, cmd);
	proc_queue(msg_type);

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SENT_TO_SYSTEM);
	m_execInterface.notifyOfExternalEvent();
}

void
ProtobufCommPlexilAdapter::proc_get_param_command(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (args.size() < 1 || args.size() > 2) {
		warn("ProtobufCommAdapter:GetParameter: "
		     "Command requires either one or two arguments");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (args.front().valueType() != PLEXIL::STRING_TYPE) {
		warn("ProtobufCommAdapter:GetParameter: first argument "
		     " '" << args.front() << "' is not a string");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string msg_id;
	if (!args.front().getValue(msg_id)) {
		warn("ProtobufCommAdapter:GetParameter: message ID is unknown");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:GetParameter: message ID not in queued messages");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::vector<PLEXIL::Value>::const_iterator it = ++args.begin();
	int32_t id = 0;
	if (it != args.end()) {
		if (it->valueType() != PLEXIL::INTEGER_TYPE) {
			warn("ProtobufCommAdapter:GetParameter: second argument "
			     " '" << *it << "' is not an integer");
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}

		if (!it->getValue(id)) {
			warn("ProtobufCommAdapter:GetParameter: second argument is unknown");
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}

		if (id < 0 || id > 3) {
			warn("ProtobufCommAdapter:GetParameter: second argument "
			     " '" << *it << "' is not a valid index (must be 0..3)");
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}
	}

	const message_meta &msgmeta = messages_[msg_id];
	
	switch (id) {
	case 0: m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(msg_id)); break;
	case 1: m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(msgmeta.from_host)); break;
	case 2: m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(msgmeta.from_port)); break;
	case 3: m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(msgmeta.time_received.in_sec())); break;
	}

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
ProtobufCommPlexilAdapter::proc_send_rv_command(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (args.size() == 0) {
		warn("ProtobufCommAdapter:SendReturnValue: The SendReturnValue"
		     " command requires at least one argument");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (args.front().valueType() != PLEXIL::STRING_TYPE) {
		warn("ProtobufCommAdapter:SendReturnValue: The argument to the "
		     "SendReturnValue command, " << args.front() << ", is not a string");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string msg_id;
	if (!args.front().getValue(msg_id)) {
		warn("ProtobufCommAdapter:SendReturnValue: message ID is unknown");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	release_message(msg_id);

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

/** Abort currently running execution.
 * @param cmd command to abort
 */
void
ProtobufCommPlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
	m_execInterface.handleCommandAbortAck(cmd, false);
	m_execInterface.notifyOfExternalEvent();
}


void
ProtobufCommPlexilAdapter::add_recipient(const std::string& msg_type,
                                         PLEXIL::Command* cmd)
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_entry& q = get_queue(msg_type);
	q.recipients.push_back(cmd);
}

void
ProtobufCommPlexilAdapter::remove_recipient(const std::string& msg_type,
                                            PLEXIL::Command* cmd)
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_entry& q = get_queue(msg_type);
	std::remove(q.recipients.begin(), q.recipients.end(), cmd);	
}

void
ProtobufCommPlexilAdapter::add_message(const std::string& msg_type,
                                       message_meta&& msg)
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_entry& q = get_queue(msg_type);
	std::string msg_id = gen_msgid(msg_type);

	messages_[msg_id] = std::move(msg);

	q.messages.push_back(msg_id);
}


void
ProtobufCommPlexilAdapter::release_message(const std::string& msg_id)
{
	std::lock_guard<std::mutex> lock(queue_mutex_);

	std::string::size_type colon_pos = msg_id.find(':');
	if (colon_pos != std::string::npos) {
		std::string msg_type{msg_id.substr(0, colon_pos)};
		queue_entry& q = get_queue(msg_type);

		std::remove(q.messages.begin(), q.messages.end(), msg_id);
		messages_.erase(msg_id);
	}
}

std::string
ProtobufCommPlexilAdapter::gen_msgid(const std::string& msg_type)
{
	unsigned long int id = ++msg_counter_;
	return msg_type + ":" + std::to_string(id);
}

void
ProtobufCommPlexilAdapter::proc_queue(const std::string& msg_type)
{
	std::lock_guard<std::mutex> lock(queue_mutex_);
	queue_entry &q = get_queue(msg_type);
	auto mi = q.messages.begin();
	auto ri = q.recipients.begin();
	bool notify = ! q.messages.empty() && ! q.recipients.empty();
	while (mi != q.messages.end() && ri != q.recipients.end()) {
		m_execInterface.handleCommandReturn(*ri, *mi);
		ri = q.recipients.erase(ri);
		mi = q.messages.erase(mi);
	}
	if (notify) {
		m_execInterface.notifyOfExternalEvent();
	}
}


ProtobufCommPlexilAdapter::queue_entry &
ProtobufCommPlexilAdapter::get_queue(const std::string& msg_type)
{
	auto q = queue_.find(msg_type);
	if (q != queue_.end()) {
		return q->second;
	} else {
		auto new_q = queue_.insert(std::make_pair(msg_type, queue_entry()));
		return new_q.first->second;
	}
}


bool
ProtobufCommPlexilAdapter::verify_args(const std::vector<PLEXIL::Value> &args, const std::string& func,
                                       const std::vector<std::pair<std::string, PLEXIL::ValueType>> &types)
{
	if (args.size() != types.size()) {
		warn("ProtobufCommAdapter:" << func << ":"
		     << "Command requires " << types.size() << " arguments, got " << args.size());
		for (size_t i = 0; i < args.size(); ++i) {
			warn("ProtobufCommAdapter:" << func << ":"
			     << " Argument " << i << " = " << args[i]);
		}
		return false;
	}
	for (size_t i = 0; i < args.size(); ++i) {
		// Treat UNKNOWN_TYPE as "any type and we don't care/inspect later"
		if (types[i].second == PLEXIL::UNKNOWN_TYPE) continue;

		if (args[i].valueType() != types[i].second) {
			warn("ProtobufCommAdapter:" << func << ":"
			     << "Command argument " << i << "(" << types[i].first << ") expected to be of type "
			     << PLEXIL::valueTypeName(types[i].second) << ", but is of type "
			     << PLEXIL::valueTypeName(args[i].valueType()));
			return false;
		}
	}
	return true;
}

/** Create Protobuf message.
 */
void
ProtobufCommPlexilAdapter::pb_create(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_create",
	                  {{"msg_type", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string msg_type;
	args[0].getValue(msg_type);

	std::lock_guard<std::mutex> lock(queue_mutex_);
	std::string msg_id = gen_msgid(msg_type);

  try {
    std::shared_ptr<google::protobuf::Message> m =
      message_register_->new_message_for(msg_type);

    message_meta msgmeta{
      .time_received = fawkes::Time(clock_),
      .from_host     = "",
      .from_port     = 0,
      .message       = std::shared_ptr<google::protobuf::Message>(m)
    };
    
    messages_[msg_id] = std::move(msgmeta);

    m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(msg_id));
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
  } catch (std::runtime_error &e) {
	  logger_->log_warn("PlexilProtobuf", "Cannot create message of type %s: %s",
	                    msg_type.c_str(), e.what());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
  }
}

/** Destroy Protobuf message.
 */
void
ProtobufCommPlexilAdapter::pb_destroy(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_destroy",
	                  {{"msg_id", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string msg_id;
	args[0].getValue(msg_id);

	std::lock_guard<std::mutex> lock(queue_mutex_);

	if (messages_.find(msg_id) != messages_.end()) {
		messages_.erase(msg_id);
	}
}

static std::pair<std::string, long int>
parse_field_name(const std::string& field_name, const std::string& func)
{
	std::string field;
	long int index = -1;

	std::string::size_type opening = field_name.find('[');
	if (opening != std::string::npos) {
		std::string::size_type closing = field_name.find(']', opening);
		if (closing != std::string::npos) {
			field = field_name.substr(0, opening);
			std::string idx_str = field_name.substr(opening+1, closing-opening-1);
			if (idx_str.empty()) {
				index = std::numeric_limits<long int>::max();
			} else {
				index = std::stol(idx_str);
			}
		} else {
			warn("ProtobufCommAdapter:" << func << ":"
			     << " Missing closing bracket in " << field_name);
		}
	} else {
		field = field_name;
	}
	return std::make_pair(field, index);
}

static bool
traverse_field(google::protobuf::Message* &  msg,
               const std::string &           field_name,
               const FieldDescriptor* &      field,
               std::string &                 partial_name,
               long int &                    partial_index,
               const std::string &           func)

{
  std::vector<std::string> field_path = str_split(field_name, '.');
  for (size_t i = 0; i < field_path.size(); ++i) {
	  std::pair<std::string, long int> parsed_field =
	    parse_field_name(field_path[i], func);
	  partial_name  = parsed_field.first;
	  partial_index = parsed_field.second;

	  if (partial_name.empty()) {
		  warn("ProtobufCommAdapter:" << func << ":"
		       << " Invalid sub-field " << field_path[i]);		  
		  return false;
	  }

	  const Descriptor *desc = msg->GetDescriptor();
	  field = desc->FindFieldByName(partial_name);
	  if (partial_index >= 0 && ! field->is_repeated()) {
		  warn("ProtobufCommAdapter:" << func << ":"
		       << " Index for non-repeated field " << partial_name);		  
		  return false;
			  
	  } else if (partial_index < 0 && field->is_repeated()) {
		  warn("ProtobufCommAdapter:" << func << ":"
		       << " No index for repeated field " << partial_name);
		  return false;
	  }

	  const Reflection *refl = msg->GetReflection();

	  // special case: if someone tries to access the array just one
	  // behind the last boundary, treat this as adding an element.
	  // this makes consecutive accesses to the same just added value
	  // more natural, e.g., my_field[0].value1 and then my_field[0].value2.
	  if (field->is_repeated() && partial_index >= 0 && partial_index == refl->FieldSize(*msg, field))
	  {
		  partial_index = std::numeric_limits<long int>::max();
	  }

	  if (partial_index >= 0 && partial_index < std::numeric_limits<long int>::max() &&
	      partial_index >= refl->FieldSize(*msg, field))
	  {
		  warn("ProtobufCommAdapter:" << func << ":"
		       << " Index " << partial_index << " out of bounds for " << partial_name);
		  return false;
	  }

	  if (i == (field_path.size() - 1)) {
		  if (field->type() == FieldDescriptor::TYPE_MESSAGE) {
			  warn("ProtobufCommAdapter:" << func << ":"
			       << " Final sub-field " << field_path[i] << " is a message");
			  return false;
		  }
	  } else {
		  if (field->type() != FieldDescriptor::TYPE_MESSAGE) {
			  warn("ProtobufCommAdapter:" << func << ":"
			       << " Sub-field " << field_path[i] << " is not a message");
			  return false;
		  }

		  if (field->is_repeated()) {
			  if (partial_index == std::numeric_limits<long int>::max()) {
				  msg = refl->AddMessage(msg, field);
			  } else {
				  // out of bounds check already done above
				  msg = refl->MutableRepeatedMessage(msg, field, partial_index);
			  }
		  } else {
			  msg = refl->MutableMessage(msg, field);
		  }
	  }
  }
  return true;
}


void
ProtobufCommPlexilAdapter::pb_set_value(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_set_value",
	                  {{"msg_id", PLEXIL::STRING_TYPE},
	                   {"field", PLEXIL::STRING_TYPE},
	                   {"value", PLEXIL::UNKNOWN_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   msg_id;
	std::string   field_name;
	PLEXIL::Value value;
	args[0].getValue(msg_id);
	args[1].getValue(field_name);
	value = args[2];

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:pb_set_value:"
		     << " Unknown message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	message_meta &msgmeta = messages_[msg_id];
	std::shared_ptr<google::protobuf::Message> m = msgmeta.message;

  const FieldDescriptor *field = nullptr;
  google::protobuf::Message *msg= m.get();
   
  std::string partial_name;
  long int    partial_index = -1;

  if (! traverse_field(msg, field_name, field, partial_name, partial_index, "pb_set_value")) {
	  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	  m_execInterface.notifyOfExternalEvent();
	  return;
  }
  
  //field = desc->FindFieldByName(field_path.back());

  if (! field) {
		warn("ProtobufCommAdapter:pb_set_value:"
		     << " Unknown field " << field_name << " for message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }
  const Reflection *refl       = msg->GetReflection();

  bool add_repeated = false;
  if (field->is_repeated() && partial_index == std::numeric_limits<long int>::max()) {
	  add_repeated = true;
  }

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
	    if (value.valueType() != PLEXIL::REAL_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Real, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    double v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddDouble(msg, field, v);
			    } else {
				    refl->SetRepeatedDouble(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetDouble(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_FLOAT:
	    if (value.valueType() != PLEXIL::REAL_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Real, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    double v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddFloat(msg, field, v);
			    } else {
				    refl->SetRepeatedFloat(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetFloat(msg, field, v);
		    }

	    }
	    break;

    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
	    if (value.valueType() != PLEXIL::INTEGER_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Integer, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    int v;
		    value.getValue(v);
		    refl->SetInt64(msg, field, v);
	    }
	    break;

    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
	    if (value.valueType() != PLEXIL::INTEGER_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Integer, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    int v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddUInt64(msg, field, v);
			    } else {
				    refl->SetRepeatedUInt64(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetUInt64(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
	    if (value.valueType() != PLEXIL::INTEGER_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Integer, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    int v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddInt32(msg, field, v);
			    } else {
				    refl->SetRepeatedInt32(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetInt32(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_BOOL:
	    if (value.valueType() != PLEXIL::BOOLEAN_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Boolean, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    bool v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddBool(msg, field, v);
			    } else {
				    refl->SetRepeatedBool(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetBool(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_STRING:
	    if (value.valueType() != PLEXIL::STRING_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects String, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    std::string v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddString(msg, field, v);
			    } else {
				    refl->SetRepeatedString(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetString(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_MESSAGE:
	    // does not occur with dotted access
	    return;

    case FieldDescriptor::TYPE_BYTES:
	    warn("ProtobufCommAdapter:pb_set_value: cannot set byte field.");
	    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	    m_execInterface.notifyOfExternalEvent();
	    break;

    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
	    if (value.valueType() != PLEXIL::INTEGER_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects Integer, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    int v;
		    value.getValue(v);
		    if (field->is_repeated()) {
			    if (add_repeated) {
				    refl->AddUInt32(msg, field, v);
			    } else {
				    refl->SetRepeatedUInt32(msg, field, partial_index, v);
			    }
		    } else {
			    refl->SetUInt32(msg, field, v);
		    }
	    }
	    break;

    case FieldDescriptor::TYPE_ENUM:
	    if (value.valueType() != PLEXIL::STRING_TYPE) {
		    warn("ProtobufCommAdapter:pb_set_value:"
		         "Invalid type for field " << field_name << ", expects String, got "
		         << PLEXIL::valueTypeName(value.valueType()));
		    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		    m_execInterface.notifyOfExternalEvent();
		    return;
	    } else {
		    std::string v;
		    value.getValue(v);

		    const EnumDescriptor *enumdesc = field->enum_type();
		    const EnumValueDescriptor *enumval = enumdesc->FindValueByName(v);
		    if (enumval) {
			    if (field->is_repeated()) {
				    if (add_repeated) {
					    refl->AddEnum(msg, field, enumval);
				    } else {
					    refl->SetRepeatedEnum(msg, field, partial_index, enumval);
				    }
			    } else {
				    refl->SetEnum(msg, field, enumval);
			    }
		    } else {
			    warn("ProtobufCommAdapter:pb_set_value:"
			         "Invalid enum value '" << v << "' for field " << field_name);
			    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			    m_execInterface.notifyOfExternalEvent();
			    return;
		    }
	    }
	    break;
    default:
	    break;
    }
  } catch (std::logic_error &e) {
	  warn("ProtobufCommAdapter:pb_set_value:"
	       "Failed to set field " << field_name << ": " << e.what());
	  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	  m_execInterface.notifyOfExternalEvent();
  }

  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}


void
ProtobufCommPlexilAdapter::pb_get_value(PLEXIL::Command* cmd, PLEXIL::ValueType value_type)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_get_value",
	                  {{"msg_id", PLEXIL::STRING_TYPE},
	                   {"field", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   msg_id;
	std::string   field_name;
	args[0].getValue(msg_id);
	args[1].getValue(field_name);

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:pb_get_value:"
		     << " Unknown message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	message_meta &msgmeta = messages_[msg_id];
	std::shared_ptr<google::protobuf::Message> m = msgmeta.message;

  const FieldDescriptor *field = nullptr;
  google::protobuf::Message *msg= m.get();
   
  std::string partial_name;
  long int    partial_index = -1;

  if (! traverse_field(msg, field_name, field, partial_name, partial_index, "pb_get_value")) {
	  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	  m_execInterface.notifyOfExternalEvent();
	  return;
  }
  
  //field = desc->FindFieldByName(field_path.back());

  if (! field) {
		warn("ProtobufCommAdapter:pb_get_value:"
		     << " Unknown field " << field_name << " for message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }
  const Reflection *refl       = msg->GetReflection();

  // check return value
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:
  case FieldDescriptor::TYPE_FLOAT:
	  if (value_type != PLEXIL::REAL_TYPE) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << ":"
		       << field_name << " not of expected type Real");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }
	  break;

  case FieldDescriptor::TYPE_INT64:
  case FieldDescriptor::TYPE_SFIXED64:
  case FieldDescriptor::TYPE_SINT64:
  case FieldDescriptor::TYPE_UINT64:
  case FieldDescriptor::TYPE_FIXED64:
  case FieldDescriptor::TYPE_INT32:
  case FieldDescriptor::TYPE_SFIXED32:
  case FieldDescriptor::TYPE_SINT32:
  case FieldDescriptor::TYPE_UINT32:
  case FieldDescriptor::TYPE_FIXED32:
	  if (value_type != PLEXIL::INTEGER_TYPE) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << ":"
		       << field_name << " not of expected type Integer");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }
	  break;


  case FieldDescriptor::TYPE_BOOL:
	  if (value_type != PLEXIL::BOOLEAN_TYPE) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << ":"
		       << field_name << " not of expected type Boolean");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }
	  break;

  case FieldDescriptor::TYPE_STRING:
  case FieldDescriptor::TYPE_ENUM:
	  if (value_type != PLEXIL::STRING_TYPE) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << ":"
		       << field_name << " not of expected type String");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }
	  break;

  default:
	  if (value_type != PLEXIL::STRING_TYPE) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << ":"
		       << field_name << " of unsupported protobuf type");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }
	  break;
  }

  if (field->is_repeated()) {
	  if (partial_index >= refl->FieldSize(*msg, field)) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName()
		       << "Index " << partial_index << " for " << partial_name
		       << " is out of bounds");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }

	  switch (field->type()) {
	  case FieldDescriptor::TYPE_DOUBLE:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetRepeatedDouble(*msg, field, partial_index)));
		  break;
    case FieldDescriptor::TYPE_FLOAT:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetRepeatedFloat(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_UINT64:
    case FieldDescriptor::TYPE_FIXED64:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetRepeatedUInt64(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_UINT32:
    case FieldDescriptor::TYPE_FIXED32:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetRepeatedUInt32(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_BOOL:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetRepeatedBool(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_STRING:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value(refl->GetRepeatedString(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_ENUM:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value(refl->GetRepeatedEnum(*msg, field, partial_index)->name()));
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_INT32:
    case FieldDescriptor::TYPE_SINT32:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetRepeatedInt32(*msg, field, partial_index)));
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
	    m_execInterface.handleCommandReturn(
	      cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetRepeatedInt64(*msg, field, partial_index)));
      break;
    default:
	    break;
    }

  } else {
	  if (! refl->HasField(*msg, field)) {
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << " not set");
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }

	  // Now get the actual value
	  switch (field->type()) {
	  case FieldDescriptor::TYPE_DOUBLE:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetDouble(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_FLOAT:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetFloat(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_INT64:
	  case FieldDescriptor::TYPE_SFIXED64:
	  case FieldDescriptor::TYPE_SINT64:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetInt64(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_UINT64:
	  case FieldDescriptor::TYPE_FIXED64:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetUInt64(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_INT32:
	  case FieldDescriptor::TYPE_SFIXED32:
	  case FieldDescriptor::TYPE_SINT32:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetInt32(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_UINT32:
	  case FieldDescriptor::TYPE_FIXED32:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value((PLEXIL::Integer)refl->GetUInt32(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_BOOL:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetBool(*msg, field)));
		  break;
	  case FieldDescriptor::TYPE_STRING:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetString(*msg, field)));
		  break;
	  
	  case FieldDescriptor::TYPE_ENUM:
		  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->GetEnum(*msg, field)->name()));
		  break;

	  default:
		  warn("ProtobufCommAdapter:pb_get_value:" << m->GetTypeName() << " invalid value type for " << field_name);
		  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		  m_execInterface.notifyOfExternalEvent();
		  return;
	  }

  }

  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
  m_execInterface.notifyOfExternalEvent();
}


void
ProtobufCommPlexilAdapter::pb_get_length(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_get_length",
	                  {{"msg_id", PLEXIL::STRING_TYPE},
	                   {"field", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   msg_id;
	std::string   field_name;
	args[0].getValue(msg_id);
	args[1].getValue(field_name);

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:pb_get_length:"
		     << " Unknown message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	message_meta &msgmeta = messages_[msg_id];
	std::shared_ptr<google::protobuf::Message> m = msgmeta.message;

  const FieldDescriptor *field = nullptr;
  google::protobuf::Message *msg= m.get();
   
  std::string partial_name;
  long int    partial_index = -1;

  if (! traverse_field(msg, field_name, field, partial_name, partial_index, "pb_get_length")) {
	  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
	  m_execInterface.notifyOfExternalEvent();
	  return;
  }

  if (! field) {
		warn("ProtobufCommAdapter:pb_get_length:"
		     << " Unknown field " << field_name << " for message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }

  if (! field->is_repeated()) {
		warn("ProtobufCommAdapter:pb_get_length:"
		     << " Field " << field_name << " is not a repeated field in " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }
  const Reflection *refl       = msg->GetReflection();

  m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(refl->FieldSize(*msg, field)));
  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
  m_execInterface.notifyOfExternalEvent();
}

void
ProtobufCommPlexilAdapter::pb_tostring(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_tostring",
	                  {{"msg_id", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string   msg_id;
	args[0].getValue(msg_id);

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:pb_tostring:"
		     << " Unknown message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	message_meta &msgmeta = messages_[msg_id];
	std::shared_ptr<google::protobuf::Message> m = msgmeta.message;

	m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(m->DebugString()));
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}


void
ProtobufCommPlexilAdapter::pb_broadcast(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_broadcast",
	                  {{"peer_id", PLEXIL::INTEGER_TYPE},
	                   {"msg_id", PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	int           peer_id;
	std::string   msg_id;
	args[0].getValue(peer_id);
	args[1].getValue(msg_id);

	if (peers_.find(peer_id) == peers_.end()) {
		warn("ProtobufCommAdapter:pb_broadcast:"
		     << " Unknown peer " << peer_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	if (messages_.find(msg_id) == messages_.end()) {
		warn("ProtobufCommAdapter:pb_broadcast:"
		     << " Unknown message " << msg_id);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	message_meta &msgmeta = messages_[msg_id];
	std::shared_ptr<google::protobuf::Message> m = msgmeta.message;

	std::lock_guard<std::mutex> lock(map_mutex_);

  //logger_->log_info("CLIPS-Protobuf", "Broadcasting %s", (*m)->GetTypeName().c_str());
  try {
    peers_[peer_id]->send(m);
  } catch (google::protobuf::FatalException &e) {
		warn("ProtobufCommAdapter:pb_broadcast:"
		     << " Failed to send message " << msg_id << "(" << e.what() << ")");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  } catch (fawkes::Exception &e) {
		warn("ProtobufCommAdapter:pb_broadcast:"
		     << " Failed to send message " << msg_id << "(" << e.what_no_backtrace() << ")");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  } catch (std::runtime_error &e) {
		warn("ProtobufCommAdapter:pb_broadcast:"
		     << " Failed to  message " << msg_id << "(" << e.what() << ")");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
  }

  messages_.erase(msg_id);

  m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
  m_execInterface.notifyOfExternalEvent();
}


/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
void
ProtobufCommPlexilAdapter::pb_peer_create_local_crypto(PLEXIL::Command* cmd,
                                                       const std::vector<PLEXIL::Value> *override_args)
{
	std::vector<PLEXIL::Value> const &args = override_args ? *override_args : cmd->getArgValues();

	if (! verify_args(args, "pb_peer_create_local_crypto",
	                  {{"address",    PLEXIL::STRING_TYPE},
	                   {"send_port",  PLEXIL::INTEGER_TYPE},
	                   {"recv_port",  PLEXIL::INTEGER_TYPE},
	                   {"crypto_key", PLEXIL::STRING_TYPE},
	                   {"cipher",     PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string address;
	int send_port, recv_port;
	std::string crypto_key, cipher;

	args[0].getValue(address);
	args[1].getValue(send_port);
	args[2].getValue(recv_port);
	args[3].getValue(crypto_key);
	args[4].getValue(cipher);
	
	if (recv_port <= 0)  recv_port = send_port;

	if (send_port > 0) {
		std::shared_ptr<ProtobufBroadcastPeer> peer =
		  std::make_shared<ProtobufBroadcastPeer>(address,
		                                          (unsigned short)send_port, (unsigned short)recv_port,
		                                          &*message_register_, crypto_key, cipher);

		int peer_id;
    {
	    std::lock_guard<std::mutex> lock(map_mutex_);
	    peer_id = ++next_client_id_;
	    peers_[peer_id] = peer;
    }

    peer->signal_received()
      .connect(boost::bind(&ProtobufCommPlexilAdapter::handle_peer_msg, this, peer_id, _1, _2, _3, _4));
    peer->signal_recv_error()
      .connect(boost::bind(&ProtobufCommPlexilAdapter::handle_peer_recv_error, this, peer_id, _1, _2));
    peer->signal_send_error()
      .connect(boost::bind(&ProtobufCommPlexilAdapter::handle_peer_send_error, this, peer_id, _1));

    m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(peer_id));
    m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
		m_execInterface.notifyOfExternalEvent();
  } else {
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
  }
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
void
ProtobufCommPlexilAdapter::pb_peer_create_crypto(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &in_args = cmd->getArgValues();
	if (! verify_args(in_args, "pb_peer_create_crypto",
	                  {{"address",    PLEXIL::STRING_TYPE},
	                   {"port",  PLEXIL::INTEGER_TYPE},
	                   {"crypto_key", PLEXIL::STRING_TYPE},
	                   {"cipher",     PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::vector<PLEXIL::Value> args{in_args[0], in_args[1], in_args[1], in_args[2], in_args[3]};

	pb_peer_create_local_crypto(cmd, &args);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @return peer identifier
 */
void
ProtobufCommPlexilAdapter::pb_peer_create(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &in_args = cmd->getArgValues();
	if (! verify_args(in_args, "pb_peer_create",
	                  {{"address",    PLEXIL::STRING_TYPE},
	                   {"port",  PLEXIL::INTEGER_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::vector<PLEXIL::Value> args{in_args[0], in_args[1], in_args[1],
	                                PLEXIL::Value(""), PLEXIL::Value("")};

	pb_peer_create_local_crypto(cmd, &args);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p send_port
 * @return peer identifier
 */
void
ProtobufCommPlexilAdapter::pb_peer_create_local(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &in_args = cmd->getArgValues();
	if (! verify_args(in_args, "pb_peer_create_local",
	                  {{"address",    PLEXIL::STRING_TYPE},
	                   {"send_port",  PLEXIL::INTEGER_TYPE},
	                   {"recv_port",  PLEXIL::INTEGER_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::vector<PLEXIL::Value> args{in_args[0], in_args[1], in_args[2],
	                                PLEXIL::Value(""), PLEXIL::Value("")};

	pb_peer_create_local_crypto(cmd, &args);
}


/** Disable peer.
 * @param peer_id ID of the peer to destroy
 */
void
ProtobufCommPlexilAdapter::pb_peer_destroy(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_peer_destroy",
	                  {{"peer_id",    PLEXIL::INTEGER_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	int peer_id;
	args[0].getValue(peer_id);

	if (peers_.find(peer_id) != peers_.end()) {
    peers_.erase(peer_id);
  }
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}


/** Setup crypto for peer. 
 * @param peer_id ID of the peer to destroy
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 */
void
ProtobufCommPlexilAdapter::pb_peer_setup_crypto(PLEXIL::Command* cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (! verify_args(args, "pb_peer_setup_crypto",
	                  {{"peer_id",    PLEXIL::INTEGER_TYPE},
	                   {"crypto_key",  PLEXIL::STRING_TYPE},
	                   {"cipher",  PLEXIL::STRING_TYPE}}))
	{
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	int peer_id;
	std::string crypto_key;
	std::string cipher;

	args[0].getValue(peer_id);
	args[1].getValue(crypto_key);
	args[2].getValue(cipher);

	if (peers_.find(peer_id) != peers_.end()) {
		peers_[peer_id]->setup_crypto(crypto_key, cipher);
	}

	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}


/** Handle message that came from a peer/robot
 * @param endpoint the endpoint from which the message was received
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void
ProtobufCommPlexilAdapter::handle_peer_msg(int peer_id,
                                           boost::asio::ip::udp::endpoint &endpoint,
                                           uint16_t component_id, uint16_t msg_type,
                                           std::shared_ptr<google::protobuf::Message> msg)
{
	message_meta m{
	   .time_received = fawkes::Time(clock_),
	   .from_host     = endpoint.address().to_string(),
	   .from_port     = endpoint.port(),
	   .message       = msg
	  };

	add_message(msg->GetTypeName(), std::move(m));
	proc_queue(msg->GetTypeName());
}


/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void
ProtobufCommPlexilAdapter::handle_peer_recv_error(int peer_id,
                                                  boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
  if (logger_) {
	  logger_->log_warn("PlexilProtobuf",
	                    "Failed to receive peer message from %s:%u: %s",
	                    endpoint.address().to_string().c_str(), endpoint.port(),
	                    msg.c_str());
  }
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void
ProtobufCommPlexilAdapter::handle_peer_send_error(int peer_id, std::string msg)
{
	if (logger_) {
		logger_->log_warn("PlexilProtobuf",
		                  "Failed to send peer message: %s", msg.c_str());
  }
}


extern "C" {
	void initProtobufCommAdapter() {
		REGISTER_ADAPTER(ProtobufCommPlexilAdapter, "ProtobufCommAdapter");
	}
}
