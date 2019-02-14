
/***************************************************************************
 *  protobuf_adapter.h - PLEXIL adapter for the protobuf_comm
 *
 *  Created: Thu Aug 16 11:05:36 2018
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

#ifndef __PLUGINS_PLEXIL_PROTOBUF_ADAPTER_H_
#define __PLUGINS_PLEXIL_PROTOBUF_ADAPTER_H_

#include <logging/logger.h>
#include <config/config.h>
#include <utils/time/time.h>
#include <utils/time/clock.h>
#include <protobuf_comm/peer.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>

#include <memory>
#include <mutex>

/** Interface adapter to provide logging facilities. */
class ProtobufCommPlexilAdapter
	: public PLEXIL::InterfaceAdapter
{
public:
	ProtobufCommPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface);
	ProtobufCommPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
	                   pugi::xml_node const xml);

	/// @cond DELETED
	ProtobufCommPlexilAdapter() = delete;
	ProtobufCommPlexilAdapter(const ProtobufCommPlexilAdapter &) = delete;
	ProtobufCommPlexilAdapter & operator=(const ProtobufCommPlexilAdapter &) = delete;
	/// @endcond

	virtual ~ProtobufCommPlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
  void invokeAbort(PLEXIL::Command *cmd);

private:
	/// @cond INTERNAL
	struct queue_entry {


		std::vector<std::string>      messages;
		std::vector<PLEXIL::Command*> recipients;
	};

	struct message_meta {
		fawkes::Time                                time_received;
		std::string                                 from_host;
		int                                         from_port;
		std::shared_ptr<google::protobuf::Message>  message;
	};
	/// @endcond


	void proc_receive_command(PLEXIL::Command *cmd);
	void proc_get_param_command(PLEXIL::Command *cmd);
	void proc_send_rv_command(PLEXIL::Command *cmd);

	void add_recipient(const std::string& msg_type, PLEXIL::Command* cmd);
	void remove_recipient(const std::string& msg_type, PLEXIL::Command* cmd);

	std::string gen_msgid(const std::string& msg_type);
	void add_message(const std::string& msg_type, message_meta&& msg);
	void release_message(const std::string& msg_id);
	std::shared_ptr<google::protobuf::Message> get_message(const std::string& msg_id);

	void proc_queue(const std::string& msg_type);
	queue_entry& get_queue(const std::string& msg_type);

	void pb_create(PLEXIL::Command* cmd);
	void pb_destroy(PLEXIL::Command* cmd);
  void pb_set_value(PLEXIL::Command* cmd);
	void pb_get_value(PLEXIL::Command* cmd, PLEXIL::ValueType value_type);
  void pb_get_length(PLEXIL::Command* cmd);
  void pb_has_field(PLEXIL::Command* cmd);
  void pb_tostring(PLEXIL::Command* cmd);
  void pb_broadcast(PLEXIL::Command* cmd);

	void pb_peer_create(PLEXIL::Command* cmd);
	void pb_peer_create_local(PLEXIL::Command* cmd);
	void pb_peer_create_crypto(PLEXIL::Command* cmd);
	void pb_peer_create_local_crypto(PLEXIL::Command* cmd, const std::vector<PLEXIL::Value> *args = nullptr);
	void pb_peer_destroy(PLEXIL::Command* cmd);
	void pb_peer_setup_crypto(PLEXIL::Command* cmd);

	
	void handle_peer_msg(int peer_id,
	                     boost::asio::ip::udp::endpoint &endpoint,
	                     uint16_t component_id, uint16_t msg_type,
	                     std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(int peer_id, boost::asio::ip::udp::endpoint &endpoint, std::string msg);
  void handle_peer_send_error(int peer_id, std::string msg);

private:
	fawkes::Configuration *     config_;
	fawkes::Logger *            logger_;
	fawkes::Clock *             clock_;
	
	std::mutex                          queue_mutex_;
	std::map<std::string, queue_entry>  queue_;
	std::map<std::string, message_meta> messages_;
	unsigned long int                   msg_counter_;

	typedef std::map<int, std::shared_ptr<protobuf_comm::ProtobufBroadcastPeer>> PeerMap;
	
  int                                              next_client_id_;
	std::mutex                                       map_mutex_;
	std::shared_ptr<protobuf_comm::MessageRegister>  message_register_;
	PeerMap                                          peers_;

	std::map<std::string, std::function<void (PLEXIL::Command*)>> commands_;
};

extern "C" {
  void initProtobufCommAdapter();
}

#endif
