
/***************************************************************************
 * Protoboard plugin template
 * - Header for the ProtoBuf thread
 *
 * Copyright 2019 Victor Mataré
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

#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <protobuf_comm/server.h>

#include <list>
#include <map>
#include <queue>

#define CFG_PREFIX "/plugins/protoboard"

namespace protobuf_comm {
class ProtobufStreamClient;
class ProtobufBroadcastPeer;
} // namespace protobuf_comm

namespace protoboard {

class BlackboardManager;

/**
 * Receive incoming ProtoBuf messages and pass them on to the @a BlackboardManager
 * for publication to the appropriate blackboard interface.
 */
class ProtobufThead : public fawkes::Thread,
                      public fawkes::LoggingAspect,
                      public fawkes::ConfigurableAspect,
                      public fawkes::BlackBoardAspect
{
public:
	/// Empty-initialization constructor
	ProtobufThead();
	/// Destructor
	virtual ~ProtobufThead() override;

	/// @return whether incoming ProtoBuf messages are in the queue
	bool pb_queue_incoming();

	/// Wrapper for a ProtoBuf message and its metadata
	struct incoming_message
	{
		/// The ProtoBuf peer ID that received this message
		long int peer_id;
		/// The boost::asio UDP endpoint used by the receiving peer
		boost::asio::ip::udp::endpoint endpoint;
		/// The ProtoBuf component ID
		uint16_t component_id;
		/// The ProtoBuf type ID
		uint16_t msg_type;
		/// The message itself
		std::shared_ptr<google::protobuf::Message> msg;
	};

	/// @return The head of the incoming ProtoBuf message queue (popped)
	incoming_message pb_queue_pop();

	long int peer_create(const std::string &host, int port);
	long int peer_create_local(const std::string &host, int send_to_port, int recv_on_port);
	long int peer_create_crypto(const std::string &host,
	                            int                port,
	                            const std::string &crypto_key = "",
	                            const std::string &cipher     = "");
	long int peer_create_local_crypto(const std::string &host,
	                                  int                send_to_port,
	                                  int                recv_on_port,
	                                  const std::string &crypto_key = "",
	                                  const std::string &cipher     = "");
	void     peer_destroy(long int peer_id);

	/**
	 * Send a ProtoBuf message to the given peer
	 * @param peer_id The peer to send to
	 * @param msg The message
	 */
	void send(long int peer_id, std::shared_ptr<google::protobuf::Message> msg);

	/**
	 * Deferred initialization of the pointer to the BlackboardManager
	 * @param bb_manager the BlackboardManager to use
	 */
	void
	set_bb_manager(BlackboardManager *bb_manager)
	{
		bb_manager_ = bb_manager;
	}

	/**
	 * Helper to give ProtoBuf converters access to the BlackBoard instance in use
	 * @return A ready-to-use pointer to the BlackBoard
	 */
	fawkes::BlackBoard *
	get_blackboard()
	{
		return blackboard;
	}

protected:
	virtual void init() override;

private:
	/** Get protobuf_comm peers.
   * @return protobuf_comm peer */
	const std::map<long int, protobuf_comm::ProtobufBroadcastPeer *> &
	peers() const
	{
		return peers_;
	}

	/** Signal invoked for a message that has been sent via broadcast.
   * @return signal
   */
	boost::signals2::signal<void(long, std::shared_ptr<google::protobuf::Message>)> &
	signal_peer_sent()
	{
		return sig_peer_sent_;
	}

	void
	peer_setup_crypto(long int peer_id, const std::string &crypto_key, const std::string &cipher);

	void handle_peer_msg(long int                                   peer_id,
	                     boost::asio::ip::udp::endpoint &           endpoint,
	                     uint16_t                                   component_id,
	                     uint16_t                                   msg_type,
	                     std::shared_ptr<google::protobuf::Message> msg);
	void handle_peer_recv_error(long int                        peer_id,
	                            boost::asio::ip::udp::endpoint &endpoint,
	                            std::string                     msg);
	void handle_peer_send_error(long int peer_id, std::string msg);

	protobuf_comm::MessageRegister *message_register_;

	boost::signals2::signal<void(long int, std::shared_ptr<google::protobuf::Message>)>
	  sig_peer_sent_;

	fawkes::Mutex map_mutex_;
	fawkes::Mutex msgq_mutex_;
	long int      next_client_id_;

	std::map<long int, protobuf_comm::ProtobufBroadcastPeer *> peers_;

	BlackboardManager *          bb_manager_;
	std::queue<incoming_message> pb_queue_;
};

} // namespace protoboard

#endif // MESSAGE_HANDLER_H
