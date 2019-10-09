
/***************************************************************************
 * Protoboard plugin template
 * - Header for the blackboard manager
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

#ifndef BLACKBOARD_MANAGER_H
#define BLACKBOARD_MANAGER_H

#include "protoboard_types.h"
#include "protobuf_thread.h"

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/utils/on_message_waker.h>
#include <core/threading/thread.h>
#include <interfaces/ProtobufPeerInterface.h>

#include <boost/fusion/include/any.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/std_tuple.hpp>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace protoboard {

template <class IfaceT>
std::string iface_id_for_type();

std::vector<std::string> proto_dirs();
pb_conversion_map        make_receiving_interfaces_map();

template <class IfaceT, class MessageTypeList>
class bb_iface_manager
{
public:
	bb_iface_manager() : interface_(nullptr), blackboard_(nullptr), waker_(nullptr)
	{
	}

	void
	init(fawkes::BlackBoard *blackboard, fawkes::Thread *thread)
	{
		blackboard_ = blackboard;
		interface_  = blackboard_->open_for_writing<IfaceT>(iface_id_for_type<IfaceT>().c_str());
		waker_      = new fawkes::BlackBoardOnMessageWaker(blackboard, interface_, thread);
	}

	void
	finalize()
	{
		if (waker_)
			delete waker_;
		if (blackboard_ && interface_)
			blackboard_->close(interface_);
	}

	IfaceT *
	interface() const
	{
		return interface_;
	}

private:
	IfaceT *                          interface_;
	fawkes::BlackBoard *              blackboard_;
	fawkes::BlackBoardOnMessageWaker *waker_;
};

class AbstractProtobufSender
{
public:
	AbstractProtobufSender(BlackboardManager *bb_mgr);
	virtual ~AbstractProtobufSender();
	virtual bool process_sending_interfaces() = 0;

	virtual void init()     = 0;
	virtual void finalize() = 0;

protected:
	BlackboardManager *bb_manager;

	struct handle_messages
	{
		BlackboardManager *manager;

		template <class IfaceT, class MessageT>
		bool operator()(const bb_iface_manager<IfaceT, type_list<MessageT>> &pair) const;

		template <class IfaceT, class MessageT1, class... MessageTs>
		bool
		operator()(const bb_iface_manager<IfaceT, type_list<MessageT1, MessageTs...>> &iface_mgr) const;
	};
};

template <class... IfaceManagerTs>
class ProtobufSender : public AbstractProtobufSender
{
public:
	ProtobufSender(BlackboardManager *bb_mgr);

	virtual void init() override;
	virtual void finalize() override;

	virtual bool
	process_sending_interfaces() override
	{
		return boost::fusion::any(bb_sending_interfaces_, handle_messages{this->bb_manager});
	}

private:
	std::tuple<IfaceManagerTs...> bb_sending_interfaces_;
};

class BlackboardManager : public fawkes::Thread,
                          public fawkes::LoggingAspect,
                          public fawkes::ConfigurableAspect,
                          public fawkes::BlackBoardAspect,
                          public fawkes::ClockAspect
{
public:
	BlackboardManager(ProtobufThead *msg_handler);

	fawkes::BlackBoard *get_blackboard();
	void                set_protobuf_sender(AbstractProtobufSender *sender);

	friend AbstractProtobufSender;

protected:
	virtual void init() override;
	virtual void finalize() override;
	virtual void loop() override;

	template <class InterfaceT, class MessageT>
	void handle_message(InterfaceT *iface, MessageT *msg);

private:
	ProtobufThead *                         message_handler_;
	fawkes::ProtobufPeerInterface *         peer_iface_;
	pb_conversion_map                       bb_receiving_interfaces_;
	fawkes::BlackBoardOnMessageWaker *      on_message_waker_;
	unsigned int                            next_peer_idx_;
	std::unique_ptr<AbstractProtobufSender> pb_sender_;

	void add_peer(fawkes::ProtobufPeerInterface *iface, long peer_id);

	template <class MessageT, class InterfaceT>
	bool handle_message_type(InterfaceT *iface);

	template <class InterfaceT>
	struct on_interface
	{
		InterfaceT *       iface;
		BlackboardManager *manager;

		on_interface(InterfaceT *iface, BlackboardManager *manager) : iface(iface), manager(manager)
		{
		}

		template <class MessageT>
		bool
		handle_msg_types()
		{
			return manager->handle_message_type<MessageT>(iface);
		}

		// This template is disabled if MessageTs is {} to resolve ambiguity
		template <class MessageT1, class... MessageTs>
		typename std::enable_if<(sizeof...(MessageTs) > 0), bool>::type
		handle_msg_types()
		{
			return handle_msg_types<MessageTs...>() || handle_msg_types<MessageT1>();
		}
	};
};

template <class... IfaceManagerTs>
ProtobufSender<IfaceManagerTs...>::ProtobufSender(BlackboardManager *bb_mgr)
: AbstractProtobufSender(bb_mgr)
{
}

template <class... IfaceManagerTs>
void
ProtobufSender<IfaceManagerTs...>::init()
{
	boost::fusion::for_each(bb_sending_interfaces_, [this](auto &iface_mgr) {
		iface_mgr.init(this->bb_manager->get_blackboard(), this->bb_manager);
	});
}

template <class... IfaceManagerTs>
void
ProtobufSender<IfaceManagerTs...>::finalize()
{
	boost::fusion::for_each(bb_sending_interfaces_,
	                        [this](auto &iface_mgr) { iface_mgr.finalize(); });
}

template <class IfaceT, class MessageT>
bool
AbstractProtobufSender::handle_messages::
operator()(const bb_iface_manager<IfaceT, type_list<MessageT>> &pair) const
{
	return manager->handle_message_type<MessageT>(pair.interface());
}

template <class IfaceT, class MessageT1, class... MessageTs>
bool
AbstractProtobufSender::handle_messages::
operator()(const bb_iface_manager<IfaceT, type_list<MessageT1, MessageTs...>> &iface_mgr) const
{
	return BlackboardManager::on_interface<IfaceT>{iface_mgr.interface(), manager}
	         .template handle_msg_types<MessageTs...>()
	       || manager->handle_message_type<MessageT1>(iface_mgr.interface());
}

template <class MessageT, class InterfaceT>
bool
BlackboardManager::handle_message_type(InterfaceT *iface)
{
	if (!iface->msgq_empty()) {
		bool rv = false;
		while (MessageT *msg = iface->msgq_first_safe(msg)) {
			try {
				handle_message(iface, msg);
			} catch (std::exception &e) {
				logger->log_error(
				  name(), "Exception handling %s on %s: %s", msg->type(), iface->uid(), e.what());
			}
			iface->msgq_pop();
			rv = true;
		}
		iface->write();
		return rv;
	} else
		return false;
}

} // namespace protoboard

#endif // BLACKBOARD_MANAGER_H
