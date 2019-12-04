
/***************************************************************************
 * Protoboard plugin template
 * - Templates that implement translation of incoming ProtoBuf messages
 *   to BlackBoard interfaces according to the appropriate template
 *   specializations
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

#ifndef PROTOBUF_TO_BB_H_
#define PROTOBUF_TO_BB_H_

#include "protoboard_types.h"

#include <blackboard/blackboard.h>
#include <google/protobuf/message.h>
#include <logging/logger.h>

#include <boost/bimap.hpp>
#include <boost/core/demangle.hpp>
#include <memory>

namespace protoboard {

template <class IfaceT>
std::string iface_id_for_type();

/**
 * Must be implemented by the user.
 * @return A map of ProtoBuf type names to their appropriate @a pb_converter instances
 */
pb_conversion_map make_receiving_interfaces_map();

/**
 * Default ProtoBuf to blackboard converter. This class just defines the necessary operations
 * but does nothing in itself. Thus it can be used to silently ignore certain incoming ProtoBuf
 * message types.
 */
class pb_convert : public std::enable_shared_from_this<pb_convert>
{
public:
	/// Empty-init constructor
	pb_convert();
	/// Default copy constructor
	pb_convert(const pb_convert &) = default;
	/// Destructor. Does nothing since members aren't owned by this class.
	virtual ~pb_convert();

	/** Default copy assignment
	 * @return The left-hand side */
	pb_convert &operator=(const pb_convert &) = default;

	/** Deferred initialization
	 * @param blackboard A pointer to a ready-to-use blackboard
	 * @param logger A pointer to a ready-to-use logger */
	virtual void
	init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, const std::string & = "");

	/** Dereference @a msg and pass it on to handle it by reference
	 * @param msg shared_ptr to a ProtoBuf message */
	virtual void handle(std::shared_ptr<google::protobuf::Message> msg);

	/** Handle a ProtoBuf message by reference. Overridden in @ref pb_converter
	 * @param msg Reference to a generic ProtoBuf message */
	virtual void handle(const google::protobuf::Message &msg);

protected:
	/// Blackboard used by the main thread
	fawkes::BlackBoard *blackboard_;
	/// Logger from the main thread
	fawkes::Logger *logger_;
};

/**
 * The workhorse of the ProtoBuf to Blackboard conversion
 * @tparam A concrete ProtoBuf message type
 * @tparam The BlackBoard interface type that the ProtoBuf type should be mapped to
 */
template <class ProtoT, class IfaceT>
class pb_converter : public pb_convert
{
public:
	/// The ProtoBuf message type that goes in
	typedef ProtoT input_type;
	/// The blackboard interface type that the ProtoBuf contents are written to
	typedef IfaceT output_type;

	/// Empty-init
	pb_converter()
	: pb_convert(), interface_(nullptr), name_(boost::core::demangle(typeid(*this).name()))
	{
	}

	/** Copying this is prohibited
	 * @param "" deleted */
	pb_converter(const pb_converter<ProtoT, IfaceT> &) = delete;

	/** Copying this is prohibited
	 * @param "" deleted
	 * @return deleted */
	pb_converter<ProtoT, IfaceT> &operator=(const pb_converter<ProtoT, IfaceT> &) = delete;

	/** Move construction
	 * @param o Another pb_converter to move from */
	pb_converter(pb_converter<ProtoT, IfaceT> &&o)
	: pb_convert(o),
	  interface_(std::move(o.interface_)),
	  name_(boost::core::demangle(typeid(*this).name()))
	{
		o.interface_ = nullptr;
	}

	/** Move assignment
	 * @param o Another pb_converter to move from
	 * @return A reference to this pb_converter */
	pb_converter<ProtoT, IfaceT> &
	operator=(pb_converter<ProtoT, IfaceT> &&o)
	{
		pb_convert::operator=(o);
		this->interface_    = o.interface_;
		o.interface_        = nullptr;
		name_               = boost::core::demangle(typeid(*this).name());
		return *this;
	}

	/// Close blackboard interface on destruction
	virtual ~pb_converter()
	{
		close();
	}

	/** Deferred initialization, coincides with main thread initialization
	 * @param blackboard Initialized blackboard
	 * @param logger Logger used by the main thread
	 * @param id Blackboard interface ID to open */
	virtual void
	init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, const std::string &id = "") override
	{
		pb_convert::init(blackboard, logger);
		std::string iface_id = iface_id_for_type<IfaceT>();

		if (id.length()) {
			if (iface_id.back() != '/')
				iface_id += '/';
			iface_id += id;
		}

		interface_ = blackboard_->open_for_writing<IfaceT>(iface_id.c_str());
		logger->log_info(name(), "Initialized %s.", iface_id.c_str());
	}

	virtual void
	handle(const google::protobuf::Message &msg) override
	{
		handle(dynamic_cast<const ProtoT &>(msg));
	}

	/** Handle a ProtoBuf message with known type. Just delegates to a user-definable method
	 * where the ProtoBuf message is matched up with the appropriate blackboard interface.
	 * @param msg The incoming ProtoBuf message */
	virtual void
	handle(const ProtoT &msg)
	{
		handle(msg, interface_);
		interface_->write();
	}

	/// @return whether we have a Blackboard interface
	virtual bool
	is_open()
	{
		return interface_;
	}

	/// Give up the current blackboard interface (closes it)
	virtual void
	close()
	{
		if (is_open()) {
			blackboard_->close(interface_);
			interface_ = nullptr;
		}
	}

	/// @return the current blackboard interface
	IfaceT *
	interface()
	{
		return interface_;
	}

	/** @return The blackboard ID suffix if this is part of a sequence. Defaults to "".
	 * Must be overriden for ProtoBuf message types that are part of a sequence and should be put
	 * in separate interfaces. */
	static std::string
	get_sequence_id(const ProtoT &)
	{
		return "";
	}

	/// @return The demangled class name for logging
	const char *
	name()
	{
		return name_.c_str();
	}

protected:
	/** Write the contents of a ProtoBuf message into the appropriate blackboard interface.
	 * Must be specialized by the user for each ProtoBuf message -> blackboard interface pair
	 * @param msg The message received
	 * @param iface The appropriate interface */
	virtual void handle(const ProtoT &msg, IfaceT *iface);

private:
	IfaceT *    interface_;
	std::string name_;
};

/**
 * A special handler for repeated ProtoBuf fields.
 * @tparam ProtoT the ProtoBuf message type that contains a repeated field we want to unwrap
 * @tparam The @a pb_converter type that should be used (repeatedly) on the repeated field
 */
template <class ProtoT, class OutputT>
class pb_sequence_converter : public pb_convert
{
private:
	typedef google::protobuf::RepeatedPtrField<typename OutputT::input_type> sequence_type;

public:
	/// Default constructor
	pb_sequence_converter()
	{
	}

	/** Handle a repeated field inside a ProtoBuf message, where the individual repeated
	 * sub-messages should be mapped to a blackboard interface each.
	 * @param msg The message containing the repeated field that should be extracted */
	virtual void
	handle(const google::protobuf::Message &msg) override
	{
		sequence_type fields = extract_sequence(dynamic_cast<const ProtoT &>(msg));

		if (fields.empty())
			return;

		typename sequence_type::const_iterator field_it = fields.begin();

		for (; field_it != fields.end(); ++field_it) {
			std::string seq_id = OutputT::get_sequence_id(*field_it);
			auto        map_it = sub_converters_.find(seq_id);
			if (map_it == sub_converters_.end()) {
				sub_converters_.insert({seq_id, OutputT()});
				map_it = sub_converters_.find(seq_id);
			}

			if (!map_it->second.is_open())
				map_it->second.init(blackboard_, logger_, seq_id);
			map_it->second.handle(*field_it);
		}
	}

	/** Must be implemented by the user.
	 * @param msg The message containing the repeated field
	 * @return The repeated field */
	virtual const sequence_type &extract_sequence(const ProtoT &msg);

private:
	std::unordered_map<std::string, OutputT> sub_converters_;
};

} // namespace protoboard

#endif //PROTOBUF_TO_BB_H_
