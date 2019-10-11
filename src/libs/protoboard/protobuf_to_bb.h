
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

class pb_convert : public std::enable_shared_from_this<pb_convert>
{
public:
	pb_convert();

	pb_convert(const pb_convert &) = default;
	pb_convert &operator=(const pb_convert &) = default;

	virtual ~pb_convert();

	virtual void init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, size_t = 0);

	virtual void handle(std::shared_ptr<google::protobuf::Message> msg);

	virtual void handle(const google::protobuf::Message &);

protected:
	fawkes::BlackBoard *blackboard_;
	fawkes::Logger *    logger_;
};

template <class ProtoT, class IfaceT>
class pb_converter : public pb_convert
{
public:
	typedef ProtoT input_type;
	typedef IfaceT output_type;

	pb_converter() : pb_convert(), interface_(nullptr)
	{
	}

	// Don't copy this
	pb_converter(const pb_converter<ProtoT, IfaceT> &) = delete;
	pb_converter<ProtoT, IfaceT> &operator=(const pb_converter<ProtoT, IfaceT> &) = delete;

	// Only move!
	pb_converter(pb_converter<ProtoT, IfaceT> &&o)
	: pb_convert(o), interface_(std::move(o.interface_))
	{
		o.interface_ = nullptr;
	}

	pb_converter<ProtoT, IfaceT> &
	operator=(pb_converter<ProtoT, IfaceT> &&o)
	{
		pb_convert::operator=(o);
		this->interface_    = o.interface_;
		o.interface_        = nullptr;
		return *this;
	}

	virtual ~pb_converter()
	{
		close();
	}

	virtual void
	init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, size_t id = 0) override
	{
		pb_convert::init(blackboard, logger);
		std::string iface_id = iface_id_for_type<IfaceT>();

		if (iface_id.back() != '/')
			iface_id += '/';
		iface_id += std::to_string(id);

		interface_ = blackboard_->open_for_writing<IfaceT>(iface_id.c_str());
		logger->log_info(boost::core::demangle(typeid(*this).name()).c_str(),
		                 "Initialized %s.",
		                 iface_id.c_str());
	}

	virtual void
	handle(const google::protobuf::Message &msg) override
	{
		handle(dynamic_cast<const ProtoT &>(msg));
	}

	virtual void
	handle(const ProtoT &msg)
	{
		handle(msg, interface_);
		interface_->write();
	}

	virtual bool
	is_open()
	{
		return interface_;
	}

	virtual void
	close()
	{
		if (is_open()) {
			blackboard_->close(interface_);
			interface_ = nullptr;
		}
	}

	IfaceT *
	interface()
	{
		return interface_;
	}

	static size_t
	get_sequence_index(const ProtoT &)
	{
		return 0;
	}

protected:
	virtual void handle(const ProtoT &msg, IfaceT *iface);

private:
	IfaceT *interface_;
};

template <class ProtoT, class OutputT>
class pb_sequence_converter : public pb_convert
{
public:
	typedef google::protobuf::RepeatedPtrField<typename OutputT::input_type> sequence_type;

	pb_sequence_converter()
	{
	}

	virtual void
	handle(const google::protobuf::Message &msg) override
	{
		sequence_type fields = extract_sequence(dynamic_cast<const ProtoT &>(msg));

		if (fields.empty())
			return;

		typename sequence_type::const_iterator field_it = fields.begin();

		for (; field_it != fields.end(); ++field_it) {
			size_t seq_idx = OutputT::get_sequence_index(*field_it);
			auto   map_it  = sub_converters_.find(seq_idx);
			if (map_it == sub_converters_.end()) {
				sub_converters_.insert({seq_idx, OutputT()});
				map_it = sub_converters_.find(seq_idx);
			}

			if (!map_it->second.is_open())
				map_it->second.init(blackboard_, logger_, seq_idx);
			map_it->second.handle(*field_it);
		}
	}

	virtual const sequence_type &extract_sequence(const ProtoT &msg);

private:
	std::unordered_map<size_t, OutputT> sub_converters_;
};

} // namespace protoboard

#endif //PROTOBUF_TO_BB_H_
