
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

#include "protobuf_to_bb.h"

namespace protoboard {

pb_convert::pb_convert() : blackboard_(nullptr), logger_(nullptr)
{
}

pb_convert::~pb_convert()
{
}

void
pb_convert::init(fawkes::BlackBoard *blackboard, fawkes::Logger *logger, const std::string &)
{
	blackboard_ = blackboard;
	logger_     = logger;
}

void
pb_convert::handle(std::shared_ptr<google::protobuf::Message> msg)
{
	handle(*msg);
}

void
pb_convert::handle(const google::protobuf::Message &)
{
}

} // namespace protoboard
