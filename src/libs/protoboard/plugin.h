
/***************************************************************************
 * Protoboard plugin template
 * - Main plugin template: Instantiate this with the appropriate interface
 *   handler mappings to create a domain-specific plugin.
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

#ifndef PROTOBOARD_PLUGIN_H
#define PROTOBOARD_PLUGIN_H

#include "blackboard_manager.h"
#include "protobuf_thread.h"

#include <core/plugin.h>

template <class... IfaceManagerTs>
class ProtoboardPlugin : public fawkes::Plugin
{
public:
	ProtoboardPlugin(fawkes::Configuration *cfg) : Plugin(cfg)
	{
		protoboard::ProtobufThead *    protobuf_thread = new protoboard::ProtobufThead();
		protoboard::BlackboardManager *bb_mgr = new protoboard::BlackboardManager(protobuf_thread);
		bb_mgr->set_protobuf_sender(new protoboard::ProtobufSender<IfaceManagerTs...>(bb_mgr));
		protobuf_thread->set_bb_manager(bb_mgr);
		thread_list.push_back(bb_mgr);
		thread_list.push_back(protobuf_thread);
	}
};

#endif // PROTOBOARD_PLUGIN_H
