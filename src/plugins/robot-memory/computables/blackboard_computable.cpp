/***************************************************************************
 *  blackboard_computable.cpp - Computable providing blackboard access
 *    
 *
 *  Created: 1:22:31 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "blackboard_computable.h"

/** @class BlackboardComputable  blackboard_computable.h
 * Computable providing access to blackboard interfaces.
 * The Query has to match {interface:{$exists:true}} on the blackboard collection
 * @author Frederik Zwilling
 */

using namespace fawkes;
using namespace bsoncxx;
using namespace mongocxx;

/**
 * Constructor with references to objects of the plugin
 * @param robot_memory Robot Memory
 * @param blackboard Blackboard
 * @param logger Logger
 * @param config Configuration
 */
BlackboardComputable::BlackboardComputable(RobotMemory *          robot_memory,
                                           fawkes::BlackBoard *   blackboard,
                                           fawkes::Logger *       logger,
                                           fawkes::Configuration *config)
{
	robot_memory_ = robot_memory;
	blackboard_   = blackboard;
	logger_       = logger;

	//register computable
	document::value query = from_json("{interface:{$exists:true}}");
	int   priority        = config->get_int("plugins/robot-memory/computables/blackboard/priority");
	float caching_time =
	  config->get_float("plugins/robot-memory/computables/blackboard/caching-time");
	computable = robot_memory_->register_computable(std::move(query),
	                                                "robmem.blackboard",
	                                                &BlackboardComputable::compute_interfaces,
	                                                this,
	                                                caching_time,
	                                                priority);
}

BlackboardComputable::~BlackboardComputable()
{
	robot_memory_->remove_computable(computable);
}

std::list<document::value>
BlackboardComputable::compute_interfaces(const document::view &query, const std::string &collection)
{
	std::list<document::value> res;
	std::string                type  = query["interface"].get_utf8().value.to_string();
	std::string                id    = "*";
	auto                       id_it = query.find("id");
	if (id_it != query.end()) {
		id = query["id"].get_utf8().value.to_string();
	}
	//get all matching interfaces
	for (Interface *interface : blackboard_->open_multiple_for_reading(type.c_str(), id.c_str())) {
		interface->read();
		//build document
		using namespace bsoncxx::builder;
		basic::document doc;
		doc.append(basic::kvp("interface", interface->type()));
		doc.append(basic::kvp("id", interface->id()));
		for (InterfaceFieldIterator it = interface->fields(); it != interface->fields_end(); ++it) {
			if (it.get_length() > 1 && it.get_type() != IFT_STRING) {
				doc.append(basic::kvp(std::string(it.get_name()), [it](basic::sub_array array) {
					for (unsigned int i = 0; i < it.get_length(); i++) {
						switch (it.get_type()) {
						case IFT_BOOL: array.append(it.get_bool(i)); break;
						case IFT_INT8: array.append(it.get_int8(i)); break;
						case IFT_UINT8: array.append(it.get_uint8(i)); break;
						case IFT_INT16: array.append(it.get_int16(i)); break;
						case IFT_UINT16: array.append(it.get_uint16(i)); break;
						case IFT_INT32: array.append(it.get_int32(i)); break;
						case IFT_UINT32: array.append(static_cast<int64_t>(it.get_uint32(i))); break;
						case IFT_INT64: array.append(static_cast<int64_t>(it.get_int64(i))); break;
						case IFT_UINT64: array.append(static_cast<int64_t>(it.get_uint64(i))); break;
						case IFT_FLOAT: array.append(it.get_float(i)); break;
						case IFT_DOUBLE: array.append(it.get_double(i)); break;
						case IFT_STRING: array.append(it.get_string()); break;
						case IFT_BYTE: array.append(it.get_byte(i)); break;
						case IFT_ENUM: array.append(it.get_enum_string(i)); break;
						}
					}
				}));
			} else {
				std::string key{it.get_name()};
				switch (it.get_type()) {
				case IFT_BOOL: doc.append(basic::kvp(key, it.get_bool())); break;
				case IFT_INT8: doc.append(basic::kvp(key, it.get_int8())); break;
				case IFT_UINT8: doc.append(basic::kvp(key, it.get_uint8())); break;
				case IFT_INT16: doc.append(basic::kvp(key, it.get_int16())); break;
				case IFT_UINT16: doc.append(basic::kvp(key, it.get_uint16())); break;
				case IFT_INT32: doc.append(basic::kvp(key, it.get_int32())); break;
				case IFT_UINT32: doc.append(basic::kvp(key, static_cast<int64_t>(it.get_uint32()))); break;
				case IFT_INT64: doc.append(basic::kvp(key, static_cast<int64_t>(it.get_int64()))); break;
				case IFT_UINT64: doc.append(basic::kvp(key, static_cast<int64_t>(it.get_uint64()))); break;
				case IFT_FLOAT: doc.append(basic::kvp(key, it.get_float())); break;
				case IFT_DOUBLE: doc.append(basic::kvp(key, it.get_double())); break;
				case IFT_STRING: doc.append(basic::kvp(key, it.get_string())); break;
				case IFT_BYTE: doc.append(basic::kvp(key, it.get_byte())); break;
				case IFT_ENUM: doc.append(basic::kvp(key, it.get_enum_string())); break;
				}
			}
		}
		res.push_back(doc.extract());
		blackboard_->close(interface);
	}
	return res;
}
