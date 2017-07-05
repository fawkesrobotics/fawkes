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
using namespace mongo;

/**
 * Constructor with references to objects of the plugin
 * @param robot_memory Robot Memory
 * @param blackboard Blackboard
 * @param logger Logger
 * @param config Configuration
 */
BlackboardComputable::BlackboardComputable(RobotMemory* robot_memory, fawkes::BlackBoard* blackboard, fawkes::Logger* logger, fawkes::Configuration* config)
{
  robot_memory_ = robot_memory;
  blackboard_ = blackboard;
  logger_ = logger;

  //register computable
  Query query = fromjson("{interface:{$exists:true}}");
  int priority = config->get_int("plugins/robot-memory/computables/blackboard/priority");
  float caching_time = config->get_float("plugins/robot-memory/computables/blackboard/caching-time");
  computable = robot_memory_->register_computable(query, "robmem.blackboard", &BlackboardComputable::compute_interfaces, this, caching_time, priority);
}

BlackboardComputable::~BlackboardComputable()
{
  robot_memory_->remove_computable(computable);
}

std::list<mongo::BSONObj> BlackboardComputable::compute_interfaces(mongo::BSONObj query, std::string collection)
{
  std::list<mongo::BSONObj> res;
  std::string type = query.getField("interface").String();
  std::string id = "*";
  if(query.hasField("id"))
    id = query.getField("id").String();
  //get all matching interfaces
  for(Interface* interface : blackboard_->open_multiple_for_reading(type.c_str(), id.c_str()))
  {
    interface->read();
    //build document
    BSONObjBuilder b;
    b << "interface" << interface->type()
        << "id" << interface->id();
    for(InterfaceFieldIterator it = interface->fields(); it != interface->fields_end(); ++it)
    {
      if(it.get_length() > 1 && it.get_type() != IFT_STRING)
      {
        b << it.get_name() << get_interface_fields(it);
      }
      else
      {
        switch(it.get_type())
        {
          case IFT_BOOL: b << it.get_name() << it.get_bool(); break;
          case IFT_INT8: b << it.get_name() << it.get_int8(); break;
          case IFT_UINT8: b << it.get_name() << it.get_uint8(); break;
          case IFT_INT16: b << it.get_name() << it.get_int16(); break;
          case IFT_UINT16: b << it.get_name() << it.get_uint16(); break;
          case IFT_INT32: b << it.get_name() << it.get_int32(); break;
          case IFT_UINT32: b << it.get_name() << it.get_uint32(); break;
          case IFT_INT64: b << it.get_name() << (long long int) it.get_int64(); break;
          case IFT_UINT64: b << it.get_name() << (unsigned int) it.get_uint64(); break;
          case IFT_FLOAT: b << it.get_name() << it.get_float(); break;
          case IFT_DOUBLE: b << it.get_name() << it.get_double(); break;
          case IFT_STRING: b << it.get_name() << it.get_string(); break;
          case IFT_BYTE: b << it.get_name() << it.get_byte(); break;
          case IFT_ENUM: b << it.get_name() << it.get_enum_string(); break;
        }
      }
    }
    res.push_back(b.obj());
    blackboard_->close(interface);
  }
  return res;
}

mongo::BSONArray BlackboardComputable::get_interface_fields(fawkes::InterfaceFieldIterator it)
{
  BSONArrayBuilder arr_b;
  for(unsigned int i = 0; i < it.get_length(); i++)
  {
    switch(it.get_type())
    {
      case IFT_BOOL: arr_b.appendBool(it.get_bool(i)); break;
      case IFT_INT8: arr_b.append(it.get_int8(i)); break;
      case IFT_UINT8: arr_b.append(it.get_uint8(i)); break;
      case IFT_INT16: arr_b.append(it.get_int16(i)); break;
      case IFT_UINT16: arr_b.append(it.get_uint16(i)); break;
      case IFT_INT32: arr_b.append(it.get_int32(i)); break;
      case IFT_UINT32: arr_b.append(it.get_uint32(i)); break;
      case IFT_INT64: arr_b.append((long long int) it.get_int64(i)); break;
      case IFT_UINT64: arr_b.append((unsigned int) it.get_uint64(i)); break;
      case IFT_FLOAT: arr_b.append(it.get_float(i)); break;
      case IFT_DOUBLE: arr_b.append(it.get_double(i)); break;
      case IFT_STRING: arr_b.append(it.get_string());
        return arr_b.arr();
      case IFT_BYTE: arr_b.append(it.get_byte(i)); break;
      case IFT_ENUM: arr_b.append(it.get_enum_string(i)); break;
    }
  }
  return arr_b.arr();
}
