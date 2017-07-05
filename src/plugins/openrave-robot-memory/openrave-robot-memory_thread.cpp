
/***************************************************************************
 *  openrave-robot-memory_thread.cpp - openrave-robot-memory
 *
 *  Created: Thu Nov 24 13:14:33 2016
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

#include "openrave-robot-memory_thread.h"
#include <algorithm>

using namespace fawkes;
using namespace mongo;

/** @class OpenraveRobotMemoryThread 'openrave-robot-memory_thread.h' 
 * Creates an OpenRave Scene for motion planning from data in the robot memory
 * @author Frederik Zwilling
 */

/** Constructor. */
OpenraveRobotMemoryThread::OpenraveRobotMemoryThread()
 : Thread("OpenraveRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
   BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT) 
{
}

void
OpenraveRobotMemoryThread::init()
{
  openrave_if_ = blackboard->open_for_reading<OpenRaveInterface>(config->get_string("plugins/openrave-robot-memory/openrave-if-name").c_str());
  or_rm_if_ = blackboard->open_for_writing<OpenraveRobotMemoryInterface>(config->get_string("plugins/openrave-robot-memory/if-name").c_str());
}

void
OpenraveRobotMemoryThread::loop()
{
  // process interface messages
  while (! or_rm_if_->msgq_empty() ) {
    if (or_rm_if_->msgq_first_is<OpenraveRobotMemoryInterface::ConstructSceneMessage>()) {
      construct_scene();
    } else {
      logger->log_warn(name(), "Unknown message received");
    }
    or_rm_if_->msgq_pop();
  }
}

void
OpenraveRobotMemoryThread::finalize()
{
}

void
OpenraveRobotMemoryThread::construct_scene()
{
  logger->log_info(name(), "Constructing Scene");

  // add all object types by iterating over config paths
  std::string prefix = "plugins/openrave-robot-memory/object-types/";
  std::unique_ptr<Configuration::ValueIterator> object_types(config->search(prefix.c_str()));
  while(object_types->next())
  {
    //object_types->next() yields the whole path, so we have to get the right prefix
    std::string cfg_name = std::string(object_types->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));
    //don't use the same prefix again
    if(std::find(added_object_types_.begin(), added_object_types_.end(), cfg_name) != added_object_types_.end())
      continue;
    added_object_types_.push_back(cfg_name);
    logger->log_info(name(), "Adding object type: %s", cfg_name.c_str());
    std::string cfg_prefix = prefix + cfg_name + "/";
    std::string collection = config->get_string(cfg_prefix + "collection");

    //construct query
    BSONObjBuilder query_builder;
    query_builder.appendElements(fromjson(config->get_string(cfg_prefix + "query")));
    query_builder << "frame" << "base_link" << "allow_tf" << true;
    BSONObj query = query_builder.obj();
    logger->log_info(name(), "Querying: %s", query.toString().c_str());
    QResCursor cur = robot_memory->query(query, collection);
    while(cur->more())
    {
      BSONObj block = cur->next();
      //logger->log_info(name(), "Adding: %s", cfg_prefix.c_str(), block.toString().c_str());
      std::string block_name = block.getStringField(config->get_string(cfg_prefix + "name-key"));
      if(std::find(added_objects_.begin(), added_objects_.end(), block_name) == added_objects_.end())
      {
        //add new object
        logger->log_info(name(), "adding %s", block_name.c_str());
        OpenRaveInterface::AddObjectMessage add_msg;
        add_msg.set_name(block_name.c_str());
        add_msg.set_path(config->get_string(cfg_prefix + "model-path").c_str());
        openrave_if_->msgq_enqueue_copy(&add_msg);
        added_objects_.push_back(block_name);
      }
      //move object to right position
      OpenRaveInterface::MoveObjectMessage move_msg;
      move_msg.set_name(block_name.c_str());
      move_msg.set_x(block.getField("translation").Array()[0].Double());
      move_msg.set_y(block.getField("translation").Array()[1].Double());
      move_msg.set_z(block.getField("translation").Array()[2].Double());
      openrave_if_->msgq_enqueue_copy(&move_msg);
      //rotate object
      OpenRaveInterface::RotateObjectQuatMessage rotate_msg;
      rotate_msg.set_name(block_name.c_str());
      rotate_msg.set_x(block.getField("rotation").Array()[0].Double());
      rotate_msg.set_y(block.getField("rotation").Array()[1].Double());
      rotate_msg.set_z(block.getField("rotation").Array()[2].Double());
      rotate_msg.set_w(block.getField("rotation").Array()[3].Double());
      openrave_if_->msgq_enqueue_copy(&rotate_msg);
    }
  }
  added_object_types_.clear();
  logger->log_info(name(), "Finished Constructing Scene");
}
