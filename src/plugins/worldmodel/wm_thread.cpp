
/***************************************************************************
 *  wm_thread.cpp - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
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

#include "wm_thread.h"
#include "net_thread.h"

#include "fusers/single_copy.h"
#include "fusers/multi_copy.h"
#include "fusers/objpos_average.h"

#include <netcomm/worldinfo/transceiver.h>
#include <utils/system/pathparser.h>
#include <geometry/hom_point.h>
#include <geometry/hom_vector.h>

#include <interfaces/GameStateInterface.h>
#include <interfaces/ObjectPositionInterface.h>

#include <cmath>
#include <cstring>

using namespace std;
using namespace fawkes;

/** @class WorldModelThread <plugins/worldmodel/wm_thread.h>
 * Main thread of worldmodel plugin.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor.
 * @param net_thread pointer to a WorldModelNetworkThread
 */
WorldModelThread::WorldModelThread(WorldModelNetworkThread* net_thread)
  : Thread("WorldModelThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
  __net_thread = net_thread;

  __wi_send_enabled = false;
  __wi_send_interval = 20;
  __wi_send_counter  =  1;

  __wi_send_pose    = NULL;
  __wi_send_ball    = NULL;


  __wi_send_interval = 15;
  __wi_send_counter  =  1;
}


/** Destructor. */
WorldModelThread::~WorldModelThread()
{
}

void
WorldModelThread::init()
{
  try {
    __cfg_confspace = config->get_string("/worldmodel/confspace");

    logger->log_debug("WorldModelThread", "Config space: %s", __cfg_confspace.c_str());

    std::string prefix = "/worldmodel/interfaces/" + __cfg_confspace + "/";
    std::list<std::string> combos;
    // Read interfaces
    Configuration::ValueIterator *vi = config->search(prefix.c_str());
    while (vi->next()) {
      if (strcmp(vi->type(), "string") == 0) {
	PathParser pp(vi->path());
	if ( pp.size() > 1 ) {
	  combos.push_back(pp[pp.size() - 2]);
	}
      }
    }
    combos.sort();
    combos.unique();

    for (std::list<std::string>::iterator i = combos.begin(); i != combos.end(); ++i) {
      std::string type    = config->get_string((prefix + *i + "/type").c_str());
      std::string from_id = config->get_string((prefix + *i + "/from_id").c_str());
      std::string to_id   = config->get_string((prefix + *i + "/to_id").c_str());
      std::string method  = config->get_string((prefix + *i + "/method").c_str());

      if (method == "copy") {
	if (from_id.find_first_of("*?[") == std::string::npos) {
	  logger->log_debug(name(), "Instantiating SingleCopyFuser for %s -> %s (type: %s)",
			    from_id.c_str(), to_id.c_str(), type.c_str());
	  WorldModelSingleCopyFuser *fuser = new WorldModelSingleCopyFuser(blackboard, type.c_str(),
									   from_id.c_str(), to_id.c_str());
	  __fusers.push_back(fuser);
	} else {
	  logger->log_debug(name(), "Instantiating MultiCopyFuser for %s -> %s (type: %s)",
			    from_id.c_str(), to_id.c_str(), type.c_str());
	  WorldModelMultiCopyFuser *fuser = new WorldModelMultiCopyFuser(blackboard, type.c_str(),
									 from_id.c_str(), to_id.c_str());
	  __fusers.push_back(fuser);
	}
      } else if (method == "average") {
	// sanity checks
	if (type != "ObjectPositionInterface") {
	  throw Exception("Can only average interfaces of type ObjectPositionInterface");
	}
	logger->log_debug(name(), "Instantiating ObjPosAverageFuser for %s -> %s (type: %s)",
			  from_id.c_str(), to_id.c_str(), type.c_str());
	WorldModelObjPosAverageFuser *fuser = new WorldModelObjPosAverageFuser(logger, blackboard,
									       from_id.c_str(), to_id.c_str());
	  __fusers.push_back(fuser);

      } else {
	throw Exception("Unknown fuse method '%s', for interface %s -> %s (type %s)",
			method.c_str(), from_id.c_str(), to_id.c_str(), type.c_str());
      }
    }

  } catch (Exception &e) {
    e.print_trace();
  }

  __wi_send_enabled = false;
  try {
    std::string prefix = "/worldmodel/wi_send/" + __cfg_confspace + "/";
    __wi_send_enabled = config->get_bool((prefix + "enable_send").c_str());

    if (__wi_send_enabled) {
      logger->log_debug(name(), "Sending worldinfo messages enabled");

      std::string pose_id = config->get_string((prefix + "pose_id").c_str());
      std::string ball_id = config->get_string((prefix + "ball_id").c_str());

      logger->log_debug(name(), "Obtaining pose worldinfo data from interface %s.",
			pose_id.c_str());
      logger->log_debug(name(), "Obtaining ball worldinfo data from interface %s.",
			ball_id.c_str());
           
      __wi_send_pose      = blackboard->open_for_reading<ObjectPositionInterface>(pose_id.c_str());
      __wi_send_ball      = blackboard->open_for_reading<ObjectPositionInterface>(ball_id.c_str());
  
    } else {
      logger->log_debug(name(), "Sending worldinfo messages disabled");
    }
    
  } catch (Exception& e) {
    if ( __wi_send_enabled) {
      throw;
    } else {
      logger->log_debug(name(), "Sending worldinfo messages disabled (enable not set)");
    }
  }

}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
WorldModelThread::init_failure_cleanup()
{
}


void
WorldModelThread::finalize()
{
  for (__fit = __fusers.begin(); __fit != __fusers.end(); ++__fit) {
    delete *__fit;
  }
  __fusers.clear();

  if (__wi_send_enabled) {
    try {
      blackboard->close(__wi_send_pose);
      blackboard->close(__wi_send_ball);
    } catch (Exception& e) {
      e.print_trace();
    }
  }
}


void
WorldModelThread::loop()
{
  for (__fit = __fusers.begin(); __fit != __fusers.end(); ++__fit) {
    (*__fit)->fuse();
  }

  // only send every __wi_send_interval loop
  if ( 0 != (__wi_send_counter % __wi_send_interval) ) {
    ++__wi_send_counter;
    return;
  }

  __wi_send_counter = 1;

  WorldInfoTransceiver* transceiver = __net_thread->get_transceiver();

  if (__wi_send_enabled) {
    __wi_send_pose->read();
    __wi_send_ball->read();

    bool do_send = false;

    // pose
    HomPoint pos;
    pos.x( __wi_send_pose->world_x() );
    pos.y( __wi_send_pose->world_y() );
    float yaw = __wi_send_pose->yaw();
    if (__wi_send_pose->has_writer()) {
      do_send = true;
      transceiver->set_pose(pos.x(), pos.y(), yaw,
			    __wi_send_pose->world_xyz_covariance() /* TODO */);
      transceiver->set_velocity(__wi_send_pose->world_x_velocity(),
				__wi_send_pose->world_y_velocity(),
				__wi_send_pose->world_z_velocity(),
				__wi_send_pose->world_xyz_velocity_covariance());
      
      // ball
      if (__wi_send_ball->has_writer() && __wi_send_ball->is_valid()) {
	if (__wi_send_ball->flags() & ObjectPositionInterface::FLAG_HAS_WORLD) {
	  transceiver->set_glob_ball_pos(__wi_send_ball->world_x(),
					 __wi_send_ball->world_y(),
					 __wi_send_ball->world_z(),
					 __wi_send_ball->world_xyz_covariance() );
	} else {
	  // compute global ball position
	  HomVector relative_ball;
	  relative_ball.x( __wi_send_ball->relative_x() );
	  relative_ball.y( __wi_send_ball->relative_y() );
	  relative_ball.rotate_z( yaw );
	  HomPoint global_ball = pos + relative_ball;
	  
	  transceiver->set_glob_ball_pos(global_ball.x(), global_ball.y(), 0.0,
					 __wi_send_ball->dbs_covariance() /* TODO */);
	}
	transceiver->set_glob_ball_visible(__wi_send_ball->is_visible(),
					   __wi_send_ball->visibility_history());

	// TODO
// 	transceiver->set_glob_ball_velocity(__wi_send_ball->relative_x_velocity(),
// 					    __wi_send_ball->relative_y_velocity(),
// 					    __wi_send_ball->relative_z_velocity(),
// 					    __wi_send_ball->relative_xyz_velocity_covariance());
      }
    }
    
    if (do_send) {
      transceiver->send();
    }
  }
}
