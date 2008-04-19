
/***************************************************************************
 *  wm_thread.cpp - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:56:48 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <plugins/worldmodel/wm_thread.h>
#include <plugins/worldmodel/net_thread.h>
#include <netcomm/worldinfo/transceiver.h>
#include <worldinfo_utils/data_container.h>

#include <interfaces/gamestate.h>
#include <interfaces/object.h>

#include <cmath>

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
  this->net_thread = net_thread;
  this->data = 0;
  worldinfo_sender = 0;
}


/** Destructor. */
WorldModelThread::~WorldModelThread()
{
}


void
WorldModelThread::bb_interface_created(const char *type, const char *id) throw()
{
  try {
    // object position interface with id "Ball*" created
    if ( strncmp(id, "Ball", strlen("Ball")) == 0 ) {
      ObjectPositionInterface *opi = blackboard->open_for_reading<ObjectPositionInterface>(id);
      in_ball_interfaces->push_back_locked(opi);
      BlackboardNotificationProxy *proxy = new BlackboardNotificationProxy(this);
      proxy->add_interface(opi);
      blackboard->register_listener( proxy, BlackBoard::BBIL_FLAG_WRITER | BlackBoard::BBIL_FLAG_READER );
      proxy_map[ opi->serial() ] = proxy;
    } 
  } catch (Exception &e) {
    logger->log_error(name(), "Tried to open new %s interface instance "
		      "'%s', failed, ignoring this interface.", type, id);
    logger->log_error(name(), e);
  }
}


void
WorldModelThread::bb_interface_destroyed(const char *type, const char *id) throw()
{
}


void
WorldModelThread::bb_interface_reader_added(Interface *interface, unsigned int instance_serial)  throw()
{
}


void
WorldModelThread::bb_interface_reader_removed(Interface *interface, unsigned int instance_serial)  throw()
{
  if ( !interface->has_writer() && ( interface->num_readers() == 1 ) ) {

    if ( 0 == strncmp(interface->id(), "Ball", strlen("Ball")) ) {

      // removing interface from list of ball interfaces
      in_ball_interfaces->lock();
      for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
	if ( (*opii)->serial() == interface->serial() ) {
	  in_ball_interfaces->erase(opii);
	  break;
	}
      }
      in_ball_interfaces->unlock();
      
      // mark corresponding proxy for deletion
      ProxyMap::iterator i = proxy_map.find( interface->serial() );
      if ( i != proxy_map.end() ) {
	proxy_delete_list.push_back_locked(i->second);
	proxy_map.erase(i);
      }

      // actually close interface
      blackboard->close(interface);
    }

    // TODO: opponent interfaces
  }
}


void
WorldModelThread::bb_interface_writer_added(Interface *interface, unsigned int instance_serial)  throw()
{
}


void
WorldModelThread::bb_interface_writer_removed(Interface *interface, unsigned int instance_serial)  throw()
{
  if ( interface->num_readers() == 1 ) {
    
    // writer of Ball* interface removed
    if ( 0 == strncmp( interface->id(), "Ball", strlen("Ball") ) ) {

      // removing interface from list of ball interfaces
      in_ball_interfaces->lock();
      for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
	if ( (*opii)->serial() == interface->serial() ) {
	  in_ball_interfaces->erase(opii);
	  break;
	}
      }
      in_ball_interfaces->unlock();

      // mark corresponding proxy for deletion
      ProxyMap::iterator i = proxy_map.find( interface->serial() );
      if ( i != proxy_map.end() ) {
	proxy_delete_list.push_back_locked( i->second );
	proxy_map.erase(i);
      }

      // actually close interface
      blackboard->close(interface);      
    }

    // TODO: opponents
  }
}


void
WorldModelThread::init()
{
  bbio_add_interface_create_type("ObjectPositionInterface");
  bbio_add_interface_destroy_type("ObjectPositionInterface");

  wm_game_state_interface = NULL;
  wm_ball_interface       = NULL;
  wm_pose_interface       = NULL;
  //wm_opp_interfaces      = NULL;

  in_ball_interfaces = NULL;
  in_pose_interface  = NULL;
  //in_opp_interfaces = NULL;

  try {
    wm_game_state_interface = blackboard->open_for_writing<GameStateInterface>("WM GameState");
    wm_ball_interface = blackboard->open_for_writing<ObjectPositionInterface>("WM Ball");
    wm_pose_interface = blackboard->open_for_writing<ObjectPositionInterface>("WM Pose");
    //wm_opp_interfaces = new std::list<ObjectPositionInterface *>();
    
    // pose
    in_pose_interface = blackboard->open_for_reading<ObjectPositionInterface>("OmniLocalize");

    // balls
    in_ball_interfaces = new LockList<ObjectPositionInterface *>;
    std::list<ObjectPositionInterface *> *opi_list = blackboard->open_all_of_type_for_reading<ObjectPositionInterface>("Ball");
    for ( std::list<ObjectPositionInterface *>::iterator iter = opi_list->begin();
	  iter != opi_list->end(); ++iter ) { 
      in_ball_interfaces->push_back_locked( *iter );
    }

    for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
      BlackboardNotificationProxy* proxy = new BlackboardNotificationProxy(this);
      proxy->add_interface( *opii );
      blackboard->register_listener( proxy, BlackBoard::BBIL_FLAG_WRITER | BlackBoard::BBIL_FLAG_READER );
      proxy_map[ (*opii)->serial() ] = proxy;
    }

    // TODO: opponents
    
  } catch (Exception &e) {
    init_failure_cleanup();
    e.append("WorldModel::init() failed");
    throw;
  }

  blackboard->register_observer(this, BlackBoard::BBIO_FLAG_CREATED | 
				      BlackBoard::BBIO_FLAG_DESTROYED);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_WRITER | 
				      BlackBoard::BBIL_FLAG_READER);
}


/** Clean up when init failed.
 * You may only call this from init(). Never ever call it from anywhere
 * else!
 */
void
WorldModelThread::init_failure_cleanup()
{
  try {
    if ( wm_game_state_interface )  blackboard->close(wm_game_state_interface);
    if ( wm_ball_interface )  blackboard->close(wm_ball_interface);
    if ( wm_pose_interface )  blackboard->close(wm_pose_interface);

    if ( in_pose_interface )  blackboard->close(in_pose_interface);
    if ( in_ball_interfaces ) {
      for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
	blackboard->close(*opii);
      }
      delete in_ball_interfaces;
    }
    //     if ( in_opp_interfaces ) {
    //       for (opii = in_opp_interfaces->begin(); opii != in_opp_interfaces->end(); ++opii) {
    // 	blackboard->close(*opii);
    //       }
    //       delete in_opp_interfaces;
    //     }
  } catch (...) {
    // we really screwed up, can't do anything about it, ignore error, logger is
    // initialized since this method is only called from init() which is only called if
    // all aspects had been initialized successfully
    logger->log_error(name(), "Really screwed up while finalizing, aborting cleanup. "
		              "Fawkes is no longer in a clean state. Restart!");
  }

  ProxyMap::iterator i;
  for ( i = proxy_map.begin(); i != proxy_map.end(); ++i ) {
    blackboard->unregister_listener( i->second );
    delete i->second;
  }
}


void
WorldModelThread::finalize()
{
  blackboard->close(wm_game_state_interface);
  blackboard->close(wm_ball_interface);
  blackboard->close(wm_pose_interface);
  //   for (wm_opp_interfaces->begin(); opii != wm_opp_interfaces->end(); ++opii) {
  //     blackboard->close(*opii);    
  //   }
  //   delete wm_opp_interfaces;
  
  blackboard->close(in_pose_interface);

  for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii) {
    blackboard->close(*opii);    
  }
  delete in_ball_interfaces;

  // delete active notification proxies
  for (std::map<unsigned int, BlackboardNotificationProxy * >::iterator i = proxy_map.begin();
       i != proxy_map.end(); ++i) {
    blackboard->unregister_listener( i->second );
    delete i->second;
  }

  // delete notification proxies marked for deletion
  proxy_delete_list.lock();
  while ( proxy_delete_list.size() != 0 ) {
    BlackboardNotificationProxy* proxy = proxy_delete_list.front();
    blackboard->unregister_listener(proxy);
    delete proxy;
    proxy_delete_list.pop_front();
  }
  proxy_delete_list.unlock();

  blackboard->unregister_observer(this);
  blackboard->unregister_listener(this);
}


void
WorldModelThread::loop()
{
  if ( !worldinfo_sender )
    { worldinfo_sender = net_thread->get_transceiver(); }

  if ( !data )
    { data = net_thread->get_data_container(); }

  // unregister and delete proxies marked for deletion
  proxy_delete_list.lock();
  while ( proxy_delete_list.size() != 0 ) {
    BlackboardNotificationProxy* proxy = proxy_delete_list.front();
    blackboard->unregister_listener(proxy);
    delete proxy;
    proxy_delete_list.pop_front();
  }
  proxy_delete_list.unlock();

  // game state
  WorldInfoDataContainer::GameState game_state;
  game_state = data->get_game_state();
  worldinfo_gamestate_team_t our_team_color = data->get_own_team_color();
  worldinfo_gamestate_goalcolor_t our_goal_color = data->get_own_goal_color();

  if (our_team_color == TEAM_CYAN) {
    wm_game_state_interface->set_our_team( GameStateInterface::TEAM_CYAN );
  } else {
    wm_game_state_interface->set_our_team( GameStateInterface::TEAM_MAGENTA );
  }

  if (our_goal_color == GOAL_BLUE) {
    wm_game_state_interface->set_our_goal_color( GameStateInterface::GOAL_BLUE );
  } else {
    wm_game_state_interface->set_our_goal_color( GameStateInterface::GOAL_YELLOW );
  }

  wm_game_state_interface->set_score_cyan(game_state.score_cyan);
  wm_game_state_interface->set_score_magenta(game_state.score_magenta);

  if (game_state.half == HALF_FIRST) {
    wm_game_state_interface->set_half( GameStateInterface::HALF_FIRST );
  } else {
    wm_game_state_interface->set_half( GameStateInterface::HALF_SECOND );
  }

  switch (game_state.game_state)
    {
    case(GS_FROZEN):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_FROZEN );
      break;

    case(GS_PLAY):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_PLAY );
      break;
	
    case(GS_KICK_OFF):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_KICK_OFF );
      break;
	
    case(GS_DROP_BALL):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_DROP_BALL );
      break;
	
    case(GS_PENALTY):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_PENALTY );
      break;
	
    case(GS_CORNER_KICK):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_CORNER_KICK );
      break;
	
    case(GS_THROW_IN):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_THROW_IN );
      break;
    
    case(GS_FREE_KICK):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_FREE_KICK );
      break;

    case(GS_GOAL_KICK):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_GOAL_KICK );
      break;

    case(GS_HALF_TIME):
      wm_game_state_interface->set_game_state( GameStateInterface::GS_HALF_TIME );
      break;
    }

  switch (game_state.state_team)
    {
    case(TEAM_NONE):
      wm_game_state_interface->set_state_team( GameStateInterface::TEAM_NONE );
      break;

    case(TEAM_CYAN):
      wm_game_state_interface->set_state_team( GameStateInterface::TEAM_CYAN );
      break;

    case(TEAM_MAGENTA):
      wm_game_state_interface->set_state_team( GameStateInterface::TEAM_MAGENTA );
      break;

    case(TEAM_BOTH):
      wm_game_state_interface->set_state_team( GameStateInterface::TEAM_BOTH );
      break;
    }

  wm_game_state_interface->write();

  // ball
  // TODO: currently it just looks for the first interface that starts with "BallOmni"
  for (opii = in_ball_interfaces->begin(); opii != in_ball_interfaces->end(); ++opii)
    {
      if ( strcmp("BallOmni", (*opii)->id()) != 0)
	{ continue; }

      ObjectPositionInterface* ball_interface;
      ball_interface = *( in_ball_interfaces->begin() );
      ball_interface->read();
      bool visible = ball_interface->is_visible();
      float rel_x = ball_interface->relative_x();
      float rel_y = ball_interface->relative_y();
      wm_ball_interface->set_object_type( ObjectPositionInterface::TYPE_BALL );
      wm_ball_interface->set_visible(visible);
      wm_ball_interface->set_flags( ball_interface->flags() );
      wm_ball_interface->set_relative_x(rel_x);
      wm_ball_interface->set_relative_y(rel_y);
      wm_ball_interface->set_distance( ball_interface->distance() );
      wm_ball_interface->set_bearing( ball_interface->bearing() );
      wm_ball_interface->write();
      
      if (worldinfo_sender) {
	float dist = sqrt(rel_x * rel_x + rel_y * rel_y);
	float bearing = atan2f(rel_y, rel_x);
	float *cov = ball_interface->dyp_covariance();
	worldinfo_sender->set_ball_pos(dist, bearing, 0.0 /* slope */, cov);
	worldinfo_sender->set_ball_visible(visible, 1);
	worldinfo_sender->send();
      }
    }

  // own pose
  if ( in_pose_interface->has_writer() )
    {
      float robot_x;
      float robot_y;
      float robot_theta;
      
      in_pose_interface->read();
      robot_x = in_pose_interface->world_x();
      robot_y = in_pose_interface->world_y();
      robot_theta = in_pose_interface->yaw();
      
      wm_pose_interface->set_world_x(robot_x);
      wm_pose_interface->set_world_y(robot_y);
      wm_pose_interface->set_yaw(robot_theta);
      
      worldinfo_sender->set_pose( robot_x, robot_y, robot_theta, 
				  in_pose_interface->dyp_covariance() );
      worldinfo_sender->send();
    }

  // poses of team members
  // TODO:

  // opponent poses
  // TODO:
}


WorldModelThread::BlackboardNotificationProxy::BlackboardNotificationProxy(BlackBoardInterfaceListener *listener)
{
  this->listener = listener;
}

WorldModelThread::BlackboardNotificationProxy::~BlackboardNotificationProxy()
{
}

void
WorldModelThread::BlackboardNotificationProxy::bb_interface_reader_added(Interface *interface, unsigned int instance_serial) throw() 
{
  listener->bb_interface_reader_added(interface, instance_serial);
}

void
WorldModelThread::BlackboardNotificationProxy::bb_interface_reader_removed(Interface *interface, unsigned int instance_serial) throw() 
{
  listener->bb_interface_reader_removed(interface, instance_serial);
}

void
WorldModelThread::BlackboardNotificationProxy::bb_interface_writer_added(Interface *interface, unsigned int instance_serial) throw()
{
  listener->bb_interface_writer_added(interface, instance_serial);
}

void
WorldModelThread::BlackboardNotificationProxy::bb_interface_writer_removed(Interface *interface, unsigned int instance_serial) throw()
{
  listener->bb_interface_writer_removed(interface, instance_serial);
}

void
WorldModelThread::BlackboardNotificationProxy::add_interface(Interface *interface) 
{
  bbil_add_reader_interface(interface);
  bbil_add_writer_interface(interface);
}
