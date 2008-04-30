
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
#include <cstring>

using namespace std;

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
    delete opi_list;

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

static void matrixToFloat( const Matrix &matrix, float* cov )
{
  unsigned int row, col;
  matrix.size( row, col );
  for ( unsigned int r = 0; r < row; ++r )
    for ( unsigned int c = 0; c < col; ++c )
      cov[(r*col) + c] = matrix( r, c );
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
  HomVector local_ball_pos;
  Matrix local_ball_cov;
  bool local_ball_visible = localBallPosition( local_ball_pos, local_ball_cov );

  float ball_cov[9];
  matrixToFloat( local_ball_cov, ball_cov );

  // report combined ball position we observed to the world
  // TODO ball_visible == false is not yet handled in the world info stuff, so better send nothing at all in this case
  if ( local_ball_visible && worldinfo_sender ) {
    logger->log_debug( name(), "Local ball position merged to: %f dist, %f rad", local_ball_pos.x(), local_ball_pos.y() );
    worldinfo_sender->set_ball_visible( true, 1 /* visibility history, not supported here yet either */ );
    worldinfo_sender->set_ball_pos( local_ball_pos.x(), local_ball_pos.y(), local_ball_pos.z(), ball_cov );
    worldinfo_sender->send();
  }

  HomVector global_ball_pos;
  Matrix global_ball_cov( 3, 3 );
  bool global_ball_visible = globalBallPosition( local_ball_visible, local_ball_pos, local_ball_cov,
                                                 global_ball_pos, global_ball_cov );

  // report merged ball position
  wm_ball_interface->set_object_type( ObjectPositionInterface::TYPE_BALL );
  wm_ball_interface->set_visible( local_ball_visible || global_ball_visible );
  unsigned int flags = ObjectPositionInterface::FLAG_HAS_COVARIANCES;
  if ( local_ball_visible ) {
    // TODO: we could calculate relative positions also based on the globals ones and our own position if available
    // which would be useful in at least two cases: no local sensor reading and improved accurancy due to sensor fusion
    logger->log_info( name(), "Estimated ball position [polar relative]: (%f,%f)", local_ball_pos.x(), local_ball_pos.y() );
    float dist = local_ball_pos.x();
    float bearing = local_ball_pos.y();
    wm_ball_interface->set_relative_x( dist * cosf( bearing ) );
    wm_ball_interface->set_relative_y( dist * sinf( bearing ) );
    wm_ball_interface->set_distance( dist );
    wm_ball_interface->set_bearing( bearing );
    matrixToFloat( local_ball_cov, ball_cov );
    wm_ball_interface->set_dbs_covariance( ball_cov );
    flags |= ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN |
             ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR;
  }
  if ( global_ball_visible ) {
    logger->log_info( name(), "Estimated ball position [cartesian global]: (%f,%f)", global_ball_pos.x(), global_ball_pos.y() );
    wm_ball_interface->set_world_x( global_ball_pos.x() );
    wm_ball_interface->set_world_y( global_ball_pos.y() );
    wm_ball_interface->set_world_z( global_ball_pos.z() );
    matrixToFloat( global_ball_cov, ball_cov );
    wm_ball_interface->set_world_xyz_covariance( ball_cov );
    flags |= ObjectPositionInterface::FLAG_HAS_WORLD;
  }
  wm_ball_interface->set_flags( flags );
  wm_ball_interface->write();


  // own pose
  if ( in_pose_interface->has_writer() )
    {
      float robot_x;
      float robot_y;
      float robot_theta;

      in_pose_interface->read();
      robot_x = in_pose_interface->world_x();
      robot_y = in_pose_interface->world_y();
      robot_theta = in_pose_interface->world_z();
      float *robot_cov = in_pose_interface->world_xyz_covariance();

      wm_pose_interface->set_world_x(robot_x);
      wm_pose_interface->set_world_y(robot_y);
      wm_pose_interface->set_world_z(robot_theta);
      wm_pose_interface->set_world_xyz_covariance( robot_cov );

      worldinfo_sender->set_pose( robot_x, robot_y, robot_theta, robot_cov );
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

/**
  Merge ball postions detected by local sensors.
  @param local_ball_pos Reference parameter to return the merged ball position (as relative polar coordinate).
  @param local_ball_cov Reference parameter to return the covariance of the merged ball position.
  @return @c true if at least one local sensor detected a ball.
*/
bool WorldModelThread::localBallPosition( HomVector & local_ball_pos, Matrix &local_ball_cov )
{
  // TODO: also consider covariance here and handle largely different values from different sensors somehow
  int count = 0;
  float dist = 0.0f, bearing = 0.0f;
  Matrix cov( 3, 3 );
  for ( LockList<ObjectPositionInterface *>::const_iterator iface = in_ball_interfaces->begin();
        iface != in_ball_interfaces->end(); ++iface ) {
    (*iface)->read();
    if ( !((*iface)->flags() & ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR) )
      continue; // the following code so far only handles relative polar coords, so skip the rest

    if ( !(*iface)->is_visible() )
      continue;

    ++count;
    dist += (*iface)->distance();
    bearing += (*iface)->bearing();
    cov += Matrix( 3, 3, (*iface)->dbs_covariance() );
  }
  if ( count == 0 )
    return false;
  local_ball_pos.x() = dist / count;
  local_ball_pos.y() = bearing / count;
  local_ball_pos.z() = 0.0f;
  local_ball_cov = cov / count;
  return true;
}

/**
  Merge all available ball information (local and remote).
  @param local_ball_available @c true if the following two parameters contain valid data.
  @param local_ball_pos Merged relative poloar coordinate of the ball based on local sensors.
  @param local_ball_cov Merged covarance of relative poloar coordinates of the ball based
  on local sensors.
  @param global_ball_pos Reference to the determined global cartesian ball coordinates.
  @param global_ball_cov Reference to the determined covariance of the previous parameter.
  @return @c true if there are any usable remote ball position information.
*/
bool WorldModelThread::globalBallPosition( bool localBallAvailable, const HomVector & local_ball_pos,
                                           const Matrix & local_ball_cov, HomVector & global_ball_pos,
                                           Matrix & global_ball_cov )
{
  int count = 0;
  global_ball_cov = Matrix( 3, 3 );

  // check if relative local position is usable to determine a global position
  if ( localBallAvailable ) {
    float cov[9];
    memcpy( cov, in_pose_interface->world_xyz_covariance(), 9 * sizeof(float) );
    if ( in_pose_interface->has_writer() && cov[0] <= 3.0 && cov[4] <= 3.0 && cov[8] <= 1.5 ) {
      ++count;
      global_ball_pos.x() += in_pose_interface->world_x() + (local_ball_pos.x() * cos( local_ball_pos.y() ));
      global_ball_pos.y() += in_pose_interface->world_y() + (local_ball_pos.x() * sin( local_ball_pos.y() ));
      global_ball_cov += Matrix( 3, 3, cov ) + local_ball_cov;
      logger->log_debug( name(), "Global ball position based on own localization: (%f,%f)", global_ball_pos.x(), global_ball_pos.y() );
    } else {
      logger->log_debug( name(), "My own localization is not precise enough to determine global ball position." );
    }
  }

  vector<string> hosts = data->get_hosts();
  for ( vector<string>::const_iterator it = hosts.begin(); it != hosts.end(); ++it ) {
    // TODO check ball visibility (not yet provided by WorldInfoDataContainer
    HomPose robotPose;
    Matrix robotPoseCov;
    if ( !data->get_robot_pose( (*it).c_str(), robotPose, robotPoseCov ) ) {
      logger->log_debug( name(), "%s has no pose in", (*it).c_str() );
      continue;
    }
    // he doesn't know himself where he is, ignore
    if ( robotPoseCov( 0, 0 ) > 3.0 || robotPoseCov( 1, 1 ) > 3.0 || robotPoseCov( 2, 2 ) > 1.5 ) {
      logger->log_debug( name(), "%s is not localized precisely enough (%f, %f, %f)", (*it).c_str(),
                         robotPoseCov( 0, 0 ), robotPoseCov( 1, 1 ), robotPoseCov( 2, 2 ) );
      continue;
    }
    HomPolar remoteBallPosition;
    Matrix remoteBallCov;
    if ( !data->get_ball_pos( (*it).c_str(), remoteBallPosition, remoteBallCov ) ) {
      logger->log_debug( name(), "%s does not provide ball position", (*it).c_str() );
      continue;
    }
    if ( remoteBallCov( 0, 0 ) > 3.0 || robotPoseCov( 1, 1 ) > 3.0 || robotPoseCov( 2, 2 ) > 1.5 ) {
      logger->log_debug( name(), "%s does not know ball position precisely enough", (*it).c_str() );
      continue;
    }
    ++count;
    remoteBallPosition.rotate_z( robotPose.yaw() );  // <-- ### remoteBallPosition is a relative cartesian coord!!
    global_ball_pos.x() += robotPose.x() + remoteBallPosition.x();
    global_ball_pos.y() += robotPose.y() + remoteBallPosition.y();
    global_ball_cov += robotPoseCov + remoteBallCov;
    logger->log_debug( name(), "%s provides usable ball position (%f,%f)", (*it).c_str(), remoteBallPosition.x(), remoteBallPosition.y() );
  }

  if ( count == 0 )
    return false;

  global_ball_pos.x() = global_ball_pos.x() / count;
  global_ball_pos.y() = global_ball_pos.y() / count;
  global_ball_pos.z() = 0.0f;
  global_ball_cov = global_ball_cov / count;
  return true;
}
