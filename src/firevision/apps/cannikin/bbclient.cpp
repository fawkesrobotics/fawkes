
/***************************************************************************
 *  bbclient.cpp - implementation of bbclient for cannikin
 *
 *  Generated: Tue Apr 10 14:31:49 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

/// @cond RCSoftX

#include "bbclient.h"
#include "pipeline.h"
#include "config.h"

#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>

#include <models/relative_position/relativepositionmodel.h>
#include <models/global_position/globalpositionmodel.h>
#include <models/scanlines/scanlinemodel.h>

#include <interfaces/localize_master_client.h>
#include <interfaces/ballpos_server.h>
#include <interfaces/camera_control_server.h>
#include <interfaces/alive_server.h>
#include <interfaces/cannikin_server.h>

#include <unistd.h>

/*
include "front/camera_tracker.h"
include "front/config.h"
*/

using namespace std;

// ==============================================================================
// Constructor
FirevisionCannikinBBClient::FirevisionCannikinBBClient(int argc, char* argv[], ArgumentParser *argp) :
  bb::ClientAppl(argc, argv) 
{
  this->argp = argp;
  BBOperate();
  msg_prefix = cblue + "FirevisionCannikinBBClient: " + cnormal;

  loop_running = false;
  exit_running = false;

  rob_pos_x = rob_pos_y = 0.0;
  rob_pos_x_old = rob_pos_y_old = 0.0;
  rob_pos_time.Stamp();
  rob_pos_time_old.Stamp();
  time_difference = 0.0;

  visibility_history = 0;

  pose_avg_dt = 0.f;
  pose_avg_num_samples = 0;

  show_pose_info = argp->has_arg("p");

  bearing_error = 0.05f;
  dist_error    = 0.5f;

  dist_total = bearing_total = 0.f;
  dist_num = bearing_num = 0;

  box_lost_time.Stamp();
  box_lost = true;

}


// ==============================================================================
// Destructor
FirevisionCannikinBBClient::~FirevisionCannikinBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void FirevisionCannikinBBClient::Init ()
{

  // get hostname
  std::string hostname = GetBBNames()[0];

  // Config stuff
  config = new CannikinConfig( m_pXMLConfigFile );

  // initialize box position server
  m_pBoxPosServer = new bbClients::BallPos_Server( hostname /* + "_cannikin" */ );
  BBRegisterObj( m_pBoxPosServer );


  /*
  // initialize localize master client
  m_pLocalizeMasterClient = new bbClients::Localize_Master_Client(hostname);
  BBRegisterObj( m_pLocalizeMasterClient );
  */

  // initialize camera control server
  m_pCameraControlServer = new bbClients::CameraControl_Server( hostname );
  BBRegisterObj( m_pCameraControlServer );

  m_pFrontAliveFakeServer = new bbClients::Alive_Server( "firevision_cannikin", hostname );
  BBRegisterObj( m_pFrontAliveFakeServer );

  m_pCannikinServer = new bbClients::Cannikin_Server( hostname );
  BBRegisterObj( m_pCannikinServer );

  BBOperate();
  m_pBoxPosServer->Update();  
  // m_pLocalizeMasterClient->Update();
  m_pCameraControlServer->Update();
  m_pCannikinServer->Update();
  BBOperate();

  // Call us every 100ms, this is likely to fail, but BB will just
  // drop that cycle...
  SetTime( 100 );

  pipeline = new CannikinPipeline(argp, config);
  pipeline->init();

  /*
  box_relative = pipeline->getRelativeBoxPosModel();
  box_global   = pipeline->getGlobalBoxPosModel();
  camctrl       = pipeline->getCameraControl();
  scanline_model = pipeline->getScanlineModel();
  */
}

void
FirevisionCannikinBBClient::cup_not_visible()
{
  m_pBoxPosServer->SetVisible( false );
  m_pBoxPosServer->SetConfidence( 0.f );
  m_pBoxPosServer->SetVisibilityHistory( visibility_history );

  m_pCannikinServer->SetVisible( false );

  dist_total = 0.f;
  dist_num   = 0;

  bearing_total = 0.f;
  bearing_num   = 0;

  if (! box_lost) {
    box_lost = true;
    box_lost_time.Stamp();
  }

}


// "Loop"
void
FirevisionCannikinBBClient::Loop(int Count)
{
  if ( exit_running || loop_running ) return;

  loop_running = true;

  BBOperate();
  
  // m_pLocalizeMasterClient->Update();
  m_pCameraControlServer->Update();
  m_pCannikinServer->Update();
  BBOperate();

  if ( m_pCannikinServer->ChangedMode() ) {
    pipeline->set_mode((CannikinPipeline::cannikin_mode_t)m_pCannikinServer->GetMode());
  }

  if ( m_pCannikinServer->ChangedCupColor() ) {
    cout << "Cup color changed" << endl;
    pipeline->set_cup_color((CannikinPipeline::cup_color_t)m_pCannikinServer->GetCupColor());
  }

  // m_pBoxPosServer->SetSource( bbClients::BallPos_Server::SOURCE_CANNIKIN );

  // Set the data the position model needs for useful calculations
  /*
  box_global->setRobotPosition( m_pLocalizeMasterClient->GetCurrentX(),
				 m_pLocalizeMasterClient->GetCurrentY(),
				 m_pLocalizeMasterClient->GetCurrentOri()
				 );
  timeval t;
  m_pLocalizeMasterClient->GetTimeVal(&t);
  if ( scanline_model != NULL) {
    scanline_model->setRobotPose( m_pLocalizeMasterClient->GetCurrentX(),
				  m_pLocalizeMasterClient->GetCurrentY(),
				  m_pLocalizeMasterClient->GetCurrentOri() );
  }



  if ( show_pose_info ) {
    pose_avg_dt += m_pLocalizeMasterClient->GetTimeDiff();
    ++pose_avg_num_samples;
    cout << cpurple << "RobotPos/Velo:" << cnormal
	 << "  x=" <<  m_pLocalizeMasterClient->GetCurrentX()
	 << "  y=" << m_pLocalizeMasterClient->GetCurrentY()
	 << "  ori=" <<  m_pLocalizeMasterClient->GetCurrentOri()
	 << "  rvx=" << m_pLocalizeMasterClient->GetCurrentRelVX()
	 << "  rvy=" << m_pLocalizeMasterClient->GetCurrentRelVY()
	 << "  dt=" << m_pLocalizeMasterClient->GetTimeDiff() << " sec"
	 << "  adt=" << (pose_avg_dt / pose_avg_num_samples)
	 << endl;
  }
  */

  // Do a pipeline cycle
  pipeline->loop();


  m_pCameraControlServer->SetCurrentPan(  config->CameraBearing  );
  m_pCameraControlServer->SetCurrentTilt( config->CameraSlope );
  m_pCameraControlServer->UpdateBB();
  BBOperate();
  // Also update to get the most current target values
  m_pCameraControlServer->Update();
  BBOperate();

  /* Update the information in the BB interface */

  if (pipeline->is_cup_visible() ) {

    /*
    box_glob_x = box_global->getX();
    box_glob_y = box_global->getY();

    // removed boundary check, makes no sense for cannikin
    // compare to front vision to understand
    box_rel_x   = box_relative->getX();
    box_rel_y   = box_relative->getY();
    box_dist    = box_relative->getDistance();
    box_bearing = box_relative->getBearing();
    box_slope   = box_relative->getSlope();

    if ( visibility_history > 0) {
      ++visibility_history;
    } else {
      visibility_history = 1;
    }

    m_pBoxPosServer->SetX( box_glob_x );
    m_pBoxPosServer->SetY( box_glob_y );
    m_pBoxPosServer->SetRelX( box_rel_x );
    m_pBoxPosServer->SetRelY( box_rel_y );

    box_lost = false;

    // Calculate distance error by averaging over the measured distances between
    // seen boxs.
    if ( visibility_history > 1 ) {
      // we have a last measurement to calculate distance
      dist_total    += fabs( last_dist - box_dist );
      ++dist_num;
      
      bearing_total += fabs( last_bearing - box_bearing );
      ++bearing_num;
    }
    last_dist    = box_dist;
    last_bearing = box_bearing;
    if ( dist_num > 1 ) {
      // we have at least two measurements
      dist_error    = dist_total    / dist_num;
      bearing_error = bearing_total / bearing_num;
    } else {
      // Default distance error
      bearing_error  = 0.05f;
      dist_error = 0.5f;
    }

    // ATTENTION!
    //
    // The relative velocity values are MISUSED in Cannikin/Meerkat. They
    // are used to transport the bearing and slope to the object and thus
    // it transports the base data. This is needed for now since we do not have
    // distance data at the moment and thus we need the bearing and then
    // match that with laser data in Meerkat.
    //
    // KEEP THIS IN MIND!
    m_pBoxPosServer->SetRelVelX( box_bearing  / * box_rel_vel_x * / );
    m_pBoxPosServer->SetRelVelY( box_slope    / * box_rel_vel_y * / );


    m_pBoxPosServer->SetVisible( true );
    m_pBoxPosServer->SetConfidence( 1.f );
    m_pBoxPosServer->SetVisibilityHistory( visibility_history );

    m_pBoxPosServer->SetBearingError( bearing_error );
    m_pBoxPosServer->SetDistanceError( dist_error );
    */

    if ( pipeline->is_cup_visible() ) {
      float x, y, z, wx, wy, wz;
      if ( pipeline->get_xyz(&x, &y, &z) &&
           pipeline->get_world_xyz(&wx, &wy, &wz) ) {
	m_pCannikinServer->SetVisible( true );
	m_pCannikinServer->SetCamX(x);
	m_pCannikinServer->SetCamY(y);
	m_pCannikinServer->SetCamZ(z);
	m_pCannikinServer->SetX(wx);
	m_pCannikinServer->SetY(wy);
	m_pCannikinServer->SetZ(wz);

        float bearing = atan2f(wy, wx);
        float slope = asin(wx / z);

        m_pCannikinServer->SetBearing(bearing + /* EVIL EVIL HACK */ deg2rad(3) );
        m_pCannikinServer->SetSlope(slope);
      } else {
	cup_not_visible();
      }
    } else {
      cup_not_visible();
    }

  } else {
    cout << msg_prefix << cred << "Box is NOT visible" << cnormal << endl;

    cup_not_visible();

  }
  m_pBoxPosServer->UpdateBB();

  m_pCannikinServer->SetAlive( true );
  m_pCannikinServer->SetCurrentMode((int)pipeline->mode());
  m_pCannikinServer->SetDeterminationDone(pipeline->done_determining_cup_color());
  m_pCannikinServer->SetDeterminedCupColor((int)pipeline->determined_cup_color());
  m_pCannikinServer->SetCurrentCupColor((int)pipeline->cup_color());

  m_pCannikinServer->UpdateBB();

  m_pFrontAliveFakeServer->PingAlive();
  m_pFrontAliveFakeServer->UpdateBB();

  BBPing();
  BBOperate();

  loop_running = false;

}


// ===================================================================================
// "Exit"

void
FirevisionCannikinBBClient::Exit()
{

  exit_running = true;

  cout << "FirevisionCannikinBBClient exiting" << endl;

  while ( loop_running ) {
    cout << "w" << flush;
    usleep( 0 );
  }
  cout << endl;

  // before leaving, tell bb that I cannot see box
  // (otherwise the last box position remains in bb forever)
  m_pBoxPosServer->SetVisible (false);
  m_pBoxPosServer->SetConfidence (0.0);
  m_pBoxPosServer->SetVisibilityHistory( 0 );
  m_pBoxPosServer->UpdateBB();
  BBPing();
  BBOperate();

  pipeline->finalize();

  delete m_pBoxPosServer;
  // delete m_pLocalizeMasterClient;
  delete m_pCameraControlServer;
  delete pipeline;

  exit_running = false;

}

/// @endcond RCSoftX
