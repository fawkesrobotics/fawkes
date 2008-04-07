
/***************************************************************************
 *  bbclient.cpp - implementation of bbclient for geegaw
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
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
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

#include <fvutils/camera/tracker.h>

#include <cams/camera.h>
#include <cams/cameracontrol.h>

#include <utils/system/argparser.h>
#include <utils/system/console_colors.h>

#include <models/relative_position/relativepositionmodel.h>
#include <models/global_position/globalpositionmodel.h>
#include <models/scanlines/scanlinemodel.h>

#include <interfaces/localize_master_client.h>
#include <interfaces/ballpos_server.h>
#include <interfaces/camera_control_server.h>
#include <interfaces/alive_server.h>
#include <interfaces/vision_obstacles_server.h>
#include <interfaces/geegaw_server.h>

#include <unistd.h>

/*
include "front/camera_tracker.h"
include "front/config.h"
*/

using namespace std;

// ==============================================================================
// Constructor
FirevisionGeegawBBClient::FirevisionGeegawBBClient(int argc, char* argv[],
						   ArgumentParser *argp)
  : bb::ClientAppl(argc, argv) 
{
  this->argp = argp;
  BBOperate();
  msg_prefix = cblue + "FirevisionGeegawBBClient: " + cnormal;

  loop_running = false;
  exit_running = false;

  rob_pos_x = rob_pos_y = 0.0;
  rob_pos_x_old = rob_pos_y_old = 0.0;
  rob_pos_time.Stamp();
  rob_pos_time_old.Stamp();
  time_difference = 0.0;

  pose_avg_dt = 0.f;
  pose_avg_num_samples = 0;

  show_pose_info = argp->arg("p");

  bearing_error = 0.05f;
  dist_error    = 0.5f;

  dist_total = bearing_total = 0.f;
  dist_num = bearing_num = 0;

  forward_pan  = 0.f;
  forward_tilt = 0.f;

  new_pan = new_tilt = last_pan = last_tilt = 0.f;

  box_lost_time.Stamp();
  box_lost = true;
}


// ==============================================================================
// Destructor
FirevisionGeegawBBClient::~FirevisionGeegawBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void FirevisionGeegawBBClient::Init ()
{

  // get hostname
  std::string hostname = GetBBNames()[0];

  // Config stuff
  config = new GeegawConfig( m_pXMLConfigFile );

  forward_pan  = deg2rad( config->ForwardPan  );
  forward_tilt = deg2rad( config->ForwardTilt );

  // initialize box position server
  m_pVisObsServer = new bbClients::VisionObstacles_Server( hostname );
  BBRegisterObj( m_pVisObsServer );


  // initialize localize master client
  m_pLocalizeMasterClient = new bbClients::Localize_Master_Client(hostname);
  BBRegisterObj( m_pLocalizeMasterClient );

  // initialize obj position server
  m_pObjPosServer = new bbClients::BallPos_Server( hostname );
  BBRegisterObj( m_pObjPosServer );

  // initialize geegaw server
  m_pGeegawServer = new bbClients::Geegaw_Server( hostname );
  BBRegisterObj( m_pGeegawServer );

  // initialize camera control server
  m_pCameraControlServer = new bbClients::CameraControl_Server( hostname );
  BBRegisterObj( m_pCameraControlServer );

  m_pFrontAliveFakeServer = new bbClients::Alive_Server( "firevision_front", hostname );
  BBRegisterObj( m_pFrontAliveFakeServer );

  BBOperate();
  m_pLocalizeMasterClient->Update();
  m_pCameraControlServer->Update();
  m_pObjPosServer->Update();  
  m_pGeegawServer->Update();
  BBOperate();

  SetTime( 40 );

  pipeline = new GeegawPipeline(argp, config);
  pipeline->init();

  camctrl       = pipeline->getCameraControl();

  try {
    camctrl->set_pan_tilt_rad( forward_pan, forward_tilt );
  } catch (Exception &e) {
    cout << "Caught exception, 1" << endl;
    e.print_trace();
  }

  obj_relative = pipeline->object_relpos();
  camera_tracker = new CameraTracker( obj_relative,
                                      config->CameraHeight,
                                      config->CameraOrientation);
  camera_tracker->setMode(CameraTracker::MODE_MODEL);

  /*
  box_relative = pipeline->getRelativeBoxPosModel();
  box_global   = pipeline->getGlobalBoxPosModel();
  scanline_model = pipeline->getScanlineModel();
  */
}


// "Loop"
void
FirevisionGeegawBBClient::Loop(int Count)
{
  if ( exit_running || loop_running ) return;

  loop_running = true;

  m_pCameraControlServer->Update();
  m_pObjPosServer->Update();
  m_pLocalizeMasterClient->Update();
  m_pGeegawServer->Update();
  BBOperate();

  if ( m_pGeegawServer->ChangedMode() ) {
    int mode = m_pGeegawServer->GetMode();
    cout << "Switching to mode " << mode << endl;
    pipeline->setMode((GeegawPipeline::GeegawOperationMode)mode);
    m_pGeegawServer->SetCurrentMode(mode);
    m_pGeegawServer->UpdateBB();
    BBOperate();
  }

  if ( m_pGeegawServer->ChangedColormap() ) {
    cout << msg_prefix << "Loading colormap " << m_pGeegawServer->GetColormap() << endl;
    pipeline->setColormap(m_pGeegawServer->GetColormap());
    m_pGeegawServer->SetCurrentColormap(m_pGeegawServer->GetColormap());
    m_pGeegawServer->UpdateBB();
    BBOperate();
  }

  // Do a pipeline cycle
  pipeline->loop();

  if ( pipeline->addStatusChanged() ) {
    cout << msg_prefix << "add status changed to " << pipeline->addStatus() << endl;
    m_pGeegawServer->SetObjectAddStatus(pipeline->addStatus());
    m_pGeegawServer->UpdateBB();
    BBOperate();
  }

  float pan = 0.f, tilt = 0.f;
  pipeline->pan_tilt(&pan, &tilt);

  m_pCameraControlServer->SetCurrentPan(  pan  );
  m_pCameraControlServer->SetCurrentTilt( tilt );
  m_pCameraControlServer->UpdateBB();
  BBOperate();
  // Also update to get the most current target values
  m_pCameraControlServer->Update();
  BBOperate();

  /* Update the information in the BB interface */
  if (pipeline->obstacles_found() ) {

    if ( pipeline->getMode() == GeegawPipeline::MODE_OBSTACLES ) {

      std::list<polar_coord_t> & obstacles = pipeline->getObstacles();
      std::list<polar_coord_t>::iterator i;
      int cur_obs = 0;
      //cout << "======================================================================" << endl;
      for (i = obstacles.begin(); i != obstacles.end(); ++i) {
        m_pVisObsServer->SetPosRelAngle( (*i).phi, cur_obs );
        m_pVisObsServer->SetPosRelDist( (*i).r, cur_obs );
        m_pVisObsServer->SetPosWidth( 0.1, cur_obs );
        m_pVisObsServer->SetPosHeight( 0.4, cur_obs );
        //cout << "Writing obstacle " << cur_obs << " to BB: angle=" << (*i).phi << "  dist="
        //     << (*i).r << endl;
        ++cur_obs;
        if ( cur_obs == bbClients::VisionObstacles_Client::MAX_NUM_OF_OBSTACLES ) {
          break;
        }
      }
      m_pVisObsServer->SetNumberOfObstacles(cur_obs);
    } else {
      m_pVisObsServer->SetNumberOfObstacles(0);
    }

    if ( visibility_history < 0 ) {
      visibility_history = 1;
    } else {
      ++visibility_history;
    }

    box_lost = false;

    // Misusing fields here!
    m_pObjPosServer->SetRelVelX( pipeline->object_bearing() );
    m_pObjPosServer->SetRelVelY( pipeline->object_distance() );
    m_pObjPosServer->SetConfidence( 1.f );
    m_pObjPosServer->SetVisible( true );

    if ( pipeline->getMode() == GeegawPipeline::MODE_OBSTACLES ) {
      camera_tracker->calc();
      new_pan = camera_tracker->getNewPan();
      new_tilt = forward_tilt;
    }

  } else {
    if ( pipeline->getMode() == GeegawPipeline::MODE_OBSTACLES ) {
      cout << msg_prefix << cred << "No obstacles found" << cnormal << endl;
    }
    m_pVisObsServer->SetNumberOfObstacles( 0 );
    if ( visibility_history > 0 ) {
      visibility_history = -1;
    } else {
      --visibility_history;
    }
    m_pObjPosServer->SetVisible( false );
    m_pObjPosServer->SetConfidence( 0.f );
    if ( pipeline->getMode() == GeegawPipeline::MODE_OBSTACLES ) {
      new_pan  = forward_pan;
      new_tilt = forward_tilt;
    } else {
      if (! box_lost) {
	box_lost = true;
	box_lost_time.Stamp();
      }
    }

  }
  m_pVisObsServer->UpdateBB();
  m_pObjPosServer->SetVisibilityHistory( visibility_history );
  m_pObjPosServer->UpdateBB();

  m_pFrontAliveFakeServer->PingAlive();
  m_pFrontAliveFakeServer->UpdateBB();

  BBPing();
  BBOperate();

  if ( pipeline->getMode() == GeegawPipeline::MODE_LOSTNFOUND ) {
    tracking_mode = m_pCameraControlServer->GetTrackingMode();
    m_pCameraControlServer->SetCurrentTrackingMode( tracking_mode );
    
    if ( tracking_mode == bbClients::CameraControl_Client::TRACKING_NONE ) {
      // no tracking
    //cout << "Tracking OFF." << endl;
      if (m_pCameraControlServer->ChangedTargetPan() ||
	  m_pCameraControlServer->ChangedTargetTilt() ) {
	// For the Leutron cam we know that this has to be set in one go
	// anyway so we do this
	new_pan  = m_pCameraControlServer->GetTargetPan();
	new_tilt = m_pCameraControlServer->GetTargetTilt();
      }
      
    } else if ( tracking_mode == bbClients::CameraControl_Client::TRACKING_BALL ) {
      // Track the box
      //cout << "Tracking BOX." << endl;
      now.Stamp();
      if ( box_lost &&
	   ((now - box_lost_time) >= config->ForwardDelay) ) {
	new_pan  = forward_pan;
	new_tilt = forward_tilt;
      } else {
	camera_tracker->setMode( CameraTracker::MODE_MODEL );
	camera_tracker->calc();
	new_pan  = camera_tracker->getNewPan();
	//if (config->CameraTrackTilt) {
	//  new_tilt = camera_tracker->getNewTilt();
	//} else {
	  new_tilt = forward_tilt;
	  //}
      }


    } else if ( tracking_mode == bbClients::CameraControl_Client::TRACKING_WORLD ) {
      // track a world point
      //cout << "Tracking POINT (" << m_pCameraControlServer->GetTrackWorldPoint( 0 )
      //     << "," << m_pCameraControlServer->GetTrackWorldPoint( 1 ) << ")." << endl;
      camera_tracker->setMode( CameraTracker::MODE_WORLD );
      camera_tracker->setWorldPoint( m_pCameraControlServer->GetTrackWorldPoint( 0 ),
				     m_pCameraControlServer->GetTrackWorldPoint( 1 ) );
      camera_tracker->setRobotPosition( m_pLocalizeMasterClient->GetCurrentX(),
					m_pLocalizeMasterClient->GetCurrentY(),
					m_pLocalizeMasterClient->GetCurrentOri() );
      camera_tracker->calc();
      new_pan  = camera_tracker->getNewPan();
      //if (config->CameraTrackTilt) {
      //new_tilt = camera_tracker->getNewTilt();
      //} else {
	new_tilt = forward_tilt;
	//}
      
    } else if ( tracking_mode == bbClients::CameraControl_Client::TRACKING_FORWARD ) {
      new_pan  = forward_pan;
      new_tilt = forward_tilt;
      
    } else {
      cout << msg_prefix << cred << "Invalid tracking mode set!" << cnormal << endl;
    }

  }

    if ( (fabs(new_pan  - last_pan)  >= config->PanMinChange) ||
	 (fabs(new_tilt - last_tilt) >= config->TiltMinChange)) {

      if ( new_tilt > config->TiltMax ) {
	new_tilt = config->TiltMax;
      }

      // only set new pan/tilt if value change was big enough
      try {
	camctrl->set_pan_tilt_rad( new_pan, new_tilt );
      } catch (Exception &e) {
	cout << "Caught exception, 2" << endl;
	e.print_trace();
      }
      last_pan  = new_pan;
      last_tilt = new_tilt;
    }


  loop_running = false;

}


// ===================================================================================
// "Exit"

void
FirevisionGeegawBBClient::Exit()
{

  exit_running = true;

  cout << "FirevisionGeegawBBClient exiting" << endl;

  while ( loop_running ) {
    cout << "w" << flush;
    usleep( 0 );
  }
  cout << endl;

  m_pVisObsServer->SetNumberOfObstacles( 0 );
  m_pVisObsServer->UpdateBB();
  BBPing();
  BBOperate();

  pipeline->finalize();

  delete m_pVisObsServer;
  // delete m_pLocalizeMasterClient;
  delete m_pCameraControlServer;
  delete pipeline;

  exit_running = false;

}

/// @endcond RCSoftX
