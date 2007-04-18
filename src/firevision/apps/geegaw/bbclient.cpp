
/***************************************************************************
 *  bbclient.cpp - implementation of bbclient for geegaw
 *
 *  Generated: Tue Apr 10 14:31:49 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
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
#include <interfaces/vision_obstacles_server.h>

#include <unistd.h>

/*
include "front/camera_tracker.h"
include "front/config.h"
*/

using namespace std;

// ==============================================================================
// Constructor
FirevisionGeegawBBClient::FirevisionGeegawBBClient(int argc, char* argv[], ArgumentParser *argp) :
  bb::ClientAppl(argc, argv) 
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

  show_pose_info = argp->hasArgument("p");

  bearing_error = 0.05f;
  dist_error    = 0.5f;

  dist_total = bearing_total = 0.f;
  dist_num = bearing_num = 0;

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

  // initialize box position server
  m_pVisObsServer = new bbClients::VisionObstacles_Server( hostname );
  BBRegisterObj( m_pVisObsServer );


  /*
  // initialize localize master client
  m_pLocalizeMasterClient = new bbClients::Localize_Master_Client(hostname);
  BBRegisterObj( m_pLocalizeMasterClient );
  */

  // initialize camera control server
  m_pCameraControlServer = new bbClients::CameraControl_Server( hostname );
  BBRegisterObj( m_pCameraControlServer );

  m_pFrontAliveFakeServer = new bbClients::Alive_Server( "firevision_front", hostname );
  BBRegisterObj( m_pFrontAliveFakeServer );

  BBOperate();
  // m_pLocalizeMasterClient->Update();
  m_pCameraControlServer->Update();
  BBOperate();

  SetTime( 40 );

  pipeline = new GeegawPipeline(argp, config);
  pipeline->init();

  /*
  box_relative = pipeline->getRelativeBoxPosModel();
  box_global   = pipeline->getGlobalBoxPosModel();
  camctrl       = pipeline->getCameraControl();
  scanline_model = pipeline->getScanlineModel();
  */
}


// "Loop"
void
FirevisionGeegawBBClient::Loop(int Count)
{
  if ( exit_running || loop_running ) return;

  loop_running = true;

  BBOperate();
  
  m_pCameraControlServer->Update();
  BBOperate();

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
  if (pipeline->obstacles_found() ) {

    cout << "Found some" << endl;

  } else {
    cout << msg_prefix << cred << "Box is NOT visible" << cnormal << endl;

    m_pVisObsServer->SetNumberOfObstacles( 0 );

  }
  m_pVisObsServer->UpdateBB();

  m_pFrontAliveFakeServer->PingAlive();
  m_pFrontAliveFakeServer->UpdateBB();

  BBPing();
  BBOperate();

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
