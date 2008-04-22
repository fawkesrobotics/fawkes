
/***************************************************************************
 *  bbclient.cpp - implementation of bbclient for geegaw
 *
 *  Generated: Tue Apr 10 14:31:49 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: bbclient.cpp 970 2008-04-18 13:46:32Z stf $
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

#include <fvutils/camera/tracker.h>
#include <cams/sony_evid100p_control.h>
#include <utils/system/console_colors.h>
#include <core/exception.h>
#include <utils/math/angle.h>

#include <interfaces/camera_control_server.h>

#include <unistd.h>

using namespace std;

// ==============================================================================
// Constructor
FirevisionPanTilterBBClient::FirevisionPanTilterBBClient(int argc, char* argv[])
  : bb::ClientAppl(argc, argv) 
{
  BBOperate();
  msg_prefix = cblue + "FirevisionGeegawBBClient: " + cnormal;
}


// ==============================================================================
// Destructor
FirevisionPanTilterBBClient::~FirevisionPanTilterBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void FirevisionPanTilterBBClient::Init ()
{

  // get hostname
  std::string hostname = GetBBNames()[0];

  // initialize camera control server
  m_pCameraControlServer = new bbClients::CameraControl_Server( hostname );
  BBRegisterObj( m_pCameraControlServer );

  BBOperate();

  //SetTime( 40 );

  camctrl       = new SonyEviD100PControl("/dev/ttyS0");

  try {
    camctrl->set_pan_tilt_rad( 0, 0 );
  } catch (Exception &e) {
    cout << "Caught exception, 1" << endl;
    e.print_trace();
  }

  SetTime(100);

  /*
  camera_tracker = new CameraTracker( obj_relative,
                                      config->CameraHeight,
                                      config->CameraOrientation);
  camera_tracker->set_mode(CameraTracker::MODE_MODEL);
  */
}


// "Loop"
void
FirevisionPanTilterBBClient::Loop(int Count)
{ 
  //camctrl->process_control();

  m_pCameraControlServer->Update();
  BBOperate();

  static float old_pan = 20., old_tilt = 20.;

  if ( ( old_pan != m_pCameraControlServer->GetTargetPan()) || (old_tilt != m_pCameraControlServer->GetTargetTilt()) ) {
    cout << msg_prefix << "Setting pan/tilt to " << m_pCameraControlServer->GetTargetPan() << "/"
         << m_pCameraControlServer->GetTargetTilt() << endl;
    try {
      camctrl->set_pan_tilt_rad(m_pCameraControlServer->GetTargetPan(), m_pCameraControlServer->GetTargetTilt());
      old_pan = m_pCameraControlServer->GetTargetPan();
      old_tilt = m_pCameraControlServer->GetTargetTilt();
    } catch (Exception &e) {
      e.print_trace();
    }
  }
  //cout << msg_prefix << "Current target pan/tilt is " << m_pCameraControlServer->GetTargetPan() << "/" << m_pCameraControlServer->GetTargetTilt() << endl;


  /*
  float pan, tilt;
  camctrl->pan_tilt_rad(&pan, &tilt);
  cout << msg_prefix << "Current pan/tilt is " << pan << "/" << tilt << endl;

  m_pCameraControlServer->SetCurrentPan(  pan  );
  m_pCameraControlServer->SetCurrentTilt( tilt );
  m_pCameraControlServer->UpdateBB();
  BBOperate();
  */

  BBPing();

  /*
  static int count = 0;

  try {
    if ( count % 50  == 0 ) {
      camctrl->set_pan_tilt_rad(deg2rad(-100), deg2rad(-25));
    } else if ( count % 25 == 0 ) {
      camctrl->set_pan_tilt_rad(deg2rad(100), deg2rad(25));
    }
  } catch (Exception &e) {
    e.print_trace();
  }

  ++count;
*/
}


// ===================================================================================
// "Exit"

void
FirevisionPanTilterBBClient::Exit()
{

  cout << "FirevisionPanTilterBBClient exiting" << endl;

  delete m_pCameraControlServer;
}

/// @endcond RCSoftX
