
/***************************************************************************
 *  bbgeegaw.cpp - implementation of bbclient for geegaw
 *
 *  Generated: Tue Apr 17 09:33:40 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: bbgeegaw.cpp 106 2007-04-17 10:32:26Z tim $
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

#include <blackboard/clientappl.h>
#include <interfaces/vision_obstacles_client.h>
#include <interfaces/object_position_client.h>
#include <interfaces/geegaw_client.h>

#include <unistd.h>
#include <utils/system/getkey.h>

using namespace std;

class GeegawTestBBClient : public bb::ClientAppl
{
 public:
  GeegawTestBBClient(int argc, char* argv[]);
  virtual ~GeegawTestBBClient();

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  bbClients::VisionObstacles_Client   *m_pVisObsClient;
  bbClients::ObjectPosition_Client    *m_pObjPosClient;
  bbClients::Geegaw_Client   *m_pGeegawClient;

  int last_mode;
};


// ==============================================================================
// Constructor
GeegawTestBBClient::GeegawTestBBClient(int argc, char* argv[]) :
  bb::ClientAppl(argc, argv) 
{
  BBOperate();
}



// ==============================================================================
// Destructor
GeegawTestBBClient::~GeegawTestBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void GeegawTestBBClient::Init ()
{

  // get hostname
  std::string hostname = GetBBNames()[0];

  // initialize localize camera control client
  m_pVisObsClient = new bbClients::VisionObstacles_Client( hostname );
  BBRegisterObj( m_pVisObsClient );

  // initialize object position client
  m_pObjPosClient = new bbClients::ObjectPosition_Client( hostname );
  BBRegisterObj( m_pObjPosClient );

  // initialize localize camera control client
  m_pGeegawClient = new bbClients::Geegaw_Client( hostname );
  BBRegisterObj( m_pGeegawClient );

  BBOperate();
  m_pVisObsClient->Update();
  m_pObjPosClient->read();
  m_pGeegawClient->Update();
  BBOperate();

  last_mode = m_pGeegawClient->GetCurrentMode();

  // Call us every 200ms
  SetTime(200);


}


// "Loop"
void
GeegawTestBBClient::Loop(int Count)
{ 

  m_pVisObsClient->Update();
  m_pObjPosClient->read();
  m_pGeegawClient->Update();
  BBOperate();

  if ( m_pGeegawClient->GetCurrentMode() != last_mode ) {
    cout << "Mode switched from " << last_mode << " to " << m_pGeegawClient->GetCurrentMode() << endl;
    last_mode = m_pGeegawClient->GetCurrentMode();
  }

  char key = getkey();

  switch (key) {
  case 'o':
    cout << "Requesting switch to OBSTACLES mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_OBSTACLES);
    m_pGeegawClient->UpdateBB();
    BBOperate();
    break;
  case 'l':
    cout << "Requesting switch to LOSTNFOUND mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_LOSTNFOUND);
    m_pGeegawClient->UpdateBB();
    BBOperate();
    break;
  case 'i':
    cout << "Requesting switch to SIFT mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_SIFT);
    m_pGeegawClient->SetObjectimage("../res/opx/alogo.png");
    m_pGeegawClient->UpdateBB();
    BBOperate();
    break;
  case 'u':
    cout << "Requesting switch to SURF mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_SURF);
    m_pGeegawClient->SetObjectimage("../res/opx/objects/");
    m_pGeegawClient->UpdateBB();
    BBOperate();
    //    cout<<"The new mode is : " << m_pGeegawClient->GetCurrentMode() << endl; 
    break;
  case 'p':
    cout << "Requesting switch to SIFTPP mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_SIFTPP);
    m_pGeegawClient->SetObjectimage("../res/opx/alogo.png");
    m_pGeegawClient->UpdateBB();
    BBOperate();
    break;
  case 'a':
    cout << "Requesting switch to ADD_OBJECT mode" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_ADD_OBJECT);
    m_pGeegawClient->UpdateBB();
    BBOperate();
    break;
  case 'r':
    cout << "Requesting COLORMAP_RESET" << endl;
    m_pGeegawClient->SetMode(bbClients::Geegaw_Client::MODE_RESET_COLORMAP);
    m_pGeegawClient->UpdateBB();
    BBOperate();
  }

  //  cout<<"The new mode is : " << m_pGeegawClient->GetCurrentMode() << endl; 
  

  if ( m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_OBSTACLES ) {
    cout << "==========================================================================" << endl;
    for (int i = 0; i < m_pVisObsClient->GetNumberOfObstacles(); ++i) {
      cout << "Obstacle " << i << " at angle=" << m_pVisObsClient->GetPosRelAngle(i)
	   << "  dist=" << m_pVisObsClient->GetPosRelDist(i)
	   << "  width=" << m_pVisObsClient->GetPosWidth(i)
	   << "  height=" << m_pVisObsClient->GetPosHeight(i) << endl;
    }
  } else if (m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_ADD_OBJECT ) {
    if (m_pGeegawClient->ChangedObjectAddStatus()) {
      int status = m_pGeegawClient->GetObjectAddStatus();
      cout << "Add Object status changed to: ";
      if (status == bbClients::Geegaw_Client::ADD_STATUS_NOTRUNNING) {
	cout << "NOTRUNNING";
      } else if ( status == bbClients::Geegaw_Client::ADD_STATUS_INPROGRESS) {
	cout << "INPROGRESS";
      } else if ( status == bbClients::Geegaw_Client::ADD_STATUS_SUCCESS) {
	cout << "SUCCESS";
      } else if ( status == bbClients::Geegaw_Client::ADD_STATUS_FAILURE) {
	cout << "FAILURE";
      } else {
	cout << "UNKNOWN";
      }
      cout << endl;
    }
  } else if (m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_LOSTNFOUND ) {
    cout << "lostnfound mode" << endl;
  } else if (m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_SIFT ) {
    cout << "SIFT mode" << endl;
    if ( m_pObjPosClient->visible() ) {
      cout << "object visible" << endl;
    } 
    else {
      cout << "object NOT visible" << endl;
    }  
  } else if (m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_SURF ) {
    cout << "SURF mode" << endl;
  } else if (m_pGeegawClient->GetCurrentMode() == bbClients::Geegaw_Client::MODE_SIFTPP ) {
    cout << "SIFTPP mode" << endl;
  }
}


// ===================================================================================
// "Exit"

void
GeegawTestBBClient::Exit()
{
  cout << "GeegawTestBBClient exiting" << endl;
  delete m_pVisObsClient;
  delete m_pObjPosClient;
}


int
main(int argc, char **argv)
{
  GeegawTestBBClient *bbclient = new GeegawTestBBClient(argc, argv);
  bbclient->Main();
  delete bbclient;
}

/// @endcond
