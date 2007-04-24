
/***************************************************************************
 *  bbgeegaw.cpp - implementation of bbclient for geegaw
 *
 *  Generated: Tue Apr 17 09:33:40 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: bbgeegaw.cpp 106 2007-04-17 10:32:26Z tim $
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

#include <blackboard/clientappl.h>
#include <interfaces/vision_obstacles_client.h>

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

  BBOperate();
  m_pVisObsClient->Update();
  BBOperate();

  // Call us every 200ms
  SetTime(200);


}


// "Loop"
void
GeegawTestBBClient::Loop(int Count)
{ 

  m_pVisObsClient->Update();
  BBOperate();

  cout << "==========================================================================" << endl;
  for (int i = 0; i < m_pVisObsClient->GetNumberOfObstacles(); ++i) {
    cout << "Obstacle " << i << " at angle=" << m_pVisObsClient->GetPosRelAngle(i)
	 << "  dist=" << m_pVisObsClient->GetPosRelDist(i)
	 << "  width=" << m_pVisObsClient->GetPosWidth(i)
	 << "  height=" << m_pVisObsClient->GetPosHeight(i) << endl;
  }
}


// ===================================================================================
// "Exit"

void
GeegawTestBBClient::Exit()
{
  cout << "GeegawTestBBClient exiting" << endl;
  delete m_pVisObsClient;
}


int
main(int argc, char **argv)
{
  GeegawTestBBClient *bbclient = new GeegawTestBBClient(argc, argv);
  bbclient->Main();
  delete bbclient;
}

/// @endcond
