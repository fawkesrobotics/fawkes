
/***************************************************************************
 *  bbcannikin.cpp - implementation of bbclient for cannikin
 *
 *  Generated: Tue Apr 17 09:33:40 2007
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

#include <blackboard/clientappl.h>
#include <interfaces/cannikin_client.h>

#include <unistd.h>
#include <utils/system/getkey.h>
#include <utils/math/angle.h>

using namespace std;

class CannikinTestBBClient : public bb::ClientAppl
{
 public:
  CannikinTestBBClient(int argc, char* argv[]);
  virtual ~CannikinTestBBClient();

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  bbClients::Cannikin_Client   *m_pCannikin;
  bool display;
  bool display_once;

};


// ==============================================================================
// Constructor
CannikinTestBBClient::CannikinTestBBClient(int argc, char* argv[]) :
  bb::ClientAppl(argc, argv) 
{
  display = false;
  display_once = false;
  BBOperate();
}



// ==============================================================================
// Destructor
CannikinTestBBClient::~CannikinTestBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void CannikinTestBBClient::Init ()
{

  // get hostname
  std::string hostname = GetBBNames()[0];

  // initialize localize camera control client
  m_pCannikin = new bbClients::Cannikin_Client( hostname );
  BBRegisterObj( m_pCannikin );

  BBOperate();
  m_pCannikin->Update();
  BBOperate();

  // Call us every 50ms
  SetTime(50);

  cout << "BB client application for testing Cannikin"
       << " of Firevision." << endl << endl
       << "Valid keys:" << endl
       << "y         Select yellow cup" << endl
       << "g         Select green cup" << endl
       << "b         Select blue cup" << endl
       << "r         Select red cup" << endl
       << "o         Select orange cup" << endl
       << "d         display information" << endl
       << "n         display info exactly once" << endl
       << "m         Toggle mode" << endl
       << "x         eXit" << endl;

}


// "Loop"
void
CannikinTestBBClient::Loop(int Count)
{ 

  m_pCannikin->Update();
  BBOperate();

  char key = getkey();

  if ( display || display_once ) {
    cout << "Cam (X, Y, Z):    (" << m_pCannikin->GetCamX() << ","
	 << m_pCannikin->GetCamY() << "," << m_pCannikin->GetCamZ() << ")" << endl
	 << "RobRelPos (X,Y,Z):  (" << m_pCannikin->GetX() << ","
	 << m_pCannikin->GetY() << "," << m_pCannikin->GetZ() << ")" << endl
	 << "Bearing: " << rad2deg(m_pCannikin->GetBearing()) << "°" << endl
	 << "Slope: " << rad2deg(m_pCannikin->GetSlope()) << "°" << endl
	 << "Cup visible:  " << m_pCannikin->IsVisible() << endl
	 << "Cup Color:    ";
    if (m_pCannikin->GetCurrentCupColor() ==  bbClients::Cannikin_Client::CC_YELLOW) {
      cout << "yellow";
    } else if (m_pCannikin->GetCurrentCupColor() ==  bbClients::Cannikin_Client::CC_BLUE) {
      cout << "blue";
    } else if (m_pCannikin->GetCurrentCupColor() ==  bbClients::Cannikin_Client::CC_GREEN) {
      cout << "green";
    } else if (m_pCannikin->GetCurrentCupColor() ==  bbClients::Cannikin_Client::CC_RED) {
      cout << "red";
    } else if (m_pCannikin->GetCurrentCupColor() ==  bbClients::Cannikin_Client::CC_ORANGE) {
      cout << "orange";
    } else {
      cout << "I have no fucking clue";
    }
    cout << endl
	 << "Current mode: ";
    if (m_pCannikin->GetCurrentMode() ==  bbClients::Cannikin_Client::MODE_DETECTION ) {
      cout << "DETECTION";
    } else if (m_pCannikin->GetCurrentMode() ==  bbClients::Cannikin_Client::MODE_DETERMINE_CUP_COLOR ) {
      cout << "DETERMINE_CUP_COLOR";
    } else {
      cout << "I have no fucking clue";
    }
    cout << endl;

    if ( m_pCannikin->GetCurrentMode() == bbClients::Cannikin_Client::MODE_DETERMINE_CUP_COLOR ) {
      cout << "Cup color detected: " << ((m_pCannikin->GetDeterminationDone()) ? "Yes" : "No") << endl;
      cout << "Determined cup color: ";
      if (m_pCannikin->GetDeterminedCupColor() ==  bbClients::Cannikin_Client::CC_YELLOW) {
	cout << "yellow";
      } else if (m_pCannikin->GetDeterminedCupColor() ==  bbClients::Cannikin_Client::CC_BLUE) {
	cout << "blue";
      } else if (m_pCannikin->GetDeterminedCupColor() ==  bbClients::Cannikin_Client::CC_GREEN) {
	cout << "green";
      } else if (m_pCannikin->GetDeterminedCupColor() ==  bbClients::Cannikin_Client::CC_RED) {
      cout << "red";
      } else if (m_pCannikin->GetDeterminedCupColor() ==  bbClients::Cannikin_Client::CC_ORANGE) {
	cout << "orange";
      } else {
	cout << "I have no fucking clue";
      }
      cout << endl;
    }
    display_once = false;
  }

  switch (key) {
  case 'x':
  case 'X':
    bb::ApplExit::ApplThrow();
    break;
  case 'y':
    cout << "Setting desired cup to 'YELLOW'" << endl;
    m_pCannikin->SetCupColor( bbClients::Cannikin_Client::CC_YELLOW );
    m_pCannikin->UpdateBB();
    break;
  case 'g':
    cout << "Setting desired cup to 'GREEN'" << endl;
    m_pCannikin->SetCupColor( bbClients::Cannikin_Client::CC_GREEN );
    m_pCannikin->UpdateBB();
    break;
  case 'b':
    cout << "Setting desired cup to 'BLUE'" << endl;
    m_pCannikin->SetCupColor( bbClients::Cannikin_Client::CC_BLUE );
    m_pCannikin->UpdateBB();
    break;
  case 'r':
    cout << "Setting desired cup to 'RED'" << endl;
    m_pCannikin->SetCupColor( bbClients::Cannikin_Client::CC_RED );
    m_pCannikin->UpdateBB();
    break;
  case 'o':
    cout << "Setting desired cup to 'ORANGE'" << endl;
    m_pCannikin->SetCupColor( bbClients::Cannikin_Client::CC_ORANGE );
    m_pCannikin->UpdateBB();
    break;
  case 'd':
    display = ! display;
    break;
  case 'm':
    if (m_pCannikin->GetCurrentMode() == bbClients::Cannikin_Client::MODE_DETECTION ) {
      m_pCannikin->SetMode(bbClients::Cannikin_Client::MODE_DETERMINE_CUP_COLOR);
      m_pCannikin->UpdateBB();
    } else {
      m_pCannikin->SetMode(bbClients::Cannikin_Client::MODE_DETECTION);
      m_pCannikin->UpdateBB();
    }
  case 'n':
    display_once = true;
    break;
  }

  BBOperate();
}


// ===================================================================================
// "Exit"

void
CannikinTestBBClient::Exit()
{
  cout << "CannikinTestBBClient exiting" << endl;
  delete m_pCannikin;
}


int
main(int argc, char **argv)
{
  CannikinTestBBClient *bbclient = new CannikinTestBBClient(argc, argv);
  bbclient->Main();
  delete bbclient;
}
