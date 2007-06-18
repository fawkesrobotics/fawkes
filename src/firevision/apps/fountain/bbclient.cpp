
/***************************************************************************
 *  bbclient.cpp - implementation of bbclient for Fountain, only for sync of
 *                 aliveness
 *
 *  Generated: Wed Feb 08 12:47:29 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <apps/fountain/bbclient.h>
#include <utils/system/console_colors.h>

using namespace std;

// ==============================================================================
// Constructor
FirevisionFountainBBClient::FirevisionFountainBBClient(int argc, char* argv[]) :
  bb::ClientAppl(argc, argv) 
{
  BBOperate();
  msg_prefix = cblue + "FirevisionFountainBBClient: " + cnormal;
  cout << msg_prefix << "My name is " << argv[0] << endl;

  alive_clients.clear();
}


// ==============================================================================
// Destructor
FirevisionFountainBBClient::~FirevisionFountainBBClient()
{
  Exit();
}



// =============================================================================
// "Init"
void FirevisionFountainBBClient::Init ()
{
  hostname = GetEnvVar("HOSTNAME", "");
  BBSetNr( hostname );
  BBOperate();

  alive_server = new bbClients::Alive_Server("firevision_fountain", hostname);
  BBRegisterObj( alive_server );
  BBOperate();

  cout << msg_prefix << "Connected to BB on " << hostname << endl;

  alive_server->PingAlive();
  alive_server->UpdateBB();
  BBOperate();
}


// "Loop"
void
FirevisionFountainBBClient::Loop(int Count)
{
  alive.clear();
  map< string, bbClients::Alive_Client * >::iterator acit;
  for (acit = alive_clients.begin(); acit != alive_clients.end(); ++acit) {
    (*acit).second->Update();
  }
  alive_server->PingAlive();
  alive_server->UpdateBB();
  BBOperate();
  for (acit = alive_clients.begin(); acit != alive_clients.end(); ++acit) {
    alive[(*acit).first] = (*acit).second->GetIsAlive(3);
  }
}


// ===================================================================================
// "Exit"
void
FirevisionFountainBBClient::Exit()
{
  map< string, bbClients::Alive_Client * >::iterator acit;
  for (acit = alive_clients.begin(); acit != alive_clients.end(); ++acit) {
    delete (*acit).second;
  }
  alive_clients.clear();
}



void
FirevisionFountainBBClient::addAliveClient(string program)
{
  if (alive_clients.find( program ) == alive_clients.end()) {

    bbClients::Alive_Client *alive_client;

    alive_client = new bbClients::Alive_Client(program, hostname);
    BBRegisterObj( alive_client );
    BBOperate();

    alive_clients[program] = alive_client;
  }
}


bool
FirevisionFountainBBClient::isAlive(string program)
{
  if (alive_clients.find( program ) == alive_clients.end()) {
    addAliveClient(program);
    alive_clients[program]->Update();
    BBOperate();
    alive[program] = alive_clients[program]->GetIsAlive(-1);
  }

  // has to be done twice to yield correct results
  return alive[program];
}

/// @endcond
