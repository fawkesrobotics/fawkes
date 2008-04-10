
/***************************************************************************
 *  bbclient.h - This header defines a bbclient for Fountain, only for alive
 *               checks
 *
 *  Generated: Wed Feb 08 12:45:52 2006
 *  Copyright  2005-2006 Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_APPS_FOUNTAIN_BBCLIENT_H_
#define __FIREVISION_APPS_FOUNTAIN_BBCLIENT_H_

/// @cond RCSoftX

#include <blackboard/clientappl.h>

class FirevisionFountainBBClient : public bb::ClientAppl
{ 
 public:
  FirevisionFountainBBClient(int argc, char* argv[]);
  virtual ~FirevisionFountainBBClient();

  virtual bool isAlive(std::string program);
  virtual void addAliveClient(std::string program);

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  std::string msg_prefix;
  std::string hostname;

  bbClients::Alive_Server *alive_server;
  std::map< std::string, bbClients::Alive_Client * >  alive_clients;
  std::map< std::string, bool >                       alive;

}; 

/// @endcond

#endif
