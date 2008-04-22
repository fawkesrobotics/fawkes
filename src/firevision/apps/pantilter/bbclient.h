
/***************************************************************************
 *  bbclient.h - This header defines a bbclient for Suricate
 *
 *  Generated: Tue Apr 10 14:32:09 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: bbclient.h 949 2008-04-17 12:35:32Z stf $
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


#ifndef __FIREVISION_APPS_PANTILTER_BBCLIENT_H_
#define __FIREVISION_APPS_PANTILTER_BBCLIENT_H_

/// @cond RCSoftX

#include <blackboard/clientappl.h>
#include <utils/system/timestamp.h>
#include <string>

namespace bbClients {
  class CameraControl_Server;
}

class CameraControl;
class CameraTracker;

class FirevisionPanTilterBBClient : public bb::ClientAppl
{ 
 public:
  FirevisionPanTilterBBClient(int argc, char* argv[]);
  virtual ~FirevisionPanTilterBBClient();

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  std::string msg_prefix;

  CameraControl         *camctrl;
  CameraTracker         *camera_tracker;

  bbClients::CameraControl_Server    *m_pCameraControlServer;

}; 

/// @endcond

#endif
