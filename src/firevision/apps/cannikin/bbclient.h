
/***************************************************************************
 *  bbclient.h - This header defines a bbclient for Suricate
 *
 *  Generated: Tue Apr 10 14:32:09 2007
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


#ifndef __FIREVISION_APPS_CANNIKIN_BBCLIENT_H_
#define __FIREVISION_APPS_CANNIKIN_BBCLIENT_H_

/// @cond RCSoftX

#include <blackboard/clientappl.h>
#include <utils/system/timestamp.h>

class RelativePositionModel;
class GlobalPositionModel;
class ScanlineModel;
class ArgumentParser;
class CannikinPipeline;
class CannikinConfig;
class CameraTracker;
class CameraControl;

namespace bbClients {
  // class Localize_Master_Client;
  class BallPos_Server;
  class CameraControl_Server;
  class Alive_Server;
  class Cannikin_Server;
}

class FirevisionCannikinBBClient : public bb::ClientAppl
{ 
 public:
  FirevisionCannikinBBClient(int argc, char* argv[], ArgumentParser *argp);
  virtual ~FirevisionCannikinBBClient();

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  void cup_not_visible();

  ArgumentParser    *argp;
  CannikinConfig    *config;
  CannikinPipeline  *pipeline;

  bool            show_pose_info;
  float           pose_avg_dt;
  unsigned int    pose_avg_num_samples;

  bool            loop_running;
  bool            exit_running;

  CameraControl         *camctrl;

  RelativePositionModel *box_relative;
  GlobalPositionModel   *box_global;
  ScanlineModel         *scanline_model;

  std::string msg_prefix;

  float new_pan;
  float new_tilt;
  float last_pan;
  float last_tilt;

  float bearing_error;
  float bearing_total;
  unsigned int bearing_num;
  float last_bearing;

  float dist_error;
  float dist_total;
  unsigned int dist_num;
  float last_dist;

  float forward_pan;
  float forward_tilt;

  float box_rel_x;
  float box_rel_y;
  float box_glob_x;
  float box_glob_y;
  float box_dist;
  float box_bearing;
  float box_slope;

  float rob_pos_x;
  float rob_pos_y;
  float rob_pos_x_old;
  float rob_pos_y_old;
  Timestamp   rob_pos_time;
  Timestamp   rob_pos_time_old;
  float time_difference;

  Timestamp   box_lost_time;
  bool             box_lost;

  Timestamp   now;

  int   visibility_history;

  CameraTracker         *camera_tracker;


  // bbClients::Localize_Master_Client *m_pLocalizeMasterClient;
  bbClients::BallPos_Server        *m_pBoxPosServer;
  bbClients::CameraControl_Server  *m_pCameraControlServer;
  bbClients::Alive_Server          *m_pFrontAliveFakeServer;
  bbClients::Cannikin_Server       *m_pCannikinServer;

}; 

/// @endcond

#endif
