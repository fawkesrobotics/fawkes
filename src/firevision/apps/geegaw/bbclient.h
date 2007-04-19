
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


#ifndef __FIREVISION_APPS_GEEGAW_BBCLIENT_H_
#define __FIREVISION_APPS_GEEGAW_BBCLIENT_H_

/// @cond RCSoftX

#include <blackboard/clientappl.h>
#include <utils/system/timestamp.h>

class RelativePositionModel;
class GlobalPositionModel;
class ScanlineModel;
class ArgumentParser;
class GeegawPipeline;
class GeegawConfig;
class CameraControl;
class CameraTracker;

namespace bbClients {
  class Localize_Master_Client;
  class CameraControl_Server;
  class Alive_Server;
  class VisionObstacles_Server;
  class BallPos_Server;
}

class FirevisionGeegawBBClient : public bb::ClientAppl
{ 
 public:
  FirevisionGeegawBBClient(int argc, char* argv[], ArgumentParser *argp);
  virtual ~FirevisionGeegawBBClient();

  virtual void Init();
  virtual void Loop(int Count);
  virtual void Exit();

 private:

  bool object_mode;

  ArgumentParser  *argp;
  GeegawConfig    *config;
  GeegawPipeline  *pipeline;

  bool            show_pose_info;
  float           pose_avg_dt;
  unsigned int    pose_avg_num_samples;

  bool            loop_running;
  bool            exit_running;

  CameraControl         *camctrl;
  CameraTracker         *camera_tracker;

  RelativePositionModel *obj_relative;
  ScanlineModel         *scanline_model;

  std::string msg_prefix;

  int   tracking_mode;

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

  float rob_pos_x;
  float rob_pos_y;
  float rob_pos_x_old;
  float rob_pos_y_old;
  Timestamp   rob_pos_time;
  Timestamp   rob_pos_time_old;
  float time_difference;

  Timestamp   box_lost_time;
  bool             box_lost;

  int              visibility_history;

  Timestamp   now;

  bbClients::VisionObstacles_Server  *m_pVisObsServer;
  bbClients::CameraControl_Server    *m_pCameraControlServer;
  bbClients::Alive_Server            *m_pFrontAliveFakeServer;
  bbClients::BallPos_Server          *m_pObjPosServer;
  bbClients::Localize_Master_Client  *m_pLocalizeMasterClient;

}; 

/// @endcond

#endif
