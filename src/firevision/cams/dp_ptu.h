
/***************************************************************************
 *  dp_ptu.h - Controller for Directed Perception, Inc. Pan-Tilt Unit on B21
 *
 *  Created: Wed Nov 29 23:02:42 2006
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

#ifndef __FIREVISION_CAMS_DP_PTU_H_
#define __FIREVISION_CAMS_DP_PTU_H_

#include <cams/cameracontrol.h>

#define DPPTU_MAX_OBUFFER_SIZE  20
#define DPPTU_MAX_IBUFFER_SIZE 255

class CameraArgumentParser;

class DPPTUControl : public CameraControl
{

 public:

  DPPTUControl(const char *port);
  DPPTUControl(const CameraArgumentParser *cap);
  virtual ~DPPTUControl();

  // pan/tilt
  virtual bool supports_pan();
  virtual bool supports_tilt();
  virtual void set_pan(int pan);
  virtual void set_tilt(int tilt);
  virtual void set_pan_tilt(int pan, int tilt);
  virtual void set_pan_tilt_rad(float pan, float tilt);
  virtual int  pan();
  virtual int  tilt();
  virtual void start_get_pan_tilt();
  virtual void pan_tilt(int *pan, int *tilt);
  virtual void pan_tilt_rad(float *pan, float *tilt);
  virtual int  min_pan();
  virtual int  max_pan();
  virtual int  min_tilt();
  virtual int  max_tilt();
  virtual void reset_pan_tilt();
  virtual void set_pan_tilt_limit(int pan_left, int pan_right,
				  int tilt_up, int tilt_down);
  virtual void reset_pan_tilt_limit();

 private:
  void  open();
  void  send(const char *command, int value);
  void  send(const char *command);
  void  write(const char *buffer);
  bool  read(char *buffer, unsigned int buffer_size);
  bool  result_ok();
  bool  data_available();
  int   query_int(const char *query_command);
  int   pan_rad2ticks(float r);
  int   tilt_rad2ticks(float r);
  float pan_ticks2rad(int ticks);
  float tilt_ticks2rad(int ticks);


 private:
  char *tty_port;
  int   dev; // fd
  bool  opened;

  // commands
  static const char * DPPTU_PAN_ABSPOS;
  static const char * DPPTU_TILT_ABSPOS;
  static const char * DPPTU_PAN_RELPOS;
  static const char * DPPTU_TILT_RELPOS;
  static const char * DPPTU_PAN_RESOLUTION;
  static const char * DPPTU_TILT_RESOLUTION;
  static const char * DPPTU_PAN_MIN;
  static const char * DPPTU_PAN_MAX;
  static const char * DPPTU_TILT_MIN;
  static const char * DPPTU_TILT_MAX;
  static const char * DPPTU_LIMITENFORCE_QUERY;
  static const char * DPPTU_LIMITENFORCE_ENABLE;
  static const char * DPPTU_LIMITENFORCE_DISABLE;
  static const char * DPPTU_IMMEDIATE_EXECUTION;
  static const char * DPPTU_SLAVED_EXECUTION;
  static const char * DPPTU_AWAIT_COMPLETION;
  static const char * DPPTU_HALT_ALL;
  static const char * DPPTU_HALT_PAN;
  static const char * DPPTU_HALT_TILT;
  static const char * DPPTU_PAN_SPEED;
  static const char * DPPTU_TILT_SPEED;
  static const char * DPPTU_PAN_ACCEL;
  static const char * DPPTU_TILT_ACCEL;
  static const char * DPPTU_PAN_BASESPEED;
  static const char * DPPTU_TILT_BASESPEED;
  static const char * DPPTU_PAN_UPPER_SPEED_LIMIT;
  static const char * DPPTU_PAN_LOWER_SPEED_LIMIT;
  static const char * DPPTU_TILT_UPPER_SPEED_LIMIT;
  static const char * DPPTU_TILT_LOWER_SPEED_LIMIT;
  static const char * DPPTU_RESET;
  static const char * DPPTU_STORE;
  static const char * DPPTU_RESTORE;
  static const char * DPPTU_FACTORY_RESET;
  static const char * DPPTU_ECHO_QUERY;
  static const char * DPPTU_ECHO_ENABLE;
  static const char * DPPTU_ECHO_DISABLE;
  static const char * DPPTU_ASCII_VERBOSE;
  static const char * DPPTU_ASCII_TERSE;
  static const char * DPPTU_ASCII_QUERY;
  static const char * DPPTU_VERSION;

  char out_buffer[DPPTU_MAX_OBUFFER_SIZE];
  char in_buffer[DPPTU_MAX_IBUFFER_SIZE];

  int pan_resolution;
  int tilt_resolution;
  int pan_upper_limit;
  int pan_lower_limit;
  int tilt_lower_limit;
  int tilt_upper_limit;

  unsigned int max_wait_ms;
  unsigned int effect;

};

#endif
