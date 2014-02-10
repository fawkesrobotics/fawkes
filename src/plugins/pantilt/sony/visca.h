
/***************************************************************************
 *  visca.h - Class for accessing visca cams
 *
 *  Created: Wed Jun 08 12:06:15 2005 (FireVision)
 *  Copyright  2005-2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_PANTILT_SONY_VISCA_H_
#define __PLUGINS_PANTILT_SONY_VISCA_H_

#include <core/exception.h>

#ifdef TIMETRACKER_VISCA
#  warning Visca time tracker enabled
#  include <utils/timetracker.h>
#  include <fstream>
#endif

#include <cstddef>

class ViscaException : public fawkes::Exception
{
 public:
  ViscaException(const char *msg);
  ViscaException(const char *msg, const int _errno);
};

class ViscaInquiryRunningException : public ViscaException
{
 public:
  ViscaInquiryRunningException();
};


class Visca {

 public:
  static const unsigned int VISCA_WHITEBLANCE_AUTO;
  static const unsigned int VISCA_WHITEBALANCE_INDOOR;
  static const unsigned int VISCA_WHITEBALANCE_OUTDOOR;
  static const unsigned int VISCA_WHITEBALANCE_ONE_PUSH;
  static const unsigned int VISCA_WHITEBALANCE_ATW;
  static const unsigned int VISCA_WHITEBALANCE_MANUAL;

  static const unsigned int  NONBLOCKING_PANTILT;
  static const unsigned int  NONBLOCKING_ZOOM;
  static const unsigned int  NONBLOCKING_NUM;

  static const unsigned int  MAX_PAN_SPEED;
  static const unsigned int  MAX_TILT_SPEED;

  /// Zoom value: wide
  static const unsigned int    VISCA_ZOOM_VALUE_WIDE            = 0x0000;
  /// Zoom value: 1x
  static const unsigned int    VISCA_ZOOM_VALUE_1X              = 0x0E6D;
  /// Zoom value: 2x
  static const unsigned int    VISCA_ZOOM_VALUE_2X              = 0x188E;
  /// Zoom value: 3x
  static const unsigned int    VISCA_ZOOM_VALUE_3X              = 0x2507;
  /// Zoom value: 4x
  static const unsigned int    VISCA_ZOOM_VALUE_4X              = 0x2B82;
  /// Zoom value: 5x
  static const unsigned int    VISCA_ZOOM_VALUE_5X              = 0x3130;
  /// Zoom value: 6x
  static const unsigned int    VISCA_ZOOM_VALUE_6X              = 0x352E;
  /// Zoom value: 7x
  static const unsigned int    VISCA_ZOOM_VALUE_7X              = 0x385D;
  /// Zoom value: 8x
  static const unsigned int    VISCA_ZOOM_VALUE_8X              = 0x3B48;
  /// Zoom value: 9x
  static const unsigned int    VISCA_ZOOM_VALUE_9X              = 0x3E01;
  /// Zoom value: 10x
  static const unsigned int    VISCA_ZOOM_VALUE_10X             = 0x4000;
  /// Zoom value: 20x
  static const unsigned int    VISCA_ZOOM_VALUE_DIG_20X         = 0x5000;
  /// Zoom value: 30x
  static const unsigned int    VISCA_ZOOM_VALUE_DIG_30X         = 0x6000;
  /// Zoom value: 40x
  static const unsigned int    VISCA_ZOOM_VALUE_DIG_40X         = 0x7000;

  Visca(const char *device_file, unsigned int def_timeout_ms = 10,
	bool blocking = true);
  virtual ~Visca();

  void         open();
  void         close();

  // basic communication
  void         set_address();
  void         clear();

  // power
  void         set_power(bool powered);
  bool         is_powered();

  // low level
  void         send();
  void         recv(unsigned int timeout_ms = 0xFFFFFFFF);
  void         recv_ack(unsigned int *socket = NULL);
  void         send_with_reply();
  void         send_nonblocking(unsigned int *socket = NULL);
  void         cancel_command(unsigned int socket);
  bool         data_available();
  void         process();

  // pan tilt stuff
  void         reset_pan_tilt();
  /** Query for pan/tilt but do not wait until finished
   * This will send an inquire to the camera that asks for pan/tilt values but
   * it does not wait for the data! A later call to getPanTilt will then block and
   * wait until the results arrive.
   * Not that you can _not_ run another inquire (get*) method until this call has
   * finished! You will get VISCA_E_INQRUNNING as error message.
   */
  void         start_get_pan_tilt();
  void         set_pan_tilt(int pan, int tilt);
  void         get_pan_tilt(int &pan, int &tilt);
  void         set_pan_tilt_limit(int pan_left, int pan_right, int tilt_up, int tilt_down);
  void         reset_pan_tilt_limit();
  void         set_pan_tilt_speed(unsigned char pan_speed, unsigned char tilt_speed);
  void         get_pan_tilt_speed(unsigned char &pan_speed, unsigned char &tilt_speed);

  bool         is_nonblocking_finished(unsigned int item) const;

  // zoom
  void         reset_zoom();
  void         set_zoom(unsigned int zoom);
  void         get_zoom(unsigned int &zoom);
  void         set_zoom_speed_tele(unsigned int speed);
  void         set_zoom_speed_wide(unsigned int speed);
  void         set_zoom_digital_enabled(bool enabled);

  // effects, just to play with...
  void         reset_effect();
  void         apply_effect(unsigned char effect);
  void         apply_effect_pastel();
  void         apply_effect_neg_art();
  void         apply_effect_sepia();
  void         apply_effect_bnw();
  void         apply_effect_solarize();
  void         apply_effect_mosaic();
  void         apply_effect_slim();
  void         apply_effect_stretch();

  unsigned int get_white_balance_mode();

  bool         get_mirror();
  void         set_mirror(bool mirror);

 private:
  // possible running inquires
  static const unsigned int VISCA_RUNINQ_NONE                   = 0;
  static const unsigned int VISCA_RUNINQ_PANTILT                = 1;

  // Cameras
  static const unsigned char VISCA_BUS_0                        = 0;
  static const unsigned char VISCA_BUS_1                        = 1;
  static const unsigned char VISCA_BUS_2                        = 2;
  static const unsigned char VISCA_BUS_3                        = 3;
  static const unsigned char VISCA_BUS_4                        = 4;
  static const unsigned char VISCA_BUS_5                        = 5;
  static const unsigned char VISCA_BUS_6                        = 6;
  static const unsigned char VISCA_BUS_7                        = 7;
  static const unsigned char VISCA_BUS_BROADCAST                = 8;

  // basic formatting
  static const unsigned char VISCA_COMMAND                      = 0x01;
  static const unsigned char VISCA_CANCEL                       = 0x20;
  static const unsigned char VISCA_INQUIRY                      = 0x09;
  static const unsigned char VISCA_TERMINATOR                   = 0xFF;

  // response types
  static const unsigned char VISCA_RESPONSE_CLEAR               = 0x40;
  static const unsigned char VISCA_RESPONSE_ADDRESS             = 0x30;
  static const unsigned char VISCA_RESPONSE_ACK                 = 0x40;
  static const unsigned char VISCA_RESPONSE_COMPLETED           = 0x50;
  static const unsigned char VISCA_RESPONSE_ERROR               = 0x60;

  // errors
  static const unsigned char VISCA_ERROR_LENGTH                 = 0x01;
  static const unsigned char VISCA_ERROR_SYNTAX                 = 0x02;
  static const unsigned char VISCA_ERROR_BUFFERFULL             = 0x03;
  static const unsigned char VISCA_ERROR_CANCELLED              = 0x04;
  static const unsigned char VISCA_ERROR_NOSOCKET               = 0x05;
  static const unsigned char VISCA_ERROR_NOTEXECABLE            = 0x41;


  // categories
  static const unsigned char VISCA_CATEGORY_INTERFACE           = 0x00;
  static const unsigned char VISCA_CATEGORY_CAMERA1             = 0x04;
  static const unsigned char VISCA_CATEGORY_PAN_TILTER          = 0x06;
  static const unsigned char VISCA_CATEGORY_CAMERA2             = 0x07;

  static const unsigned char VISCA_POWER                        = 0x00;
  static const unsigned char   VISCA_POWER_ON                   = 0x02;
  static const unsigned char   VISCA_POWER_OFF                  = 0x03;
  static const unsigned char VISCA_DEVICE_INFO                  = 0x02;
  static const unsigned char VISCA_KEYLOCK                      = 0x17;
  static const unsigned char VISCA_ID                           = 0x22;
  static const unsigned char VISCA_ZOOM                         = 0x07;
  static const unsigned char   VISCA_ZOOM_STOP                  = 0x00;
  static const unsigned char   VISCA_ZOOM_TELE                  = 0x02;
  static const unsigned char   VISCA_ZOOM_WIDE                  = 0x03;
  static const unsigned char   VISCA_ZOOM_TELE_SPEED            = 0x20;
  static const unsigned char   VISCA_ZOOM_WIDE_SPEED            = 0x30;
  static const unsigned char VISCA_ZOOM_VALUE                   = 0x47;
  static const unsigned char VISCA_ZOOM_FOCUS_VALUE             = 0x47;
  static const unsigned char VISCA_DZOOM                        = 0x06;
  static const unsigned char   VISCA_DZOOM_ON                   = 0x02;
  static const unsigned char   VISCA_DZOOM_OFF                  = 0x03;
  static const unsigned char VISCA_FOCUS                        = 0x08;
  static const unsigned char   VISCA_FOCUS_STOP                 = 0x00;
  static const unsigned char   VISCA_FOCUS_FAR                  = 0x02;
  static const unsigned char   VISCA_FOCUS_NEAR                 = 0x03;
  static const unsigned char   VISCA_FOCUS_FAR_SPEED            = 0x20;
  static const unsigned char   VISCA_FOCUS_NEAR_SPEED           = 0x30;
  static const unsigned char VISCA_FOCUS_VALUE                  = 0x48;
  static const unsigned char VISCA_FOCUS_AUTO                   = 0x38;
  static const unsigned char   VISCA_FOCUS_AUTO_MAN             = 0x10;
  static const unsigned char VISCA_FOCUS_ONE_PUSH               = 0x18;
  static const unsigned char   VISCA_FOCUS_ONE_PUSH_TRIG        = 0x01;
  static const unsigned char   VISCA_FOCUS_ONE_PUSH_INF         = 0x02;
  static const unsigned char VISCA_FOCUS_AUTO_SENSE             = 0x58;
  static const unsigned char   VISCA_FOCUS_AUTO_SENSE_HIGH      = 0x02;
  static const unsigned char   VISCA_FOCUS_AUTO_SENSE_LOW       = 0x03;
  static const unsigned char VISCA_FOCUS_NEAR_LIMIT             = 0x28;
  static const unsigned char VISCA_WB                           = 0x35;
  static const unsigned char   VISCA_WB_AUTO                    = 0x00;
  static const unsigned char   VISCA_WB_INDOOR                  = 0x01;
  static const unsigned char   VISCA_WB_OUTDOOR                 = 0x02;
  static const unsigned char   VISCA_WB_ONE_PUSH                = 0x03;
  static const unsigned char   VISCA_WB_ATW                     = 0x04;
  static const unsigned char   VISCA_WB_MANUAL                  = 0x05;
  static const unsigned char   VISCA_WB_ONE_PUSH_TRIG           = 0x05;
  static const unsigned char VISCA_RGAIN                        = 0x03;
  static const unsigned char VISCA_RGAIN_VALUE                  = 0x43;
  static const unsigned char VISCA_BGAIN                        = 0x04;
  static const unsigned char VISCA_BGAIN_VALUE                  = 0x44;
  static const unsigned char VISCA_AUTO_EXP                     = 0x39;
  static const unsigned char   VISCA_AUTO_EXP_FULL_AUTO         = 0x00;
  static const unsigned char   VISCA_AUTO_EXP_MANUAL            = 0x03;
  static const unsigned char   VISCA_AUTO_EXP_SHUTTER_PRIORITY  = 0x0A;
  static const unsigned char   VISCA_AUTO_EXP_IRIS_PRIORITY     = 0x0B;
  static const unsigned char   VISCA_AUTO_EXP_GAIN_PRIORITY     = 0x0C;
  static const unsigned char   VISCA_AUTO_EXP_BRIGHT            = 0x0D;
  static const unsigned char   VISCA_AUTO_EXP_SHUTTER_AUTO      = 0x1A;
  static const unsigned char   VISCA_AUTO_EXP_IRIS_AUTO         = 0x1B;
  static const unsigned char   VISCA_AUTO_EXP_GAIN_AUTO         = 0x1C;
  static const unsigned char VISCA_SLOW_SHUTTER                 = 0x5A;
  static const unsigned char   VISCA_SLOW_SHUTTER_AUTO          = 0x02;
  static const unsigned char   VISCA_SLOW_SHUTTER_MANUAL        = 0x03;
  static const unsigned char VISCA_SHUTTER                      = 0x0A;
  static const unsigned char VISCA_SHUTTER_VALUE                = 0x4A;
  static const unsigned char VISCA_IRIS                         = 0x0B;
  static const unsigned char VISCA_IRIS_VALUE                   = 0x4B;
  static const unsigned char VISCA_GAIN                         = 0x0C;
  static const unsigned char VISCA_GAIN_VALUE                   = 0x4C;
  static const unsigned char VISCA_BRIGHT                       = 0x0D;
  static const unsigned char VISCA_BRIGHT_VALUE                 = 0x4D;
  static const unsigned char VISCA_EXP_COMP                     = 0x0E;
  static const unsigned char VISCA_EXP_COMP_POWER               = 0x3E;
  static const unsigned char VISCA_EXP_COMP_VALUE               = 0x4E;
  static const unsigned char VISCA_BACKLIGHT_COMP               = 0x33;
  static const unsigned char VISCA_APERTURE                     = 0x02;
  static const unsigned char VISCA_APERTURE_VALUE               = 0x42;
  static const unsigned char VISCA_ZERO_LUX                     = 0x01;
  static const unsigned char VISCA_IR_LED                       = 0x31;
  static const unsigned char VISCA_WIDE_MODE                    = 0x60;
  static const unsigned char   VISCA_WIDE_MODE_OFF              = 0x00;
  static const unsigned char   VISCA_WIDE_MODE_CINEMA           = 0x01;
  static const unsigned char   VISCA_WIDE_MODE_16_9             = 0x02;
  static const unsigned char VISCA_MIRROR                       = 0x61;
  static const unsigned char   VISCA_MIRROR_ON                  = 0x02;
  static const unsigned char   VISCA_MIRROR_OFF                 = 0x03;
  static const unsigned char VISCA_FREEZE                       = 0x62;
  static const unsigned char VISCA_PICTURE_EFFECT               = 0x63;
  static const unsigned char   VISCA_PICTURE_EFFECT_OFF         = 0x00;
  static const unsigned char   VISCA_PICTURE_EFFECT_PASTEL      = 0x01;
  static const unsigned char   VISCA_PICTURE_EFFECT_NEGATIVE    = 0x02;
  static const unsigned char   VISCA_PICTURE_EFFECT_SEPIA       = 0x03;
  static const unsigned char   VISCA_PICTURE_EFFECT_BW          = 0x04;
  static const unsigned char   VISCA_PICTURE_EFFECT_SOLARIZE    = 0x05;
  static const unsigned char   VISCA_PICTURE_EFFECT_MOSAIC      = 0x06;
  static const unsigned char   VISCA_PICTURE_EFFECT_SLIM        = 0x07;
  static const unsigned char   VISCA_PICTURE_EFFECT_STRETCH     = 0x08;
  static const unsigned char VISCA_DIGITAL_EFFECT               = 0x64;
  static const unsigned char   VISCA_DIGITAL_EFFECT_OFF         = 0x00;
  static const unsigned char   VISCA_DIGITAL_EFFECT_STILL       = 0x01;
  static const unsigned char   VISCA_DIGITAL_EFFECT_FLASH       = 0x02;
  static const unsigned char   VISCA_DIGITAL_EFFECT_LUMI        = 0x03;
  static const unsigned char   VISCA_DIGITAL_EFFECT_TRAIL       = 0x04;
  static const unsigned char VISCA_DIGITAL_EFFECT_LEVEL         = 0x65;
  static const unsigned char VISCA_MEMORY                       = 0x3F;
  static const unsigned char   VISCA_MEMORY_RESET               = 0x00;
  static const unsigned char   VISCA_MEMORY_SET                 = 0x01;
  static const unsigned char   VISCA_MEMORY_RECALL              = 0x02;
  static const unsigned char VISCA_DISPLAY                      = 0x15;
  static const unsigned char   VISCA_DISPLAY_TOGGLE             = 0x10;
  static const unsigned char VISCA_DATE_TIME_SET                = 0x70;
  static const unsigned char VISCA_DATE_DISPLAY                 = 0x71;
  static const unsigned char VISCA_TIME_DISPLAY                 = 0x72;
  static const unsigned char VISCA_TITLE_DISPLAY                = 0x74;
  static const unsigned char   VISCA_TITLE_DISPLAY_CLEAR        = 0x00;
  static const unsigned char VISCA_TITLE_SET                    = 0x73;
  static const unsigned char   VISCA_TITLE_SET_PARAMS           = 0x00;
  static const unsigned char   VISCA_TITLE_SET_PART1            = 0x01;
  static const unsigned char   VISCA_TITLE_SET_PART2            = 0x02;
  static const unsigned char VISCA_IRRECEIVE                    = 0x08;
  static const unsigned char   VISCA_IRRECEIVE_ON               = 0x02;
  static const unsigned char   VISCA_IRRECEIVE_OFF              = 0x03;
  static const unsigned char   VISCA_IRRECEIVE_ONOFF            = 0x10;
  static const unsigned char VISCA_PT_DRIVE                     = 0x01;
  static const unsigned char   VISCA_PT_DRIVE_HORIZ_LEFT        = 0x01;
  static const unsigned char   VISCA_PT_DRIVE_HORIZ_RIGHT       = 0x02;
  static const unsigned char   VISCA_PT_DRIVE_HORIZ_STOP        = 0x03;
  static const unsigned char   VISCA_PT_DRIVE_VERT_UP           = 0x01;
  static const unsigned char   VISCA_PT_DRIVE_VERT_DOWN         = 0x02;
  static const unsigned char   VISCA_PT_DRIVE_VERT_STOP         = 0x03;
  static const unsigned char VISCA_PT_ABSOLUTE_POSITION         = 0x02;
  static const unsigned char VISCA_PT_RELATIVE_POSITION         = 0x03;
  static const unsigned char VISCA_PT_HOME                      = 0x04;
  static const unsigned char VISCA_PT_RESET                     = 0x05;
  static const unsigned char VISCA_PT_LIMITSET                  = 0x07;
  static const unsigned char   VISCA_PT_LIMITSET_SET            = 0x00;
  static const unsigned char   VISCA_PT_LIMITSET_CLEAR          = 0x01;
  static const unsigned char     VISCA_PT_LIMITSET_SET_UR       = 0x01;
  static const unsigned char     VISCA_PT_LIMITSET_SET_DL       = 0x00;
  static const unsigned char VISCA_PT_DATASCREEN                = 0x06;
  static const unsigned char   VISCA_PT_DATASCREEN_ON           = 0x02;
  static const unsigned char   VISCA_PT_DATASCREEN_OFF          = 0x03;
  static const unsigned char   VISCA_PT_DATASCREEN_ONOFF        = 0x10;
  static const unsigned char VISCA_PT_VIDEOSYSTEM_INQ           = 0x23;
  static const unsigned char VISCA_PT_MODE_INQ                  = 0x10;
  static const unsigned char VISCA_PT_MAXSPEED_INQ              = 0x11;
  static const unsigned char VISCA_PT_POSITION_INQ              = 0x12;
  static const unsigned char VISCA_PT_DATASCREEN_INQ            = 0x06;
  /*****************/
  /* D30/D31 CODES */
  /*****************/
  static const unsigned char VISCA_WIDE_CON_LENS                = 0x26;
  static const unsigned char   VISCA_WIDE_CON_LENS_SET          = 0x00;

  static const unsigned char VISCA_AT_MODE                      = 0x01;
  static const unsigned char   VISCA_AT_ONOFF                   = 0x10;
  static const unsigned char VISCA_AT_AE                        = 0x02;
  static const unsigned char VISCA_AT_AUTOZOOM                  = 0x03;
  static const unsigned char VISCA_ATMD_FRAMEDISPLAY            = 0x04;
  static const unsigned char VISCA_AT_FRAMEOFFSET               = 0x05;
  static const unsigned char VISCA_ATMD_STARTSTOP               = 0x06;
  static const unsigned char VISCA_AT_CHASE                     = 0x07;
  static const unsigned char   VISCA_AT_CHASE_NEXT              = 0x10;

  static const unsigned char VISCA_MD_MODE                      = 0x08;
  static const unsigned char   VISCA_MD_ONOFF                   = 0x10;
  static const unsigned char VISCA_MD_FRAME                     = 0x09;
  static const unsigned char VISCA_MD_DETECT                    = 0x0A;

  static const unsigned char VISCA_MD_ADJUST                    = 0x00;
  static const unsigned char   VISCA_MD_ADJUST_YLEVEL           = 0x0B;
  static const unsigned char   VISCA_MD_ADJUST_HUELEVEL         = 0x0C;
  static const unsigned char   VISCA_MD_ADJUST_SIZE             = 0x0D;
  static const unsigned char   VISCA_MD_ADJUST_DISPTIME         = 0x0F;
  static const unsigned char   VISCA_MD_ADJUST_REFTIME          = 0x0B;
  static const unsigned char   VISCA_MD_ADJUST_REFMODE          = 0x10;

  static const unsigned char VISCA_AT_ENTRY                     = 0x15;
  static const unsigned char VISCA_AT_LOSTINFO                  = 0x20;
  static const unsigned char VISCA_MD_LOSTINFO                  = 0x21;
  static const unsigned char VISCA_ATMD_LOSTINFO1               = 0x20;
  static const unsigned char VISCA_ATMD_LOSTINFO2               = 0x07;

  static const unsigned char VISCA_MD_MEASURE_MODE_1            = 0x27;
  static const unsigned char VISCA_MD_MEASURE_MODE_2            = 0x28;

  static const unsigned char VISCA_ATMD_MODE                    = 0x22;
  static const unsigned char VISCA_AT_MODE_QUERY                = 0x23;
  static const unsigned char VISCA_MD_MODE_QUERY                = 0x24;
  static const unsigned char VISCA_MD_REFTIME_QUERY             = 0x11;
  static const unsigned char VISCA_AT_POSITION                  = 0x20;
  static const unsigned char VISCA_MD_POSITION                  = 0x21;

  void         recv_packet(unsigned int timeout_ms);
  void         handle_response();
  void         finish_nonblocking(unsigned int socket);

  char         *__device_file;
  int           __fd;
  bool          __opened;
  unsigned int  __default_timeout_ms;

  unsigned int  __inquire;

  unsigned char __recipient;
  unsigned char __sender;

  unsigned char __obuffer[16];
  unsigned char __ibuffer[1024];
  int           __obuffer_length;
  int           __ibuffer_length;

  bool          __blocking;
  bool          __nonblocking_running[2];
  unsigned int  __nonblocking_sockets[2];

  unsigned char __pan_speed;
  unsigned char __tilt_speed;

#ifdef TIMETRACKER_VISCA
  fawkes::TimeTracker *__tt;
  unsigned int         __ttc_pantilt_get_send;
  unsigned int         __ttc_pantilt_get_read;
  unsigned int         __ttc_pantilt_get_handle;
  unsigned int         __ttc_pantilt_get_interpret;
#endif

};



#endif
