
/***************************************************************************
 *  roomba_500.h - Roomba Open Interface implementation for 500 series
 *
 *  Created: Sat Jan 01 19:13:38 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROOMBA_ROOMBA_500_H_
#define __PLUGINS_ROOMBA_ROOMBA_500_H_

#include <core/plugin.h>

#include <stdint.h>

class Roomba500
{
 public:
  /** Roomba 500 Command op codes. */
  typedef enum {
    OPCODE_START		= 128,	///< Initiate communication with Roomba.
    OPCODE_BAUD			= 129,	///< Set baud rate.
    OPCODE_CONTROL		= 130,	///< Old alias for SAFE.
    OPCODE_SAFE			= 131,	///< Enter safe mode.
    OPCODE_FULL			= 132,	///< Enter full mode.
    OPCODE_POWER		= 133,	///< Power down Roomba.
    OPCODE_SPOT			= 134,	///< Start spot cleaning.
    OPCODE_CLEAN		= 135,	///< Start normal cleaning mission.
    OPCODE_MAX			= 136,	///< Start max cleaning mode.
    OPCODE_DRIVE		= 137,	///< Drive robot.
    OPCODE_MOTORS		= 138,	///< Set motor state.
    OPCODE_LEDS			= 139,	///< Control LEDs.
    OPCODE_SONG			= 140,	///< Register song.
    OPCODE_PLAY			= 141,	///< Play song.
    OPCODE_QUERY		= 142,	///< Query sensor info.
    OPCODE_SEEK_DOCK		= 143,	///< Start seeking dock.
    OPCODE_PWM_MOTORS		= 144,	///< PWM control of motors.
    OPCODE_DRIVE_WHEELS		= 145,	///< Drive wheels.
    OPCODE_DRIVE_PWM		= 146,	///< Drive by PWM.
    OPCODE_STREAM		= 148,	///< Start streaming of data.
    OPCODE_QUERY_LIST		= 149,	///< Query multiple sensor packets.
    OPCODE_PAUSE_RESUME_STREAM	= 150,	///< Pause or resume streaming data.
    OPCODE_SCHEDULE_LEDS        = 162,	///< Control schedule LEDs.
    OPCODE_DIGIT_LEDS_RAW	= 163,	///< Raw control of digit LEDs.
    OPCODE_DIGIT_LEDS_ASCII	= 164,	///< Ascii control of digit LEDs.
    OPCODE_BUTTONS		= 165,	///< Control buttons.
    OPCODE_SCHEDULE		= 167,	///< Manipulate schedule.
    OPCODE_SET_DAY_TIME		= 168	///< Set day and time.
  } OpCode;

  /** Roomba 500 sensor package IDs. */
  typedef enum {
    SENSPACK_BUMPS_DROPS		=  7,	///< Bumper and wheel drops.
    SENSPACK_WALL			=  8,	///< Wall sensor.
    SENSPACK_CLIFF_LEFT			=  9,	///< Left cliff sensor.
    SENSPACK_CLIFF_FRONT_LEFT		= 10,	///< Front left cliff sensor.
    SENSPACK_CLIFF_FRONT_RIGHT		= 11,	///< Front right cliff sensor.
    SENSPACK_CLIFF_RIGHT		= 12,	///< Right cliff sensor.
    SENSPACK_VIRTUAL_WALL		= 13,	///< Virtual wall detector.
    SENSPACK_WHEEL_OVERCURRENTS		= 14,	///< Overcurrents.
    SENSPACK_DIRT_DETECT		= 15,	///< Dirt detection sensor.
    SENSPACK_IR_CHAR_OMNI		= 17,	///< Omnidirectional IR receiver.
    SENSPACK_BUTTONS			= 18,	///< Button status.
    SENSPACK_DISTANCE			= 19,	///< Travelled distance.
    SENSPACK_ANGLE			= 20,	///< Turned angle.
    SENSPACK_CHARGING_STATE		= 21,	///< Charging state.
    SENSPACK_VOLTAGE			= 22,	///< Voltage.
    SENSPACK_CURRENT			= 23,	///< Current.
    SENSPACK_TEMPERATURE		= 24,	///< Temperature.
    SENSPACK_BATTERY_CHARGE		= 25,	///< Battery charge.
    SENSPACK_BATTERY_CAPACITY		= 26,	///< Battery capacity.
    SENSPACK_WALL_SIGNAL		= 27,	///< Wall signal value.
    SENSPACK_CLIFF_LEFT_SIGNAL		= 28,	///< Left cliff signal value.
    SENSPACK_CLIFF_FRONT_LEFT_SIGNAL	= 29,	///< Front left cliff signal value.
    SENSPACK_CLIFF_FRONT_RIGHT_SIGNAL	= 30,	///< Right cliff signal value.
    SENSPACK_CLIFF_RIGHT_SIGNAL		= 31,	///<Front right cliff signal value.
    SENSPACK_CHARGE_SOURCES		= 34,	///< Available charge sources.
    SENSPACK_OI_MODE			= 35,	///< Open Interface mode.
    SENSPACK_SONG_NUMBER		= 36,	///< Song number.
    SENSPACK_SONG_PLAYING		= 37,	///< Song playing indicator.
    SENSPACK_STREAM_PACKETS		= 38,	///< Number of stream packets.
    SENSPACK_REQ_VELOCITY		= 39,	///< Requested velocity.
    SENSPACK_REQ_RADIUS			= 40,	///< Requested radius.
    SENSPACK_REQ_RIGHT_VELOCITY		= 41,	///< Requested right velocity.
    SENSPACK_REQ_LEFT_VELOCITY		= 42,	///< Requested left velocity.
    SENSPACK_RIGHT_ENCODER		= 43,	///< Right encoder value.
    SENSPACK_LEFT_ENCODER		= 44,	///< Left encoder value.
    SENSPACK_LIGHT_BUMPER		= 45,	///< Light bumper status.
    SENSPACK_LIGHT_BUMPER_LEFT		= 46,	///< Left bumper signal.
    SENSPACK_LIGHT_BUMPER_FRONT_LEFT	= 47,	///< Front left bumper signal.
    SENSPACK_LIGHT_BUMPER_CENTER_LEFT	= 48,	///< Center left bumper signal.
    SENSPACK_LIGHT_BUMPER_CENTER_RIGHT	= 49,	///< Center right bumper signal.
    SENSPACK_LIGHT_BUMPER_FRONT_RIGHT	= 50,	///< Front right bumper signal.
    SENSPACK_LIGHT_BUMPER_RIGHT		= 51,	///< Right bumper signal.
    SENSPACK_IR_CHAR_LEFT		= 52,	///< Left IR character.
    SENSPACK_IR_CHAR_RIGHT		= 53,	///< Right IR character.
    SENSPACK_LEFT_MOTOR_CURRENT		= 54,	///< Left motor current.
    SENSPACK_RIGHT_MOTOR_CURRENT	= 55,	///< Right motor current.
    SENSPACK_BRUSH_MOTOR_CURRENT	= 56,	///< Brush motor current.
    SENSPACK_SIDE_BRUSH_MOTOR_CURRENT	= 57,	///< Side brush motor current.
    SENSPACK_STASIS			= 58,	///< Caster wheel stasis (forward
						///< movement)

    SENSPACK_GROUP_0			= 0,	///< Packet IDs 7-26
    SENSPACK_GROUP_1			= 1,	///< Packet IDs 7-16
    SENSPACK_GROUP_2			= 2,	///< Packet IDs 17-20
    SENSPACK_GROUP_3			= 3,	///< Packet IDs 21-26
    SENSPACK_GROUP_4			= 4,	///< Packet IDs 27-34
    SENSPACK_GROUP_5			= 5,	///< Packet IDs 35-42
    SENSPACK_GROUP_6			= 6,	///< Packet IDs 7-42
    SENSPACK_GROUP_ALL			= 100,	///< All packet IDs (7-58)
    SENSPACK_GROUP_101			= 101,	///< Packet IDs 43-58
    SENSPACK_GROUP_106			= 106,	///< Packet IDs 46-51
    SENSPACK_GROUP_107			= 107,	///< Packet IDs 54-58
  } SensorPacketID;

  /** Roomba 500 operation mode. */
  typedef enum {
    MODE_OFF		= 0,	///< No connection.
    MODE_PASSIVE	= 1,	///< Passive mode, no control, only listening.
    MODE_SAFE		= 2,	///< Control acquired, safety measures in place.
    MODE_FULL		= 3	///< Control acquired, safety measures disabled.
  } Mode;

  /** Charging state. */
  typedef enum {
    CHARGING_NO			= 0,	///< Not charging.
    CHARGING_RECONDITIONING	= 1,	///< Reconditioning battery.
    CHARGING_FULL		= 2,	///< Full charging cycle.
    CHARGING_TRICKLE		= 3,	///< Trickle charging.
    CHARGING_WAITING		= 4,	///< Waiting.
    CHARGING_ERROR		= 5	///< Fault condition.
  } ChargingState;

  /** Infrared character values. */
  typedef enum {
    IR_REMOTE_LEFT		= 129,	///< IR Remote Control: left button
    IR_REMOTE_FORWARD		= 130,	///< IR Remote Control: forward button
    IR_REMOTE_RIGHT		= 131,	///< IR Remote Control: right button
    IR_REMOTE_SPOT		= 132,	///< IR Remote Control: spot button
    IR_REMOTE_MAX		= 133,	///< IR Remote Control: max button
    IR_REMOTE_SMALL		= 134,	///< IR Remote Control: small button
    IR_REMOTE_MEDIUM		= 135,	///< IR Remote Control: medium button
    IR_REMOTE_LARGE_CLEAN	= 136,	///< IR Remote Control: large/clean button
    IR_REMOTE_STOP		= 137,	///< IR Remote Control: stop button
    IR_REMOTE_POWER		= 138,	///< IR Remote Control: power button
    IR_REMOTE_ARC_LEFT		= 139,	///< IR Remote Control: left arc button
    IR_REMOTE_ARC_RIGHT		= 140,	///< IR Remote Control: right arc button
    IR_REMOTE_STOP2		= 141,	///< IR Remote Control: stop button
    IR_SCHED_REMOTE_DOWNLOAD	= 142,	///< IR scheduling remote: download button
    IR_SCHED_REMOTE_SEEK_DOCK	= 143,	///< IR scheduling remote: seek dock button
    // just for completeness
    IR_DISC_DOCK_RESERVED		= 240,  ///< Roomba Discovery dock: reserved
    IR_DISC_DOCK_RED_BUOY		= 248,  ///< Roomba Discovery dock: red buoy
    IR_DISC_DOCK_GREEN_BUOY		= 244,  ///< Roomba Discovery dock: green buoy
    IR_DISC_DOCK_FORCE_FIELD		= 242,  ///< Roomba Discovery dock: red
						///< and green buoy
    IR_DISC_DOCK_RED_GREEN_BUOY		= 252,  ///< Roomba Discovery dock: red
						///< buoy and force field
    IR_DISC_DOCK_RED_BUOY_FORCE_FIELD	= 250,  ///< Roomba Discovery dock: green
						///< buoy and force field
    IR_DISC_DOCK_GREEN_BUOY_FORCE_FIELD	= 246,  ///< Roomba Discovery dock: green
						///< buoy and force field.
    IR_DISC_DOCK_RED_GREEN_BUOY_FORCE_FIELD = 254,  ///< Roomba Discovery dock: red
						///< and green buoy and force field
    IR_DOCK_RESERVED		= 160,  ///< Roomba 500 dock: reserved
    IR_DOCK_RED_BUOY		= 168,  ///< Roomba 500 dock: red buoy
    IR_DOCK_GREEN_BUOY		= 164,  ///< Roomba 500 dock: green buoy
    IR_DOCK_FORCE_FIELD		= 161,  ///< Roomba 500 dock: red and green buoy
    IR_DOCK_RED_GREEN_BUOY	= 172,  ///< Roomba 500 dock: red buoy and force
						///< field
    IR_DOCK_RED_BUOY_FORCE_FIELD	= 169,  ///< Roomba 500 dock: green buoy
						///< and force field
    IR_DOCK_GREEN_BUOY_FORCE_FIELD	= 165,  ///< Roomba 500 dock: green buoy
						///< and force field.
    IR_DOCK_RED_GREEN_BUOY_FORCE_FIELD	= 173,  ///< Roomba 500 dock: red and
						///< green buoy and force field.
    IR_VIRTUAL_WALL		= 162	///< IR Virtual Wall
  } InfraredCharacter;

  /** Days for scheduler. */
  typedef enum {
    DAY_SUNDAY		= 0,	///< Sunday.
    DAY_MONDAY		= 1,	///< Monday.
    DAY_TUESDAY		= 2,	///< Tuesday.
    DAY_WEDNESDAY	= 3,	///< Wednesday.
    DAY_THURSDAY	= 4,	///< Thursday.
    DAY_FRIDAY		= 5,	///< Friday.
    DAY_SATURDAY	= 6	///< Saturday.
  } ScheduleDay;

  /** Turning direction. */
  typedef enum {
    TURN_CLOCKWISE,		///< Clockwise turning.
    TURN_COUNTER_CLOCKWISE	///< Counter-Clockwise turning.
  } TurnDirection;

  static const unsigned char BUTTON_CLEAN;	///< Cleaning button.
  static const unsigned char BUTTON_SPOT;	///< Spot cleaning button.
  static const unsigned char BUTTON_DOCK;	///< Dock button.
  static const unsigned char BUTTON_MINUTE;	///< Minute button.
  static const unsigned char BUTTON_HOUR;	///< Hour button.
  static const unsigned char BUTTON_DAY;	///< Day button.
  static const unsigned char BUTTON_SCHEDULE;	///< Schedule button.
  static const unsigned char BUTTON_CLOCK;	///< Clock button.

  static const unsigned char WHEEL_DROP_LEFT;	///< Left wheel drop bit.
  static const unsigned char WHEEL_DROP_RIGHT;	///< Right wheel drop bit.
  static const unsigned char BUMP_LEFT;		///< Left bumper bit.
  static const unsigned char BUMP_RIGHT;	///< Right bumper bit.

  static const unsigned char OVERCURRENT_WHEEL_LEFT;	///< Left wheel bit.
  static const unsigned char OVERCURRENT_WHEEL_RIGHT;	///< Right wheel bit.
  static const unsigned char OVERCURRENT_MAIN_BRUSH;	///< Main brush bit.
  static const unsigned char OVERCURRENT_SIDE_BRUSH;	///< Side brush bit.

  static const unsigned char CHARGING_SOURCE_HOME_BASE; ///< Docking station.
  static const unsigned char CHARGING_SOURCE_INTERNAL;  ///< Internal socket.

  static const unsigned char BUMPER_LEFT;         ///< Left bumper.
  static const unsigned char BUMPER_FRONT_LEFT;   ///< Front left bumper.
  static const unsigned char BUMPER_CENTER_LEFT;  ///< Center left bumper.
  static const unsigned char BUMPER_CENTER_RIGHT; ///< Center right bumper.
  static const unsigned char BUMPER_FRONT_RIGHT;  ///< Front right bumper.
  static const unsigned char BUMPER_RIGHT;        ///< Right bumper.

  static const unsigned char LED_DEBRIS;	///< Debris LED bit.
  static const unsigned char LED_SPOT;		///< Spot LED bit.
  static const unsigned char LED_DOCK;		///< Dock LED bit.
  static const unsigned char LED_CHECK_ROBOT;	///< Check robot LED bit.

  static const unsigned char WEEKDAY_LED_SUN;	///< Sunday.
  static const unsigned char WEEKDAY_LED_MON;	///< Monday.
  static const unsigned char WEEKDAY_LED_TUE;	///< Tuesday.
  static const unsigned char WEEKDAY_LED_WED;	///< Wednesday.
  static const unsigned char WEEKDAY_LED_THU;	///< Thursday.
  static const unsigned char WEEKDAY_LED_FRI;	///< Friday.
  static const unsigned char WEEKDAY_LED_SAT;	///< Saturday.

  static const unsigned char SCHEDULING_LED_COLON;	///< Colon LED bit.
  static const unsigned char SCHEDULING_LED_PM;		///< PM LED bit.
  static const unsigned char SCHEDULING_LED_AM;		///< AM LED bit.
  static const unsigned char SCHEDULING_LED_CLOCK;	///< Clock LED bit.
  static const unsigned char SCHEDULING_LED_SCHEDULE;	///< Schedule LED bit.

  static const unsigned char DIGIT_LED_NORTH;		///< Top segment LED.
  static const unsigned char DIGIT_LED_NORTH_WEST;	///< Top left segment LED.
  static const unsigned char DIGIT_LED_NORTH_EAST;	///< Top right segment LED.
  static const unsigned char DIGIT_LED_CENTER;		///< Center segment LED.
  static const unsigned char DIGIT_LED_SOUTH_WEST;	///< Bottom left segment.
  static const unsigned char DIGIT_LED_SOUTH_EAST;	///< Bottom right segment.
  static const unsigned char DIGIT_LED_SOUTH;		///< Bottom segment LED.

  static const unsigned char MOTOR_SIDE_BRUSH;	///< Side brush motor bit.
  static const unsigned char MOTOR_VACUUM;	///< Vacuum motor bit.
  static const unsigned char MOTOR_MAIN_BRUSHES;	///< Main brush motor bit.
  static const unsigned char MOTOR_SIDE_BRUSH_BACKWARD;	///< Side backward bit.
  static const unsigned char MOTOR_MAIN_BRUSHES_BACKWARD; ///< Main backward bit.

  /// @cond OBVIOUS
  static const unsigned short int SENSPACK_SIZE_GROUP_0;
  static const unsigned short int SENSPACK_SIZE_GROUP_1;
  static const unsigned short int SENSPACK_SIZE_GROUP_2;
  static const unsigned short int SENSPACK_SIZE_GROUP_3;
  static const unsigned short int SENSPACK_SIZE_GROUP_4;
  static const unsigned short int SENSPACK_SIZE_GROUP_5;
  static const unsigned short int SENSPACK_SIZE_GROUP_6;
  static const unsigned short int SENSPACK_SIZE_GROUP_100;
  static const unsigned short int SENSPACK_SIZE_GROUP_101;
  static const unsigned short int SENSPACK_SIZE_GROUP_106;
  static const unsigned short int SENSPACK_SIZE_GROUP_107;
  static const unsigned short int SENSPACK_SIZE_BUMPS_DROPS;
  static const unsigned short int SENSPACK_SIZE_WALL;
  static const unsigned short int SENSPACK_SIZE_CLIFF_LEFT;
  static const unsigned short int SENSPACK_SIZE_CLIFF_FRONT_LEFT;
  static const unsigned short int SENSPACK_SIZE_CLIFF_FRONT_RIGHT;
  static const unsigned short int SENSPACK_SIZE_CLIFF_RIGHT;
  static const unsigned short int SENSPACK_SIZE_VIRTUAL_WALL;
  static const unsigned short int SENSPACK_SIZE_WHEEL_OVERCURRENTS;
  static const unsigned short int SENSPACK_SIZE_DIRT_DETECT;	
  static const unsigned short int SENSPACK_SIZE_IR_CHAR_OMNI;
  static const unsigned short int SENSPACK_SIZE_IR_CHAR_LEFT;
  static const unsigned short int SENSPACK_SIZE_IR_CHAR_RIGHT;
  static const unsigned short int SENSPACK_SIZE_BUTTONS;	
  static const unsigned short int SENSPACK_SIZE_DISTANCE;
  static const unsigned short int SENSPACK_SIZE_ANGLE;
  static const unsigned short int SENSPACK_SIZE_CHARGING_STATE;
  static const unsigned short int SENSPACK_SIZE_VOLTAGE;
  static const unsigned short int SENSPACK_SIZE_CURRENT;
  static const unsigned short int SENSPACK_SIZE_TEMPERATURE;
  static const unsigned short int SENSPACK_SIZE_BATTERY_CHARGE;
  static const unsigned short int SENSPACK_SIZE_BATTERY_CAPACITY;
  static const unsigned short int SENSPACK_SIZE_WALL_SIGNAL;
  static const unsigned short int SENSPACK_SIZE_CLIFF_LEFT_SIGNAL;
  static const unsigned short int SENSPACK_SIZE_CLIFF_FRONT_LEFT_SIGNAL;
  static const unsigned short int SENSPACK_SIZE_CLIFF_FRONT_RIGHT_SIGNAL;
  static const unsigned short int SENSPACK_SIZE_CLIFF_RIGHT_SIGNAL;
  static const unsigned short int SENSPACK_SIZE_CHARGE_SOURCES;
  static const unsigned short int SENSPACK_SIZE_OI_MODE;
  static const unsigned short int SENSPACK_SIZE_SONG_NUMBER;
  static const unsigned short int SENSPACK_SIZE_SONG_PLAYING;
  static const unsigned short int SENSPACK_SIZE_STREAM_PACKETS;
  static const unsigned short int SENSPACK_SIZE_REQ_VELOCITY;
  static const unsigned short int SENSPACK_SIZE_REQ_RADIUS;
  static const unsigned short int SENSPACK_SIZE_REQ_RIGHT_VELOCITY;
  static const unsigned short int SENSPACK_SIZE_REQ_LEFT_VELOCITY;
  static const unsigned short int SENSPACK_SIZE_RIGHT_ENCODER;
  static const unsigned short int SENSPACK_SIZE_LEFT_ENCODER;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_LEFT;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_FRONT_LEFT;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_CENTER_LEFT;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_CENTER_RIGHT;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_FRONT_RIGHT;
  static const unsigned short int SENSPACK_SIZE_LIGHT_BUMPER_RIGHT;
  static const unsigned short int SENSPACK_SIZE_LEFT_MOTOR_CURRENT;	
  static const unsigned short int SENSPACK_SIZE_RIGHT_MOTOR_CURRENT;
  static const unsigned short int SENSPACK_SIZE_BRUSH_MOTOR_CURRENT;
  static const unsigned short int SENSPACK_SIZE_SIDE_BRUSH_MOTOR_CURRENT;
  static const unsigned short int SENSPACK_SIZE_STASIS;
  /// @endcond

  static const float DIAMETER;		///< Robot diameter.
  static const float BUMPER_X_OFFSET;	///< X Offset of bumper.
  static const float AXLE_LENGTH;	///< Axle length.

  static const float MAX_LIN_VEL_MM_S;	///< Maximum linear velocity.
  static const float MAX_ANG_VEL_RAD_S;	///< Maximum angular velocity.
  static const float MAX_RADIUS_MM;	///< Maximum drive radius.

  static const short int MAX_PWM;	///< Maximum PWM value for wheels.
  static const unsigned short int MAX_ENCODER_COUNT;	///< Maximum encoder count.

 public:
  Roomba500(const char *device_file);
  ~Roomba500();

  void open();
  void close();

  void set_mode(Mode mode);
  void clean();
  void clean_spot();
  void seek_dock();
  void power_down();
  void stop();
  void drive_straight(float velocity_m_per_sec);
  void drive_turn(TurnDirection direction);
  void drive_arc(float velocity_m_per_sec, float radius_m);
  void drive_direct(float left_velo_m_per_sec, float right_velo_m_per_sec);
  void drive_pwm(short int left_wheel_pwm, short int right_wheel_pwm);
  void disable_brushes();
  void enable_brushes(bool main = true, bool side = true, bool vacuum = true,
		      bool main_backward = false, bool side_backward = false);

  /*
  void enable_sensors(SensorPacketID packets = SENSPACK_GROUP_ALL);
  void disable_sensors();

  bool is_data_available();
  void read_data();
  */

 private:
  void send(OpCode opcode,
	    const void *params = NULL, const size_t plength = 0);
  void assert_control()
  {
    if ((__mode != MODE_FULL) && (__mode != MODE_SAFE)) {
      throw fawkes::Exception("Stop command only available in FULL or SAFE mode.");
    }
  }


 private:
  char *__device_file;
  int   __fd;

  Mode __mode;
  bool __sensors_enabled;


  unsigned char __obuffer[16];
  unsigned char __ibuffer[82];

  int           __obuffer_length;
  int           __ibuffer_length;
};

#endif
