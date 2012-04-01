
/***************************************************************************
 *  roomba_500.h - Roomba Open Interface implementation for 500 series
 *
 *  Created: Sat Jan 01 19:13:38 2011
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

namespace fawkes {
  class Mutex;
}

class Roomba500
{
 public:
  /** Connection type. */
  typedef enum {
    CONNTYPE_SERIAL,	///< Use serial connection (device file).
    CONNTYPE_ROOTOOTH	///< Use BlueZ to find and connect to RooTooth
  } ConnectionType;

  /** Connection flags.
   * These flags allow to influence the connection creation and operation. */
  typedef enum {
    FLAG_FIREFLY_FASTMODE =  1	///< Enable fast mode, assume FireFly RooTooth
  } ConnectionFlags;

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

  /** Sensor stream state. */
  typedef enum {
    STREAM_ENABLE = 1,		///< Stream enabled.
    STREAM_DISABLE = 0		///< Stream disabled.
  } StreamState;

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

  static const unsigned char CHARGER_HOME_BASE;	///< Home base charger bit.
  static const unsigned char CHARGER_INTERNAL;	///< Internal charger bit.


  /// @cond OBVIOUS
  static const unsigned short int SENSPACK_SIZE_GROUP_0;
  static const unsigned short int SENSPACK_SIZE_GROUP_1;
  static const unsigned short int SENSPACK_SIZE_GROUP_2;
  static const unsigned short int SENSPACK_SIZE_GROUP_3;
  static const unsigned short int SENSPACK_SIZE_GROUP_4;
  static const unsigned short int SENSPACK_SIZE_GROUP_5;
  static const unsigned short int SENSPACK_SIZE_GROUP_6;
  static const unsigned short int SENSPACK_SIZE_GROUP_ALL;
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

  static const short int MAX_LIN_VEL_MM_S;	///< Maximum linear velocity.
  static const short int MAX_RADIUS_MM;	///< Maximum drive radius.

  static const short int MAX_PWM;	///< Maximum PWM value for wheels.
  static const unsigned short int MAX_ENCODER_COUNT;	///< Maximum encoder count.
  static const unsigned short int STREAM_INTERVAL_MS;	///< Time in ms between
						///< streamed sensor packets.
  static const unsigned short int MODE_CHANGE_WAIT_MS;	///< Time in ms to wait
							///< after mode changes

  static const unsigned char CHECKSUM_SIZE;	///< Checksum byte size.

#pragma pack(push,1)
  /** Struct for packet group with everything (SENSPACK_GROUP_ALL). */
  typedef struct {
    uint8_t  bumps_wheeldrops;		///< Bumps and wheeldrops bits.
    uint8_t  wall;			///< Wall sensor value.
    uint8_t  cliff_left;		///< Left cliff sensor.
    uint8_t  cliff_front_left;		///< Front left cliff sensor.
    uint8_t  cliff_front_right;		///< Front right cliff sensor.
    uint8_t  cliff_right;		///< Right cliff sensor.
    uint8_t  virtual_wall;		///< Wall sensor.
    uint8_t  overcurrents;		///< Overcurrents bits.
    uint8_t  dirt_detect;		///< Dirt detect sensor.
    uint8_t  unused_1;			///< Unused byte.
    uint8_t  ir_opcode_omni;		///< Omni IR receiver character.
    uint8_t  buttons;			///< Buttons bits.
    int16_t  distance;			///< Traveled distance in mm.
    int16_t  angle;			///< Turned angle in degree.
    uint8_t  charging_state;		///< Charging state.
    uint16_t voltage;			///< Voltage in mV.
    int16_t  current;			///< Current in mA.
    int8_t   temperature;		///< Temperature in deg C.
    uint16_t battery_charge;		///< Battery charge in mAh.
    uint16_t battery_capacity;		///< Battery capacity in mAh.
    uint16_t wall_signal;		///< Raw wall signal.
    uint16_t cliff_left_signal;		///< Raw left cliff signal.
    uint16_t cliff_front_left_signal;	///< Raw front left cliff signal.
    uint16_t cliff_front_right_signal;	///< Raw front right cliff signal.
    uint16_t cliff_right_signal;	///< Raw right cliff signal.
    uint8_t  unused_2;			///< Unused byte.
    uint16_t unused_3;			///< Unused byte.
    uint8_t  charger_available;		///< Available chargers bits.
    uint8_t  mode;			///< Open Interface mode.
    uint8_t  song_number;		///< Song number.
    uint8_t  song_playing;		///< Song playing byte.
    uint8_t  stream_num_packets;	///< Number of streamed packets.
    int16_t  velocity;			///< Velocity in mm/sec.
    int16_t  radius;			///< Radius in mm.
    int16_t  velocity_right;		///< Velocity of right wheel in mm/sec.
    int16_t  velocity_left;		///< Velocity of left wheel in mm/sec.
    uint16_t encoder_counts_left;	///< Encoder counts for left wheel.
    uint16_t encoder_counts_right;	///< Encoder counts for right wheel.
    uint8_t  light_bumper;		///< Light bumper bits.
    uint16_t light_bump_left;		///< Raw left light bumper signal.
    uint16_t light_bump_front_left;	///< Raw front left light bumper signal.
    uint16_t light_bump_center_left;	///< Raw center left light bumper signal.
    uint16_t light_bump_center_right;	///< Raw center right light bumper signal.
    uint16_t light_bump_front_right;	///< Raw front right light bumper signal.
    uint16_t light_bump_right;  	///< Raw right light bumper signal.
    uint8_t  ir_opcode_left;		///< Left IR receiver character.
    uint8_t  ir_opcode_right;		///< Right IR receiver character.
    int16_t  left_motor_current;	///< Raw left motor current signal.
    int16_t  right_motor_current;	///< Raw right motor current signal.
    int16_t  main_brush_current;	///< Raw main brush motor current signal.
    int16_t  side_brush_current;	///< Raw side brush motor current signal.
    uint8_t  stasis;			///< Castor stasis.
  } SensorPacketGroupAll;
#pragma pack(pop)

 public:
  Roomba500(ConnectionType conntype, const char *device, unsigned int flags = 0);
  ~Roomba500();

  void open();
  void close();

  /** Check if connection has been established.
   * @return true if connection has been established, false otherwise. */
  bool is_connected() const { return (__fd != -1); }

  /** Get connection type.
   * @return connection type */
  ConnectionType  get_connection_type() const { return __conntype; }
  /** Get device string.
   * @return device string */
  const char * get_device() const { return __device; }

  /** Get current mode.
   * @return current mode. */
  Mode get_mode() const { return __mode; }

  /** Check if robot is being controlled.
   * @return true if robot is being controlled, false otherwise. */
  bool is_controlled() const
  { return is_connected() && ( (__mode == MODE_SAFE) || (__mode == MODE_FULL) ); }

  void set_mode(Mode mode);
  void clean();
  void clean_spot();
  void seek_dock();
  void power_down();
  void stop();
  void drive_straight(short int velo_mm_per_sec);
  void drive_turn(TurnDirection direction);
  void drive_arc(short int velo_mm_per_sec, short int radius_mm);
  void drive(short int velocity_mm_per_sec, short int radius_mm);
  void drive_direct(short int left_mm_per_sec, short int right_mm_per_sec);
  void drive_pwm(short int left_wheel_pwm, short int right_wheel_pwm);
  void set_motors(bool main = true, bool side = true, bool vacuum = true,
		  bool main_backward = false, bool side_backward = false);
  void set_leds(bool debris, bool spot, bool dock, bool check_robot,
		unsigned char clean_color, unsigned char clean_intensity);

  void set_digit_leds(const char digits[4]);

  void enable_sensors();
  void disable_sensors();
  bool is_data_available();
  void read_sensors();
  void query_sensors();
  /** Check if sensor packet is availabe.
   * @return true if sensor packet is available, false otherwise
   */
  bool has_sensor_packet() const
  { return __sensor_packet_received; };
  const SensorPacketGroupAll  get_sensor_packet() const;

  void play_fanfare();

  static unsigned short int get_packet_size(SensorPacketID packet);

 private:
  void send(OpCode opcode,
	    const void *params = NULL, const size_t plength = 0);
  void recv(size_t index, size_t num_bytes, unsigned int timeout_ms = 0);

  void assert_control()
  {
    if ((__mode != MODE_FULL) && (__mode != MODE_SAFE)) {
      throw fawkes::Exception("Command only available in FULL or SAFE mode.");
    }
  }
  void assert_connected()
  { if (__mode == MODE_OFF) throw fawkes::Exception("Not connected to robot."); }

 private:
  ConnectionType        __conntype;
  unsigned int          __conn_flags;

  Mode                  __mode;
  SensorPacketID        __packet_id;
  unsigned char         __packet_reply_id;
  unsigned short        __packet_length;
  bool                  __sensors_enabled;
  SensorPacketGroupAll  __sensor_packet;
  bool                  __sensor_packet_received;
  fawkes::Mutex        *__sensor_mutex;

  char                 *__device;
  int                   __fd;
  fawkes::Mutex        *__read_mutex;
  fawkes::Mutex        *__write_mutex;

  unsigned char         __obuffer[16];
  unsigned char         __ibuffer[82];

  int                   __obuffer_length;
  int                   __ibuffer_length;
};

#endif
