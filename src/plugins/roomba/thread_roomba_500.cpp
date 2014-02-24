
/***************************************************************************
 *  thread_roomba_500.cpp - Roomba 500 thread
 *
 *  Created: Sun Jan 02 12:47:35 2011
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

#include "thread_roomba_500.h"
#include <interfaces/Roomba500Interface.h>

#include <utils/time/wait.h>
#include <utils/math/angle.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif

#include <interfaces/LedInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/BatteryInterface.h>
// include <interfaces/MotorInterface.h>

#include <netinet/in.h>
#include <cstdio>

using namespace fawkes;

/** Worker thread for the Roomba 500 thread.
 * @author Tim Niemueller
 */
class Roomba500Thread::WorkerThread : public fawkes::Thread
{
 public:
  /** Constructor.
   * @param logger logger
   * @param clock clock
   * @param roomba refptr to Roomba500 instance
   * @param query_mode true to query data instead of streaming it.
   */
  WorkerThread(fawkes::Logger *logger, fawkes::Clock *clock,
	       fawkes::RefPtr<Roomba500> roomba, bool query_mode)
    : Thread("Roomba500WorkerThread", Thread::OPMODE_CONTINUOUS),
      logger(logger), __roomba(roomba), __query_mode(query_mode)
  {
    __fresh_data_mutex = new Mutex();
    __time_wait = new TimeWait(clock, Roomba500::STREAM_INTERVAL_MS * 1000);

#ifdef USE_TIMETRACKER
    __tt_count  = 0;
    __ttc_query = __tt.add_class("Query");
    __ttc_loop = __tt.add_class("Loop");
#endif

    if (! __query_mode)  __roomba->enable_sensors();
  }

  /** Destructor. */
  ~WorkerThread()
  {
    if (! __query_mode)  __roomba->disable_sensors();
    delete __fresh_data_mutex;
    delete __time_wait;
  }

  virtual void loop()
  {
#ifdef USE_TIMETRACKER
    __tt.ping_start(__ttc_loop);
#endif
    
    //__time_wait->mark_start();

    try {
#ifdef USE_TIMETRACKER
      __tt.ping_start(__ttc_query);
#endif
      if (__query_mode)  __roomba->query_sensors();
      else               __roomba->read_sensors();
#ifdef USE_TIMETRACKER
      __tt.ping_end(__ttc_query);
#endif
      __fresh_data = __roomba->has_sensor_packet();
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to read sensor info, exception follows");
      logger->log_warn(name(), e);
    }

    //__time_wait->wait_systime();

#ifdef USE_TIMETRACKER
      __tt.ping_end(__ttc_loop);
      if (++__tt_count == 300) {
	__tt_count = 0;
	__tt.print_to_stdout();
	__tt.reset();
      }
#endif
  }

  /** Check if fresh data is available.
   * @return true if fresh data is available, false otherwise.
   */
  bool has_fresh_data()
  {
    __fresh_data_mutex->lock();
    bool rv = __fresh_data;
    __fresh_data = false;
    __fresh_data_mutex->unlock();
    return rv;
  }

 private:
  Logger            *logger;
  RefPtr<Roomba500>  __roomba;
  TimeWait          *__time_wait;
  Mutex             *__fresh_data_mutex;
#ifdef USE_TIMETRACKER
  TimeTracker        __tt;
  unsigned int       __ttc_query;
  unsigned int       __ttc_loop;
  unsigned int       __tt_count;
#endif

 private:
  bool __fresh_data;
  bool __query_mode;
};


/** @class Roomba500Thread "thread_roomba_500.h"
 * Roomba 500 integration thread.
 * This thread integrates the Roomba 500 robot into Fawkes. The thread
 * hooks in at the ACT hook and executes received commands on the hardware.
 * @author Tim Niemueller
 */

/** Constructor. */
Roomba500Thread::Roomba500Thread()
  : Thread("Roomba500", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}


void
Roomba500Thread::init()
{
  __led_if_debris = NULL;
  __led_if_spot = NULL;
  __led_if_dock = NULL;
  __led_if_check_robot = NULL;
  __led_if_clean_color = NULL;
  __led_if_clean_intensity = NULL;
  __switch_if_vacuuming = NULL;
  __switch_if_but_clean = NULL;
  __switch_if_but_spot = NULL;
  __switch_if_but_dock = NULL;
  __switch_if_but_minute = NULL;
  __switch_if_but_hour = NULL;
  __switch_if_but_day = NULL;
  __switch_if_but_schedule = NULL;
  __switch_if_but_clock = NULL;
  //__motor_if = NULL;
  __battery_if = NULL;
  __roomba500_if = NULL;

  __greeting_loop_count = 0;

  __cfg_device = "";
  __cfg_btsave = false;

  Roomba500::ConnectionType conntype;
  __cfg_conntype = config->get_string("/hardware/roomba/connection_type");
  __cfg_btfast = false;
  __cfg_bttype = "firefly";
  __cfg_play_fanfare = true;
  __cfg_query_mode = true;

  try {
    __cfg_play_fanfare = config->get_bool("/hardware/roomba/play_fanfare");
  } catch (Exception &e) {}

  try {
    __cfg_query_mode = config->get_bool("/hardware/roomba/query_mode");
  } catch (Exception &e) {}

  if (__cfg_conntype == "rootooth") {
    try {
      __cfg_device = config->get_string("/hardware/roomba/btaddr");
    } catch (Exception &e) {
      try {
	__cfg_device = config->get_string("/hardware/roomba/btname");
      } catch (Exception &e2) {
	logger->log_info(name(), "Neither bluetooth name nor address set, "
			 "trying auto-detect");
      }
    }
    try {
      __cfg_btfast = config->get_bool("/hardware/roomba/btfast");
    } catch (Exception &e) {}

    try {
      __cfg_bttype = config->get_string("/hardware/roomba/bttype");
    } catch (Exception &e) {
      logger->log_info(name(), "RooTooth type not set, assuming 'firefly'");
    }
    if (__cfg_bttype == "firefly") {
      // we're cool
    } else if (__cfg_bttype == "mitsumi") {
      if (__cfg_btfast) {
	logger->log_warn(name(), "Fast mode setting for Mitsumi RooTooth not "
			 "supported, please set outside of Fawkes or wait "
			 "until configuration timeout has passed.");
	__cfg_btfast = false;
      }
    } else {
      logger->log_warn(name(), "Unknown RooTooth hardware type '%s' set",
		       __cfg_bttype.c_str());
      if (__cfg_btfast) {
	logger->log_warn(name(), "Fast mode setting only supported for "
			 "FireFly RooTooth");
	__cfg_btfast = false;
      }
    }

    conntype = Roomba500::CONNTYPE_ROOTOOTH;
  } else if (__cfg_conntype == "serial") {
    __cfg_device = config->get_string("/hardware/roomba/device");
    conntype = Roomba500::CONNTYPE_SERIAL;
  } else {
    throw Exception("Unknown mode '%s', must be rootooth or serial",
		    __cfg_conntype.c_str());
  }

  try {
    __cfg_btsave = config->get_bool("/hardware/roomba/btsave");
  } catch (Exception &e) {}

  Roomba500::Mode mode = Roomba500::MODE_PASSIVE;
  __cfg_mode = "passive";
  try {
    __cfg_mode = config->get_string("/hardware/roomba/mode");
  } catch (Exception &e) {}
  if (__cfg_mode == "passive") {
    mode = Roomba500::MODE_PASSIVE;
  } else if (__cfg_mode == "safe") {
    mode = Roomba500::MODE_SAFE;
  } else if (__cfg_mode == "full") {
    mode = Roomba500::MODE_FULL;
  } else {
    throw Exception("Unknown mode '%s', must be one of passive, safe, or full",
		    __cfg_mode.c_str());
  }


  try {
    __roomba500_if = blackboard->open_for_writing<Roomba500Interface>("Roomba 500");
    __led_if_debris =
      blackboard->open_for_writing<LedInterface>("Roomba LED Debris");
    __led_if_spot = blackboard->open_for_writing<LedInterface>("Roomba LED Spot");
    __led_if_dock = blackboard->open_for_writing<LedInterface>("Roomba LED Dock");
    __led_if_check_robot =
      blackboard->open_for_writing<LedInterface>("Roomba LED Check Robot");
    __led_if_clean_color =
      blackboard->open_for_writing<LedInterface>("Roomba LED Clean Color");
    __led_if_clean_intensity =
      blackboard->open_for_writing<LedInterface>("Roomba LED Clean Intensity");
    __switch_if_vacuuming =
      blackboard->open_for_writing<SwitchInterface>("Roomba Vacuuming");
    __switch_if_but_clean =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Clean");
    __switch_if_but_spot =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Spot");
    __switch_if_but_dock =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Dock");
    __switch_if_but_minute =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Minute");
    __switch_if_but_hour =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Hour");
    __switch_if_but_day =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Day");
    __switch_if_but_schedule =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Schedule");
    __switch_if_but_clock =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Clock");
    //__motor_if = blackboard->open_for_writing<MotorInterface>("Roomba Motor");
    __battery_if = blackboard->open_for_writing<BatteryInterface>("Roomba Battery");
  } catch (Exception &e) {
    close_interfaces();
    throw;
  }

  __wt = NULL;
  try {
    unsigned int flags = 0;
    if (conntype == Roomba500::CONNTYPE_ROOTOOTH) {
      logger->log_debug(name(), "Connecting via RooTooth, this may take a while");
      if (__cfg_btfast) flags |= Roomba500::FLAG_FIREFLY_FASTMODE;
    }
    __roomba = new Roomba500(conntype, __cfg_device.c_str(), flags);

    if (__cfg_btsave) {
      logger->log_debug(name(), "Saving Bluetooth address %s. Will be used for "
			"next connection.", __roomba->get_device());
      config->set_string("/hardware/roomba/btaddr", __roomba->get_device());
    }

    __roomba->set_mode(mode);
    if (__roomba->is_controlled()) {
      if (__cfg_play_fanfare)  __roomba->play_fanfare();
      __roomba->set_leds(false, false, false, true, 0, 255);
    }
    __wt = new WorkerThread(logger, clock, __roomba, __cfg_query_mode);
  } catch (Exception &e) {
    close_interfaces();
    __roomba.clear();
    delete __wt;
    throw;
  }
  __wt->start();
}

void
Roomba500Thread::close_interfaces()
{
  blackboard->close(__led_if_debris);
  blackboard->close(__led_if_spot);
  blackboard->close(__led_if_dock);
  blackboard->close(__led_if_check_robot);
  blackboard->close(__led_if_clean_color);
  blackboard->close(__led_if_clean_intensity);
  blackboard->close(__switch_if_vacuuming);
  blackboard->close(__switch_if_but_clean);
  blackboard->close(__switch_if_but_spot);
  blackboard->close(__switch_if_but_dock);
  blackboard->close(__switch_if_but_minute);
  blackboard->close(__switch_if_but_hour);
  blackboard->close(__switch_if_but_day);
  blackboard->close(__switch_if_but_schedule);
  blackboard->close(__switch_if_but_clock);
  //blackboard->close(__motor_if);
  blackboard->close(__battery_if);
  blackboard->close(__roomba500_if);
}


void
Roomba500Thread::finalize()
{
  __wt->cancel();
  __wt->join();
  delete __wt;
  __roomba->set_mode(Roomba500::MODE_PASSIVE);
  __roomba.clear();
  close_interfaces();
}


float
Roomba500Thread::led_process(fawkes::LedInterface *iface)
{
  float intensity = iface->intensity();
  while (! iface->msgq_empty() ) {
    if (iface->msgq_first_is<LedInterface::TurnOnMessage>()) {
      intensity = 1.0;
    } else if (iface->msgq_first_is<LedInterface::TurnOffMessage>()) {
      intensity = 0.0;
    }
    iface->msgq_pop();
  }
  return intensity;
}

void
Roomba500Thread::loop()
{
  // process actuation
  float led_debris          = led_process(__led_if_debris);
  float led_spot            = led_process(__led_if_spot);
  float led_dock            = led_process(__led_if_dock);
  float led_check_robot     = led_process(__led_if_check_robot);
  float led_clean_color     = led_process(__led_if_clean_color);
  float led_clean_intensity = led_process(__led_if_clean_intensity);

  if ( (led_debris != __led_if_debris->intensity()) ||
       (led_spot != __led_if_spot->intensity()) ||
       (led_dock != __led_if_dock->intensity()) ||
       (led_check_robot != __led_if_check_robot->intensity()) ||
       (led_clean_color != __led_if_clean_color->intensity()) ||
       (led_clean_intensity != __led_if_clean_intensity->intensity()) )
  {
    try {
      __roomba->set_leds(led_debris > 0.5, led_spot > 0.5,
			 led_dock > 0.5, led_check_robot > 0.5,
			 (char)roundf(led_clean_color * 255.),
			 (char)roundf(led_clean_intensity * 255.));
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to set LEDs, exception follows");
      logger->log_warn(name(), e);
    }

    __led_if_debris->set_intensity(led_debris);
    __led_if_spot->set_intensity(led_spot);
    __led_if_dock->set_intensity(led_dock);
    __led_if_check_robot->set_intensity(led_check_robot);
    __led_if_clean_color->set_intensity(led_clean_color);
    __led_if_clean_intensity->set_intensity(led_clean_intensity);

    __led_if_debris->write();
    __led_if_spot->write();
    __led_if_dock->write();
    __led_if_check_robot->write();
    __led_if_clean_color->write();
    __led_if_clean_intensity->write();
  }

  while (! __roomba500_if->msgq_empty() ) {
    if (__roomba500_if->msgq_first_is<Roomba500Interface::StopMessage>())
    {
      try {
	__roomba->stop();
	//__roomba->set_motors(false, false, false, false, false);
	//logger->log_debug(name(), "Stopped");
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to stop robot, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (__roomba500_if->msgq_first_is<Roomba500Interface::SetModeMessage>())
    {
      Roomba500Interface::SetModeMessage *msg =
	__roomba500_if->msgq_first(msg);

      Roomba500::Mode mode = __roomba->get_mode();
      char color     =   0;
      char intensity = 255;

      switch (msg->mode()) {
      case Roomba500Interface::MODE_OFF:
	logger->log_debug(name(), "Switching off");
	mode = Roomba500::MODE_OFF;
	intensity = 0;
	break;
      case Roomba500Interface::MODE_PASSIVE:
	logger->log_debug(name(), "Switching to passive mode");
	mode = Roomba500::MODE_PASSIVE;
	color = 0;
	break;
      case Roomba500Interface::MODE_SAFE:
	logger->log_debug(name(), "Switching to safe mode");
	mode = Roomba500::MODE_SAFE;
	color = 128;
	break;
      case Roomba500Interface::MODE_FULL:
	logger->log_debug(name(), "Switching to full mode");
	mode = Roomba500::MODE_FULL;
	color = 255;
	break;
      default:
	logger->log_warn(name(), "Invalid mode %i received, ignoring",
			 msg->mode());
      }
      try {
	bool was_controlled = __roomba->is_controlled();
	if (! was_controlled) {
	  // set first
	  __roomba->set_mode(mode);
	}
	if (__roomba->is_controlled()) {
	  __roomba->set_leds(__led_if_debris->intensity() >= 0.5,
			     __led_if_spot->intensity() >= 0.5,
			     __led_if_dock->intensity() >= 0.5,
			     __led_if_check_robot->intensity() >= 0.5,
			     color, intensity);
	}
	if (was_controlled) {
	  __roomba->set_mode(mode);
	}
      } catch (Exception &e) {
	logger->log_warn(name(), "Cannot set mode, exception follows");
	logger->log_warn(name(), e);
      }

    } else if (__roomba500_if->msgq_first_is<Roomba500Interface::DockMessage>()) {
      try {
	__roomba->seek_dock();
	logger->log_debug(name(), "Docking");
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to seek dock, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (__roomba500_if->msgq_first_is<Roomba500Interface::DriveStraightMessage>())
    {
      Roomba500Interface::DriveStraightMessage *msg =
	__roomba500_if->msgq_first(msg);

      try {
	__roomba->drive_straight(msg->velocity());
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to drive straight, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (__roomba500_if->msgq_first_is<Roomba500Interface::DriveMessage>())
    {
      Roomba500Interface::DriveMessage *msg =
	__roomba500_if->msgq_first(msg);

      try {
	__roomba->drive(msg->velocity(), msg->radius());
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to drive, exception follows");
	logger->log_warn(name(), e);
      }

    } else  if (__roomba500_if->msgq_first_is<Roomba500Interface::SetMotorsMessage>())
    {
      Roomba500Interface::SetMotorsMessage *msg =
	__roomba500_if->msgq_first(msg);

      try {
	__roomba->set_motors(
	  (msg->main() != Roomba500Interface::BRUSHSTATE_OFF),
	  (msg->side() != Roomba500Interface::BRUSHSTATE_OFF),
	  msg->is_vacuuming(),
	  (msg->main() == Roomba500Interface::BRUSHSTATE_BACKWARD),
	  (msg->side() == Roomba500Interface::BRUSHSTATE_BACKWARD));
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to set motors, exception follows");
	logger->log_warn(name(), e);
      }
    }
    __roomba500_if->msgq_pop();
  }

  if (__roomba->is_controlled()) {
    if (__greeting_loop_count < 50) {
      if (++__greeting_loop_count == 50) {
	__roomba->set_leds(false, false, false, false, 0, 0);
      } else {
	__roomba->set_leds(false, false, false, true,
			   0, __greeting_loop_count * 5);
      }
    }
  }
}


/** Write data to blackboard.
 * To be called by the RoombaSensorThread during the sensor hook to
 * write new data to the blackboard if available.
 */
void
Roomba500Thread::write_blackboard()
{
  if (__wt->has_fresh_data()) {
    const Roomba500::SensorPacketGroupAll sp(__roomba->get_sensor_packet());

    int charge = (int)roundf(((float)ntohs(sp.battery_charge) /
			      (float)ntohs(sp.battery_capacity)) * 100.);

    if (__roomba->is_controlled()) {
      if (charge != __battery_percent) {
	char digits[4];
	snprintf(digits, 4, "%u%%", charge);
	__roomba->set_digit_leds(digits);
	__battery_percent = charge;
      }
    }

    __roomba500_if->set_mode((Roomba500Interface::Mode)sp.mode);
    __roomba500_if->set_wheel_drop_left(
        sp.bumps_wheeldrops & Roomba500::WHEEL_DROP_LEFT);
    __roomba500_if->set_wheel_drop_right(
	sp.bumps_wheeldrops & Roomba500::WHEEL_DROP_RIGHT);
    __roomba500_if->set_bump_left(sp.bumps_wheeldrops & Roomba500::BUMP_LEFT);
    __roomba500_if->set_bump_right(sp.bumps_wheeldrops & Roomba500::BUMP_RIGHT);
    __roomba500_if->set_cliff_left(sp.cliff_left == 1);
    __roomba500_if->set_cliff_front_left(sp.cliff_front_left == 1);
    __roomba500_if->set_cliff_front_right(sp.cliff_front_right == 1);
    __roomba500_if->set_cliff_right(sp.cliff_right == 1);
    __roomba500_if->set_wall(sp.virtual_wall == 1);
    __roomba500_if->set_overcurrent_left_wheel(
        sp.overcurrents & Roomba500::OVERCURRENT_WHEEL_LEFT);
    __roomba500_if->set_overcurrent_right_wheel(
        sp.overcurrents & Roomba500::OVERCURRENT_WHEEL_RIGHT);
    __roomba500_if->set_overcurrent_main_brush(
	sp.overcurrents & Roomba500::OVERCURRENT_MAIN_BRUSH);
    __roomba500_if->set_overcurrent_side_brush(
        sp.overcurrents & Roomba500::OVERCURRENT_SIDE_BRUSH);
    __roomba500_if->set_dirt_detect(sp.dirt_detect == 1);
    __roomba500_if->set_ir_opcode_omni(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_omni);
    __roomba500_if->set_button_clean(sp.buttons & Roomba500::BUTTON_CLEAN);
    __roomba500_if->set_button_spot(sp.buttons & Roomba500::BUTTON_SPOT);
    __roomba500_if->set_button_dock(sp.buttons & Roomba500::BUTTON_DOCK);
    __roomba500_if->set_button_minute(sp.buttons & Roomba500::BUTTON_MINUTE);
    __roomba500_if->set_button_hour(sp.buttons & Roomba500::BUTTON_HOUR);
    __roomba500_if->set_button_day(sp.buttons & Roomba500::BUTTON_DAY);
    __roomba500_if->set_button_schedule(sp.buttons & Roomba500::BUTTON_SCHEDULE);
    __roomba500_if->set_button_clock(sp.buttons & Roomba500::BUTTON_CLOCK);

    __switch_if_but_clean->set_enabled(sp.buttons & Roomba500::BUTTON_CLEAN);
    __switch_if_but_spot->set_enabled(sp.buttons & Roomba500::BUTTON_SPOT);
    __switch_if_but_dock->set_enabled(sp.buttons & Roomba500::BUTTON_DOCK);
    __switch_if_but_minute->set_enabled(sp.buttons & Roomba500::BUTTON_MINUTE);
    __switch_if_but_hour->set_enabled(sp.buttons & Roomba500::BUTTON_HOUR);
    __switch_if_but_day->set_enabled(sp.buttons & Roomba500::BUTTON_DAY);
    __switch_if_but_schedule->set_enabled(sp.buttons & Roomba500::BUTTON_SCHEDULE);
    __switch_if_but_clock->set_enabled(sp.buttons & Roomba500::BUTTON_CLOCK);

    // Convert mm to m for distance
    __roomba500_if->set_distance((int16_t)ntohs(sp.distance));
    // invert because in Fawkes positive angles go counter-clockwise, while
    // for the Roomba they go clockwise
    __roomba500_if->set_angle(- (int16_t)ntohs(sp.angle));
    __roomba500_if->set_charging_state(
	(Roomba500Interface::ChargingState)sp.charging_state);
    __roomba500_if->set_voltage(ntohs(sp.voltage));
    __roomba500_if->set_current((int)ntohs(sp.current));
    __roomba500_if->set_temperature((int)sp.temperature);
    __roomba500_if->set_battery_charge(ntohs(sp.battery_charge));
    __roomba500_if->set_battery_capacity(ntohs(sp.battery_capacity));

    __battery_if->set_voltage(ntohs(sp.voltage));
    __battery_if->set_current((int)ntohs(sp.current));
    __battery_if->set_temperature((char)sp.temperature);
    __battery_if->set_absolute_soc((float)ntohs(sp.battery_charge) /
				   (float)ntohs(sp.battery_capacity));
    __battery_if->set_relative_soc(__battery_if->absolute_soc());

    __roomba500_if->set_wall_signal(ntohs(sp.wall_signal));
    __roomba500_if->set_cliff_left_signal(ntohs(sp.cliff_left_signal));
    __roomba500_if->set_cliff_front_left_signal(ntohs(sp.cliff_front_left_signal));
    __roomba500_if->set_cliff_front_right_signal(ntohs(sp.cliff_front_right_signal));
    __roomba500_if->set_cliff_right_signal(ntohs(sp.cliff_right_signal));
    __roomba500_if->set_home_base_charger_available(
	sp.charger_available & Roomba500::CHARGER_HOME_BASE);
    __roomba500_if->set_internal_charger_available(
	sp.charger_available & Roomba500::CHARGER_INTERNAL);
    __roomba500_if->set_song_number(sp.song_number);
    __roomba500_if->set_song_playing(sp.song_playing == 1);

    __roomba500_if->set_velocity((int16_t)ntohs(sp.velocity));
    __roomba500_if->set_radius((int16_t)ntohs(sp.radius));
    __roomba500_if->set_velocity_right((int16_t)ntohs(sp.velocity_right));
    __roomba500_if->set_velocity_left((int16_t)ntohs(sp.velocity_left));
    __roomba500_if->set_encoder_counts_left(ntohs(sp.encoder_counts_left));
    __roomba500_if->set_encoder_counts_right(ntohs(sp.encoder_counts_right));

    __roomba500_if->set_bumper_left(
	sp.light_bumper & Roomba500::BUMPER_LEFT);
    __roomba500_if->set_bumper_front_left(
	sp.light_bumper & Roomba500::BUMPER_FRONT_LEFT);
    __roomba500_if->set_bumper_center_left(
	sp.light_bumper & Roomba500::BUMPER_CENTER_LEFT);
    __roomba500_if->set_bumper_center_right(
	sp.light_bumper & Roomba500::BUMPER_CENTER_RIGHT);
    __roomba500_if->set_bumper_front_right(
	sp.light_bumper & Roomba500::BUMPER_FRONT_RIGHT);
    __roomba500_if->set_bumper_right(
	sp.light_bumper & Roomba500::BUMPER_RIGHT);

    __roomba500_if->set_light_bump_left(ntohs(sp.light_bump_left));
    __roomba500_if->set_light_bump_front_left(ntohs(sp.light_bump_front_left));
    __roomba500_if->set_light_bump_center_left(ntohs(sp.light_bump_center_left));
    __roomba500_if->set_light_bump_center_right(ntohs(sp.light_bump_center_right));
    __roomba500_if->set_light_bump_front_right(ntohs(sp.light_bump_front_right));
    __roomba500_if->set_light_bump_right(ntohs(sp.light_bump_right));

    __roomba500_if->set_ir_opcode_left(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_left);
    __roomba500_if->set_ir_opcode_right(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_right);

    __roomba500_if->set_left_motor_current((int)ntohs(sp.left_motor_current));
    __roomba500_if->set_right_motor_current((int)ntohs(sp.right_motor_current));
    __roomba500_if->set_side_brush_current((int)ntohs(sp.side_brush_current));
    __roomba500_if->set_main_brush_current((int)ntohs(sp.main_brush_current));
    __roomba500_if->set_caster_stasis(sp.stasis == 1);

    __roomba500_if->write();

    __switch_if_but_clean->write();
    __switch_if_but_spot->write();
    __switch_if_but_dock->write();
    __switch_if_but_minute->write();
    __switch_if_but_hour->write();
    __switch_if_but_day->write();
    __switch_if_but_schedule->write();
    __switch_if_but_clock->write();

    __battery_if->write();
  }
}


/** Set mode and indicate with LED.
 * This will set the mode and if successful also set the color and intensity
 * of the clean LED indicating the mode.
 * @param mode mode to set
 * @exception Exception may be thrown if mode setting fails
 */
void
Roomba500Thread::set_mode(Roomba500::Mode mode)
{
  char color     =   0;
  char intensity = 255;

  switch (mode) {
  case Roomba500::MODE_OFF:     intensity =   0; break;
  case Roomba500::MODE_PASSIVE: color     =   0; break;
  case Roomba500::MODE_SAFE:    color     = 128; break;
  case Roomba500::MODE_FULL:    color     = 255; break;
  }

  __roomba->set_mode(mode);
  __roomba->set_leds(__led_if_debris->intensity() >= 0.5,
		     __led_if_spot->intensity() >= 0.5,
		     __led_if_dock->intensity() >= 0.5,
		     __led_if_check_robot->intensity() >= 0.5,
		     color, intensity);
}
