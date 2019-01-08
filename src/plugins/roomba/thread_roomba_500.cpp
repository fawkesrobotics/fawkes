
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
      logger(logger), roomba_(roomba), query_mode_(query_mode)
  {
    fresh_data_mutex_ = new Mutex();
    time_wait_ = new TimeWait(clock, Roomba500::STREAM_INTERVAL_MS * 1000);

#ifdef USE_TIMETRACKER
    tt_count_  = 0;
    ttc_query_ = tt_.add_class("Query");
    ttc_loop_ = tt_.add_class("Loop");
#endif

    if (! query_mode_)  roomba_->enable_sensors();
  }

  /** Destructor. */
  ~WorkerThread()
  {
    if (! query_mode_)  roomba_->disable_sensors();
    delete fresh_data_mutex_;
    delete time_wait_;
  }

  virtual void loop()
  {
#ifdef USE_TIMETRACKER
    tt_.ping_start(ttc_loop_);
#endif
    
    //time_wait_->mark_start();

    try {
#ifdef USE_TIMETRACKER
      tt_.ping_start(ttc_query_);
#endif
      if (query_mode_)  roomba_->query_sensors();
      else               roomba_->read_sensors();
#ifdef USE_TIMETRACKER
      tt_.ping_end(ttc_query_);
#endif
      fresh_data_ = roomba_->has_sensor_packet();
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to read sensor info, exception follows");
      logger->log_warn(name(), e);
    }

    //time_wait_->wait_systime();

#ifdef USE_TIMETRACKER
      tt_.ping_end(ttc_loop_);
      if (++tt_count_ == 300) {
	tt_count_ = 0;
	tt_.print_to_stdout();
	tt_.reset();
      }
#endif
  }

  /** Check if fresh data is available.
   * @return true if fresh data is available, false otherwise.
   */
  bool has_fresh_data()
  {
    fresh_data_mutex_->lock();
    bool rv = fresh_data_;
    fresh_data_ = false;
    fresh_data_mutex_->unlock();
    return rv;
  }

 private:
  Logger            *logger;
  RefPtr<Roomba500>  roomba_;
  TimeWait          *time_wait_;
  Mutex             *fresh_data_mutex_;
#ifdef USE_TIMETRACKER
  TimeTracker        tt_;
  unsigned int       ttc_query_;
  unsigned int       ttc_loop_;
  unsigned int       tt_count_;
#endif

 private:
  bool fresh_data_;
  bool query_mode_;
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
  led_if_debris_ = NULL;
  led_if_spot_ = NULL;
  led_if_dock_ = NULL;
  led_if_check_robot_ = NULL;
  led_if_clean_color_ = NULL;
  led_if_clean_intensity_ = NULL;
  switch_if_vacuuming_ = NULL;
  switch_if_but_clean_ = NULL;
  switch_if_but_spot_ = NULL;
  switch_if_but_dock_ = NULL;
  switch_if_but_minute_ = NULL;
  switch_if_but_hour_ = NULL;
  switch_if_but_day_ = NULL;
  switch_if_but_schedule_ = NULL;
  switch_if_but_clock_ = NULL;
  //motor_if_ = NULL;
  battery_if_ = NULL;
  roomba500_if_ = NULL;

  greeting_loop_count_ = 0;

  cfg_device_ = "";
  cfg_btsave_ = false;

  Roomba500::ConnectionType conntype;
  cfg_conntype_ = config->get_string("/hardware/roomba/connection_type");
  cfg_btfast_ = false;
  cfg_bttype_ = "firefly";
  cfg_play_fanfare_ = true;
  cfg_query_mode_ = true;

  try {
    cfg_play_fanfare_ = config->get_bool("/hardware/roomba/play_fanfare");
  } catch (Exception &e) {}

  try {
    cfg_query_mode_ = config->get_bool("/hardware/roomba/query_mode");
  } catch (Exception &e) {}

  if (cfg_conntype_ == "rootooth") {
    try {
      cfg_device_ = config->get_string("/hardware/roomba/btaddr");
    } catch (Exception &e) {
      try {
	cfg_device_ = config->get_string("/hardware/roomba/btname");
      } catch (Exception &e2) {
	logger->log_info(name(), "Neither bluetooth name nor address set, "
			 "trying auto-detect");
      }
    }
    try {
      cfg_btfast_ = config->get_bool("/hardware/roomba/btfast");
    } catch (Exception &e) {}

    try {
      cfg_bttype_ = config->get_string("/hardware/roomba/bttype");
    } catch (Exception &e) {
      logger->log_info(name(), "RooTooth type not set, assuming 'firefly'");
    }
    if (cfg_bttype_ == "firefly") {
      // we're cool
    } else if (cfg_bttype_ == "mitsumi") {
      if (cfg_btfast_) {
	logger->log_warn(name(), "Fast mode setting for Mitsumi RooTooth not "
			 "supported, please set outside of Fawkes or wait "
			 "until configuration timeout has passed.");
	cfg_btfast_ = false;
      }
    } else {
      logger->log_warn(name(), "Unknown RooTooth hardware type '%s' set",
		       cfg_bttype_.c_str());
      if (cfg_btfast_) {
	logger->log_warn(name(), "Fast mode setting only supported for "
			 "FireFly RooTooth");
	cfg_btfast_ = false;
      }
    }

    conntype = Roomba500::CONNTYPE_ROOTOOTH;
  } else if (cfg_conntype_ == "serial") {
    cfg_device_ = config->get_string("/hardware/roomba/device");
    conntype = Roomba500::CONNTYPE_SERIAL;
  } else {
    throw Exception("Unknown mode '%s', must be rootooth or serial",
		    cfg_conntype_.c_str());
  }

  try {
    cfg_btsave_ = config->get_bool("/hardware/roomba/btsave");
  } catch (Exception &e) {}

  Roomba500::Mode mode = Roomba500::MODE_PASSIVE;
  cfg_mode_ = "passive";
  try {
    cfg_mode_ = config->get_string("/hardware/roomba/mode");
  } catch (Exception &e) {}
  if (cfg_mode_ == "passive") {
    mode = Roomba500::MODE_PASSIVE;
  } else if (cfg_mode_ == "safe") {
    mode = Roomba500::MODE_SAFE;
  } else if (cfg_mode_ == "full") {
    mode = Roomba500::MODE_FULL;
  } else {
    throw Exception("Unknown mode '%s', must be one of passive, safe, or full",
		    cfg_mode_.c_str());
  }


  try {
    roomba500_if_ = blackboard->open_for_writing<Roomba500Interface>("Roomba 500");
    led_if_debris_ =
      blackboard->open_for_writing<LedInterface>("Roomba LED Debris");
    led_if_spot_ = blackboard->open_for_writing<LedInterface>("Roomba LED Spot");
    led_if_dock_ = blackboard->open_for_writing<LedInterface>("Roomba LED Dock");
    led_if_check_robot_ =
      blackboard->open_for_writing<LedInterface>("Roomba LED Check Robot");
    led_if_clean_color_ =
      blackboard->open_for_writing<LedInterface>("Roomba LED Clean Color");
    led_if_clean_intensity_ =
      blackboard->open_for_writing<LedInterface>("Roomba LED Clean Intensity");
    switch_if_vacuuming_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Vacuuming");
    switch_if_but_clean_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Clean");
    switch_if_but_spot_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Spot");
    switch_if_but_dock_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Dock");
    switch_if_but_minute_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Minute");
    switch_if_but_hour_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Hour");
    switch_if_but_day_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Day");
    switch_if_but_schedule_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Schedule");
    switch_if_but_clock_ =
      blackboard->open_for_writing<SwitchInterface>("Roomba Button Clock");
    //motor_if_ = blackboard->open_for_writing<MotorInterface>("Roomba Motor");
    battery_if_ = blackboard->open_for_writing<BatteryInterface>("Roomba Battery");
  } catch (Exception &e) {
    close_interfaces();
    throw;
  }

  wt_ = NULL;
  try {
    unsigned int flags = 0;
    if (conntype == Roomba500::CONNTYPE_ROOTOOTH) {
      logger->log_debug(name(), "Connecting via RooTooth, this may take a while");
      if (cfg_btfast_) flags |= Roomba500::FLAG_FIREFLY_FASTMODE;
    }
    roomba_ = new Roomba500(conntype, cfg_device_.c_str(), flags);

    if (cfg_btsave_) {
      logger->log_debug(name(), "Saving Bluetooth address %s. Will be used for "
                                "next connection.", roomba_->get_device().c_str());
      config->set_string("/hardware/roomba/btaddr", roomba_->get_device().c_str());
    }

    roomba_->set_mode(mode);
    if (roomba_->is_controlled()) {
      if (cfg_play_fanfare_)  roomba_->play_fanfare();
      roomba_->set_leds(false, false, false, true, 0, 255);
    }
    wt_ = new WorkerThread(logger, clock, roomba_, cfg_query_mode_);
  } catch (Exception &e) {
    close_interfaces();
    roomba_.clear();
    delete wt_;
    throw;
  }
  wt_->start();
}

void
Roomba500Thread::close_interfaces()
{
  blackboard->close(led_if_debris_);
  blackboard->close(led_if_spot_);
  blackboard->close(led_if_dock_);
  blackboard->close(led_if_check_robot_);
  blackboard->close(led_if_clean_color_);
  blackboard->close(led_if_clean_intensity_);
  blackboard->close(switch_if_vacuuming_);
  blackboard->close(switch_if_but_clean_);
  blackboard->close(switch_if_but_spot_);
  blackboard->close(switch_if_but_dock_);
  blackboard->close(switch_if_but_minute_);
  blackboard->close(switch_if_but_hour_);
  blackboard->close(switch_if_but_day_);
  blackboard->close(switch_if_but_schedule_);
  blackboard->close(switch_if_but_clock_);
  //blackboard->close(motor_if_);
  blackboard->close(battery_if_);
  blackboard->close(roomba500_if_);
}


void
Roomba500Thread::finalize()
{
  wt_->cancel();
  wt_->join();
  delete wt_;
  roomba_->set_mode(Roomba500::MODE_PASSIVE);
  roomba_.clear();
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
  float led_debris          = led_process(led_if_debris_);
  float led_spot            = led_process(led_if_spot_);
  float led_dock            = led_process(led_if_dock_);
  float led_check_robot     = led_process(led_if_check_robot_);
  float led_clean_color     = led_process(led_if_clean_color_);
  float led_clean_intensity = led_process(led_if_clean_intensity_);

  if ( (led_debris != led_if_debris_->intensity()) ||
       (led_spot != led_if_spot_->intensity()) ||
       (led_dock != led_if_dock_->intensity()) ||
       (led_check_robot != led_if_check_robot_->intensity()) ||
       (led_clean_color != led_if_clean_color_->intensity()) ||
       (led_clean_intensity != led_if_clean_intensity_->intensity()) )
  {
    try {
      roomba_->set_leds(led_debris > 0.5, led_spot > 0.5,
			 led_dock > 0.5, led_check_robot > 0.5,
			 (char)roundf(led_clean_color * 255.),
			 (char)roundf(led_clean_intensity * 255.));
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to set LEDs, exception follows");
      logger->log_warn(name(), e);
    }

    led_if_debris_->set_intensity(led_debris);
    led_if_spot_->set_intensity(led_spot);
    led_if_dock_->set_intensity(led_dock);
    led_if_check_robot_->set_intensity(led_check_robot);
    led_if_clean_color_->set_intensity(led_clean_color);
    led_if_clean_intensity_->set_intensity(led_clean_intensity);

    led_if_debris_->write();
    led_if_spot_->write();
    led_if_dock_->write();
    led_if_check_robot_->write();
    led_if_clean_color_->write();
    led_if_clean_intensity_->write();
  }

  while (! roomba500_if_->msgq_empty() ) {
    if (roomba500_if_->msgq_first_is<Roomba500Interface::StopMessage>())
    {
      try {
	roomba_->stop();
	//roomba_->set_motors(false, false, false, false, false);
	//logger->log_debug(name(), "Stopped");
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to stop robot, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (roomba500_if_->msgq_first_is<Roomba500Interface::SetModeMessage>())
    {
      Roomba500Interface::SetModeMessage *msg =
	roomba500_if_->msgq_first(msg);

      Roomba500::Mode mode = roomba_->get_mode();
      unsigned char color     =   0;
      unsigned char intensity = 255;

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
	bool was_controlled = roomba_->is_controlled();
	if (! was_controlled) {
	  // set first
	  roomba_->set_mode(mode);
	}
	if (roomba_->is_controlled()) {
	  roomba_->set_leds(led_if_debris_->intensity() >= 0.5,
			     led_if_spot_->intensity() >= 0.5,
			     led_if_dock_->intensity() >= 0.5,
			     led_if_check_robot_->intensity() >= 0.5,
			     color, intensity);
	}
	if (was_controlled) {
	  roomba_->set_mode(mode);
	}
      } catch (Exception &e) {
	logger->log_warn(name(), "Cannot set mode, exception follows");
	logger->log_warn(name(), e);
      }

    } else if (roomba500_if_->msgq_first_is<Roomba500Interface::DockMessage>()) {
      try {
	roomba_->seek_dock();
	logger->log_debug(name(), "Docking");
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to seek dock, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (roomba500_if_->msgq_first_is<Roomba500Interface::DriveStraightMessage>())
    {
      Roomba500Interface::DriveStraightMessage *msg =
	roomba500_if_->msgq_first(msg);

      try {
	roomba_->drive_straight(msg->velocity());
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to drive straight, exception follows");
	logger->log_warn(name(), e);
      }
    } else  if (roomba500_if_->msgq_first_is<Roomba500Interface::DriveMessage>())
    {
      Roomba500Interface::DriveMessage *msg =
	roomba500_if_->msgq_first(msg);

      try {
	roomba_->drive(msg->velocity(), msg->radius());
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to drive, exception follows");
	logger->log_warn(name(), e);
      }

    } else  if (roomba500_if_->msgq_first_is<Roomba500Interface::SetMotorsMessage>())
    {
      Roomba500Interface::SetMotorsMessage *msg =
	roomba500_if_->msgq_first(msg);

      try {
	roomba_->set_motors(
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
    roomba500_if_->msgq_pop();
  }

  if (roomba_->is_controlled()) {
    if (greeting_loop_count_ < 50) {
      if (++greeting_loop_count_ == 50) {
	roomba_->set_leds(false, false, false, false, 0, 0);
      } else {
	roomba_->set_leds(false, false, false, true,
			   0, greeting_loop_count_ * 5);
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
  if (wt_->has_fresh_data()) {
    const Roomba500::SensorPacketGroupAll sp(roomba_->get_sensor_packet());

    int charge = (int)roundf(((float)ntohs(sp.battery_charge) /
			      (float)ntohs(sp.battery_capacity)) * 100.);

    if (roomba_->is_controlled()) {
      if (charge != battery_percent_) {
	char digits[4];
	snprintf(digits, 4, "%d%%", charge);
	roomba_->set_digit_leds(digits);
	battery_percent_ = charge;
      }
    }

    roomba500_if_->set_mode((Roomba500Interface::Mode)sp.mode);
    roomba500_if_->set_wheel_drop_left(
        sp.bumps_wheeldrops & Roomba500::WHEEL_DROP_LEFT);
    roomba500_if_->set_wheel_drop_right(
	sp.bumps_wheeldrops & Roomba500::WHEEL_DROP_RIGHT);
    roomba500_if_->set_bump_left(sp.bumps_wheeldrops & Roomba500::BUMP_LEFT);
    roomba500_if_->set_bump_right(sp.bumps_wheeldrops & Roomba500::BUMP_RIGHT);
    roomba500_if_->set_cliff_left(sp.cliff_left == 1);
    roomba500_if_->set_cliff_front_left(sp.cliff_front_left == 1);
    roomba500_if_->set_cliff_front_right(sp.cliff_front_right == 1);
    roomba500_if_->set_cliff_right(sp.cliff_right == 1);
    roomba500_if_->set_wall(sp.virtual_wall == 1);
    roomba500_if_->set_overcurrent_left_wheel(
        sp.overcurrents & Roomba500::OVERCURRENT_WHEEL_LEFT);
    roomba500_if_->set_overcurrent_right_wheel(
        sp.overcurrents & Roomba500::OVERCURRENT_WHEEL_RIGHT);
    roomba500_if_->set_overcurrent_main_brush(
	sp.overcurrents & Roomba500::OVERCURRENT_MAIN_BRUSH);
    roomba500_if_->set_overcurrent_side_brush(
        sp.overcurrents & Roomba500::OVERCURRENT_SIDE_BRUSH);
    roomba500_if_->set_dirt_detect(sp.dirt_detect == 1);
    roomba500_if_->set_ir_opcode_omni(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_omni);
    roomba500_if_->set_button_clean(sp.buttons & Roomba500::BUTTON_CLEAN);
    roomba500_if_->set_button_spot(sp.buttons & Roomba500::BUTTON_SPOT);
    roomba500_if_->set_button_dock(sp.buttons & Roomba500::BUTTON_DOCK);
    roomba500_if_->set_button_minute(sp.buttons & Roomba500::BUTTON_MINUTE);
    roomba500_if_->set_button_hour(sp.buttons & Roomba500::BUTTON_HOUR);
    roomba500_if_->set_button_day(sp.buttons & Roomba500::BUTTON_DAY);
    roomba500_if_->set_button_schedule(sp.buttons & Roomba500::BUTTON_SCHEDULE);
    roomba500_if_->set_button_clock(sp.buttons & Roomba500::BUTTON_CLOCK);

    switch_if_but_clean_->set_enabled(sp.buttons & Roomba500::BUTTON_CLEAN);
    switch_if_but_spot_->set_enabled(sp.buttons & Roomba500::BUTTON_SPOT);
    switch_if_but_dock_->set_enabled(sp.buttons & Roomba500::BUTTON_DOCK);
    switch_if_but_minute_->set_enabled(sp.buttons & Roomba500::BUTTON_MINUTE);
    switch_if_but_hour_->set_enabled(sp.buttons & Roomba500::BUTTON_HOUR);
    switch_if_but_day_->set_enabled(sp.buttons & Roomba500::BUTTON_DAY);
    switch_if_but_schedule_->set_enabled(sp.buttons & Roomba500::BUTTON_SCHEDULE);
    switch_if_but_clock_->set_enabled(sp.buttons & Roomba500::BUTTON_CLOCK);

    // Convert mm to m for distance
    roomba500_if_->set_distance((int16_t)ntohs(sp.distance));
    // invert because in Fawkes positive angles go counter-clockwise, while
    // for the Roomba they go clockwise
    roomba500_if_->set_angle(- (int16_t)ntohs(sp.angle));
    roomba500_if_->set_charging_state(
	(Roomba500Interface::ChargingState)sp.charging_state);
    roomba500_if_->set_voltage(ntohs(sp.voltage));
    roomba500_if_->set_current((int)ntohs(sp.current));
    roomba500_if_->set_temperature((int)sp.temperature);
    roomba500_if_->set_battery_charge(ntohs(sp.battery_charge));
    roomba500_if_->set_battery_capacity(ntohs(sp.battery_capacity));

    battery_if_->set_voltage(ntohs(sp.voltage));
    battery_if_->set_current((int)ntohs(sp.current));
    battery_if_->set_temperature((char)sp.temperature);
    battery_if_->set_absolute_soc((float)ntohs(sp.battery_charge) /
				   (float)ntohs(sp.battery_capacity));
    battery_if_->set_relative_soc(battery_if_->absolute_soc());

    roomba500_if_->set_wall_signal(ntohs(sp.wall_signal));
    roomba500_if_->set_cliff_left_signal(ntohs(sp.cliff_left_signal));
    roomba500_if_->set_cliff_front_left_signal(ntohs(sp.cliff_front_left_signal));
    roomba500_if_->set_cliff_front_right_signal(ntohs(sp.cliff_front_right_signal));
    roomba500_if_->set_cliff_right_signal(ntohs(sp.cliff_right_signal));
    roomba500_if_->set_home_base_charger_available(
	sp.charger_available & Roomba500::CHARGER_HOME_BASE);
    roomba500_if_->set_internal_charger_available(
	sp.charger_available & Roomba500::CHARGER_INTERNAL);
    roomba500_if_->set_song_number(sp.song_number);
    roomba500_if_->set_song_playing(sp.song_playing == 1);

    roomba500_if_->set_velocity((int16_t)ntohs(sp.velocity));
    roomba500_if_->set_radius((int16_t)ntohs(sp.radius));
    roomba500_if_->set_velocity_right((int16_t)ntohs(sp.velocity_right));
    roomba500_if_->set_velocity_left((int16_t)ntohs(sp.velocity_left));
    roomba500_if_->set_encoder_counts_left(ntohs(sp.encoder_counts_left));
    roomba500_if_->set_encoder_counts_right(ntohs(sp.encoder_counts_right));

    roomba500_if_->set_bumper_left(
	sp.light_bumper & Roomba500::BUMPER_LEFT);
    roomba500_if_->set_bumper_front_left(
	sp.light_bumper & Roomba500::BUMPER_FRONT_LEFT);
    roomba500_if_->set_bumper_center_left(
	sp.light_bumper & Roomba500::BUMPER_CENTER_LEFT);
    roomba500_if_->set_bumper_center_right(
	sp.light_bumper & Roomba500::BUMPER_CENTER_RIGHT);
    roomba500_if_->set_bumper_front_right(
	sp.light_bumper & Roomba500::BUMPER_FRONT_RIGHT);
    roomba500_if_->set_bumper_right(
	sp.light_bumper & Roomba500::BUMPER_RIGHT);

    roomba500_if_->set_light_bump_left(ntohs(sp.light_bump_left));
    roomba500_if_->set_light_bump_front_left(ntohs(sp.light_bump_front_left));
    roomba500_if_->set_light_bump_center_left(ntohs(sp.light_bump_center_left));
    roomba500_if_->set_light_bump_center_right(ntohs(sp.light_bump_center_right));
    roomba500_if_->set_light_bump_front_right(ntohs(sp.light_bump_front_right));
    roomba500_if_->set_light_bump_right(ntohs(sp.light_bump_right));

    roomba500_if_->set_ir_opcode_left(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_left);
    roomba500_if_->set_ir_opcode_right(
	(Roomba500Interface::InfraredCharacter)sp.ir_opcode_right);

    roomba500_if_->set_left_motor_current((int)ntohs(sp.left_motor_current));
    roomba500_if_->set_right_motor_current((int)ntohs(sp.right_motor_current));
    roomba500_if_->set_side_brush_current((int)ntohs(sp.side_brush_current));
    roomba500_if_->set_main_brush_current((int)ntohs(sp.main_brush_current));
    roomba500_if_->set_caster_stasis(sp.stasis == 1);

    roomba500_if_->write();

    switch_if_but_clean_->write();
    switch_if_but_spot_->write();
    switch_if_but_dock_->write();
    switch_if_but_minute_->write();
    switch_if_but_hour_->write();
    switch_if_but_day_->write();
    switch_if_but_schedule_->write();
    switch_if_but_clock_->write();

    battery_if_->write();
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
  unsigned char color     =   0;
  unsigned char intensity = 255;

  switch (mode) {
  case Roomba500::MODE_OFF:     intensity =   0; break;
  case Roomba500::MODE_PASSIVE: color     =   0; break;
  case Roomba500::MODE_SAFE:    color     = 128; break;
  case Roomba500::MODE_FULL:    color     = 255; break;
  }

  roomba_->set_mode(mode);
  roomba_->set_leds(led_if_debris_->intensity() >= 0.5,
		     led_if_spot_->intensity() >= 0.5,
		     led_if_dock_->intensity() >= 0.5,
		     led_if_check_robot_->intensity() >= 0.5,
		     color, intensity);
}
