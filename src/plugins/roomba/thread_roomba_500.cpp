
/***************************************************************************
 *  thread_roomba_500.cpp - Roomba 500 thread
 *
 *  Created: Sun Jan 02 12:47:35 2010
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
#include "roomba_500.h"
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
   */
  WorkerThread(fawkes::Logger *logger, fawkes::Clock *clock,
	       fawkes::RefPtr<Roomba500> roomba)
    : Thread("Roomba500WorkerThread", Thread::OPMODE_CONTINUOUS),
      logger(logger), clock(clock), __roomba(roomba)
  {
    __fresh_data_mutex = new Mutex();
    __time_wait = new TimeWait(clock, Roomba500::STREAM_INTERVAL_MS * 1000);

#ifdef USE_TIMETRACKER
    __tt_count  = 0;
    __ttc_query = __tt.add_class("Query");
    __ttc_loop = __tt.add_class("Loop");
#endif

    __roomba->enable_sensors();
    __roomba->play_fanfare();
  }

  /** Destructor. */
  ~WorkerThread()
  {
    __roomba->disable_sensors();
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
      __roomba->read_sensors();
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
  Clock             *clock;
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
};


/** @class Roomba500Thread "thread_roomba_500.h"
 * Roomba 500 integration thread.
 * This thread integrates the Roomba 500 robot into Fawkes. The thread
 * hooks in at the ACT hook and executes received commands on the hardware.
 * @author Tim Niemueller
 */

/** Constructor. */
Roomba500Thread::Roomba500Thread()
  : Thread("Roomba500:Sensor", Thread::OPMODE_WAITFORWAKEUP),
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

  __cfg_device = config->get_string("/hardware/roomba/device");

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
    __roomba = new Roomba500(__cfg_device.c_str());
    __roomba->set_mode(Roomba500::MODE_SAFE);
    __roomba->set_leds(false, false, false, true, 0, 0);
    __wt = new WorkerThread(logger, clock, __roomba);
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
    __roomba->set_leds(led_debris > 0.5, led_spot > 0.5,
		       led_dock > 0.5, led_check_robot > 0.5,
		       (char)roundf(led_clean_color * 255.),
		       (char)roundf(led_clean_intensity * 255.));

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
      __roomba->stop();
    } else  if (__roomba500_if->msgq_first_is<Roomba500Interface::DriveStraightMessage>()) {
      Roomba500Interface::DriveStraightMessage *msg =
	__roomba500_if->msgq_first(msg);

      __roomba->drive_straight(msg->velocity());
    }
    __roomba500_if->msgq_pop();
  }

  if (__greeting_loop_count < 50) {
    if (++__greeting_loop_count == 50) {
      __roomba->set_leds(false, false, false, false, 0, 0);
    } else {
      __roomba->set_leds(false, false, false, true, 0, __greeting_loop_count * 5);
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
    __roomba500_if->set_distance((int)ntohs(sp.distance) / 1000.);
    // invert because in Fawkes positive angles go counter-clockwise, while
    // for the Roomba they go clockwise, additionally convert into radians.
    __roomba500_if->set_angle(-deg2rad((int)ntohs(sp.angle)));
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
    __battery_if->set_absolute_soc(ntohs(sp.battery_capacity) /
				   ntohs(sp.battery_charge));
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

    __roomba500_if->set_velocity((int)ntohs(sp.velocity) / 1000.);
    __roomba500_if->set_radius((int)ntohs(sp.radius) / 1000.);
    __roomba500_if->set_velocity_right((int)ntohs(sp.velocity_right) / 1000.);
    __roomba500_if->set_velocity_left((int)ntohs(sp.velocity_left) / 1000.);
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
