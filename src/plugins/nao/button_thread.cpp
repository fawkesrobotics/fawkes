
/***************************************************************************
 *  button_thread.cpp - Provide Nao buttons to Fawkes
 *
 *  Created: Mon Aug 15 11:02:49 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "button_thread.h"

#include <alproxies/allauncherproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/alaudioplayerproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsentinelproxy.h>
#include <alcore/alerror.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <interfaces/NaoSensorInterface.h>
#include <interfaces/SwitchInterface.h>

#include <boost/bind.hpp>
#include <cstring>
#include <cerrno>
#include <sys/stat.h>

using namespace fawkes;

#define POWEROFF_PATH "/sbin/poweroff"


/** @class NaoQiButtonThread "dcm_thread.h"
 * Thread to provide buttons to Fawkes.
 * This thread reads the sensors data from the DCM thread and
 * processes the included button data. From that it derives short and
 * long activations as well as a basic pattern (three times long
 * press) to turn off the robot.  It also plays audio samples based on
 * the activations.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
NaoQiButtonThread::NaoQiButtonThread()
  : Thread("NaoQiButtonThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


/** Destructor. */
NaoQiButtonThread::~NaoQiButtonThread()
{
}


void
NaoQiButtonThread::init()
{
  sound_longpling_ = sound_pling_ = -1;
  sound_bumper_left_ = sound_bumper_right_ = -1;
  last_shutdown_actcount = 0;

  cfg_chest_triple_long_click_shutdown_ = false;
  try {
    cfg_chest_triple_long_click_shutdown_ =
      config->get_bool("/hardware/nao/chestbut_triple_long_click_shutdown");
  } catch (Exception &e) {} // ignored

  // Is the audio player loaded?
  try {
    AL::ALPtr<AL::ALLauncherProxy> launcher(new AL::ALLauncherProxy(naoqi_broker));
    bool is_auplayer_available = launcher->isModulePresent("ALAudioPlayer");
    bool is_alsentinel_available = launcher->isModulePresent("ALSentinel");

    if (! is_auplayer_available) {
      logger->log_warn(name(), "ALAudioPlayer not available, disabling sounds");
    } else {
      auplayer_ =
        AL::ALPtr<AL::ALAudioPlayerProxy>(new AL::ALAudioPlayerProxy(naoqi_broker));
      sound_longpling_ = auplayer_->loadFile(RESDIR"/sounds/longpling.wav");
      sound_pling_ = auplayer_->loadFile(RESDIR"/sounds/pling.wav");
      sound_bumper_left_ =
        auplayer_->loadFile(RESDIR"/sounds/metal_click_1_left.wav");
      sound_bumper_right_ =
        auplayer_->loadFile(RESDIR"/sounds/metal_click_1_right.wav");
    }

    if (is_alsentinel_available) {
      logger->log_warn(name(), "ALSentinel loaded, disabling its button handling");
      AL::ALPtr<AL::ALSentinelProxy>
        alsentinel(new AL::ALSentinelProxy(naoqi_broker));
      alsentinel->enableDefaultActionSimpleClick(false);
      alsentinel->enableDefaultActionDoubleClick(false);
      alsentinel->enableDefaultActionTripleClick(false);
    }
  } catch (AL::ALError& e) {
    throw Exception("Checking module availability failed: %s",
		    e.toString().c_str());
  }

  sensor_if_ =
    blackboard->open_for_reading<NaoSensorInterface>("Nao Sensors");

  chestbut_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Chest");
  lfoot_bumper_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Foot Left");
  rfoot_bumper_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Foot Right");
  head_front_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Head Front");
  head_middle_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Head Middle");
  head_rear_if_ =
    blackboard->open_for_writing<SwitchInterface>("Nao Button Head Rear");

  chestbut_if_->resize_buffers(1);
  lfoot_bumper_if_->resize_buffers(1);
  rfoot_bumper_if_->resize_buffers(1);
  head_front_if_->resize_buffers(1);
  head_middle_if_->resize_buffers(1);
  head_rear_if_->resize_buffers(1);

  chestbut_remote_enabled_ = false;
  lfoot_bumper_remote_enabled_ = rfoot_bumper_remote_enabled_ = false;
  head_front_remote_enabled_ = head_middle_remote_enabled_ =
    head_rear_remote_enabled_ = false;

  now.set_clock(clock);
  last.set_clock(clock);
  now.stamp();
  last.stamp();
}


void
NaoQiButtonThread::finalize()
{
  blackboard->close(chestbut_if_);
  blackboard->close(lfoot_bumper_if_);
  blackboard->close(rfoot_bumper_if_);
  blackboard->close(head_front_if_);
  blackboard->close(head_middle_if_);
  blackboard->close(head_rear_if_);
  blackboard->close(sensor_if_);
  chestbut_if_ = NULL;
  lfoot_bumper_if_ = NULL;
  rfoot_bumper_if_ = NULL;
  head_front_if_ = NULL;
  head_middle_if_ = NULL;
  head_rear_if_ = NULL;
  sensor_if_ = NULL;

  if (auplayer_) {
    auplayer_->unloadFile(sound_longpling_);
    auplayer_->unloadFile(sound_pling_);
    auplayer_->unloadFile(sound_bumper_left_);
    auplayer_->unloadFile(sound_bumper_right_);
    auplayer_.reset();
  }
}



void
NaoQiButtonThread::loop()
{
  now.stamp();
  float time_diff_sec = now - &last;
  last = now;

  sensor_if_->read();

  process_pattern_button(chestbut_if_, sensor_if_->chest_button(),
                         time_diff_sec, chestbut_remote_enabled_,
                         sound_pling_, sound_longpling_);
  process_pattern_button(head_front_if_, sensor_if_->head_touch_front(),
                         time_diff_sec, head_front_remote_enabled_);
  process_pattern_button(head_middle_if_, sensor_if_->head_touch_middle(),
                         time_diff_sec, head_middle_remote_enabled_);
  process_pattern_button(head_rear_if_, sensor_if_->head_touch_rear(),
                         time_diff_sec, head_rear_remote_enabled_);

  process_bumpers(lfoot_bumper_if_, sensor_if_->l_foot_bumper_l(),
                  sensor_if_->l_foot_bumper_r(),  time_diff_sec,
                  lfoot_bumper_remote_enabled_, sound_bumper_left_);

  process_bumpers(rfoot_bumper_if_, sensor_if_->r_foot_bumper_l(),
                  sensor_if_->r_foot_bumper_r(),  time_diff_sec,
                  rfoot_bumper_remote_enabled_, sound_bumper_right_);

  if (cfg_chest_triple_long_click_shutdown_ &&
      chestbut_if_->long_activations() == 3 &&
      chestbut_if_->activation_count() != last_shutdown_actcount)
  {
    logger->log_debug(name(), "Shutting down");
    last_shutdown_actcount = chestbut_if_->activation_count();
    if (auplayer_)  auplayer_->playFile(RESDIR"/sounds/naoshutdown.wav");

    struct stat s;
    if (stat(POWEROFF_PATH, &s) == -1) {
      logger->log_error(name(), "Cannot stat '%s': %s", POWEROFF_PATH,
                        strerror(errno));
    } else {
      if (s.st_mode & S_ISUID) {
        int rv = system(POWEROFF_PATH);
        if (rv == -1 || (WEXITSTATUS(rv) != 0)) {
          logger->log_error(name(), "Failed to execute shutdown command");
        }
      } else {
        logger->log_error(name(), "SetUID bit on '%s' not set, cannot shutdown.",
                          POWEROFF_PATH);
      }
    }
  }
}


void
NaoQiButtonThread::set_interface(SwitchInterface *switch_if,
                                 bool enabled, float value, float history,
                                 unsigned int activations,
                                 unsigned int short_act, unsigned int long_act)
{
  switch_if->copy_shared_to_buffer(0);

  switch_if->set_enabled(enabled);
  switch_if->set_value(value);
  switch_if->set_history(history);
  switch_if->set_activation_count(activations);
  switch_if->set_short_activations(short_act);
  switch_if->set_long_activations(long_act);

  if (switch_if->compare_buffers(0) != 0)  switch_if->write();
}


void
NaoQiButtonThread::process_pattern_button(SwitchInterface *switch_if,
                                          float sensor_value, float time_diff_sec,
                                          bool &remote_enabled,
                                          int sound_short, int sound_long)
{
  float value = 0;
  process_messages(switch_if, remote_enabled, value);
  value = std::max(value, sensor_value);

  bool enabled = false;
  float history = switch_if->history();
  unsigned int activations = switch_if->activation_count();
  unsigned int short_act = switch_if->short_activations();
  unsigned int long_act =  switch_if->long_activations();

  pattern_button_logic(value, time_diff_sec, enabled, history,
                       activations, short_act, long_act,
                       sound_short, sound_long);

  set_interface(switch_if, enabled, value, history,
                activations, short_act, long_act);
}


void
NaoQiButtonThread::process_bumpers(SwitchInterface *switch_if,
                                   float left_value, float right_value,
                                   float time_diff_sec,
                                   bool &remote_enabled, int sound_id)
{
  float value = 0;
  process_messages(switch_if, remote_enabled, value);
  value = std::max(std::max(value, left_value), right_value);

  bool enabled = false;
  float history = switch_if->history();
  unsigned int activations = switch_if->activation_count();
  unsigned int short_act = switch_if->short_activations();
  unsigned int long_act =  switch_if->long_activations();

  bumpers_logic(value, time_diff_sec, enabled, history, activations, sound_id);

  set_interface(switch_if, enabled, value, history,
                activations, short_act, long_act);
}



void
NaoQiButtonThread::process_messages(SwitchInterface *switch_if,
                                    bool &remote_enabled, float &value)
{
  while ( ! switch_if->msgq_empty() ) {

    if (SwitchInterface::SetMessage *msg = switch_if->msgq_first_safe(msg)) {
      if (msg->is_enabled()) {
        value = std::min(0.5f, msg->value());
      } else {
        value = std::max(0.49f, msg->value());
      }

    } else if (SwitchInterface::EnableSwitchMessage *msg =
               switch_if->msgq_first_safe(msg))
    {
      logger->log_debug(name(), "Got ENable switch message for %s",
                        switch_if->id());
      value = 1.;
      remote_enabled = true;

    } else if (SwitchInterface::DisableSwitchMessage *msg =
               switch_if->msgq_first_safe(msg))
    {
      logger->log_debug(name(), "Got DISable switch message for %s",
                        switch_if->id());
      remote_enabled = false;
      value = 0.;
    }

    switch_if->msgq_pop();
  }

  if (remote_enabled)  value = 1.;
}


void
NaoQiButtonThread::pattern_button_logic(float value, float time_diff_sec,
                                        bool &enabled, float &history,
                                        unsigned int &activations,
                                        unsigned int &short_act,
                                        unsigned int &long_act,
                                        int sound_short, int sound_long)
{
  if (value < 0.5) { // button released or not pressed
    if (history > 0.025 /* sec */) { // released
      ++activations;
      if (history > 0.5) {
	++long_act;
        if (auplayer_ && (sound_long != -1))   auplayer_->play(sound_long);
      } else {
	++short_act;
        if (auplayer_ && (sound_short != -1))  auplayer_->play(sound_short);
      }
    } else if ( history < -2.0 /* sec */ ) {
      // reset after two seconds
      short_act = long_act = 0;
    }

    enabled = false;
    if ( history < 0. ) {
      history -=   time_diff_sec;
    } else {
      history  = - time_diff_sec;
    }

  } else {  // at least one is enabled
    enabled = true;
    if ( history > 0. ) {
      history += time_diff_sec;
    } else {
      history  = time_diff_sec;
    }
  }

  // stop after two minutes, nobody cares after that
  if (history < -120.) {
    history = -120.;
  } else if (history > 120.) {
    history =  120.;
  }
}


void
NaoQiButtonThread::bumpers_logic(float value, float time_diff_sec,
                                 bool &enabled, float &history,
                                 unsigned int &activations, int sound_id)
{
  if (value < 0.5) { // button released or none pressed
    enabled = false;
    if ( history < 0. ) {
      history -=   time_diff_sec;
    } else {
      history  = - time_diff_sec;
    }

  } else {  // at least one is enabled
    if (history <= 0. /* sec */) { // pressed
      if (auplayer_ && (sound_id != -1))  auplayer_->play(sound_id);
      ++activations;
    }

    enabled = true;
    if ( history > 0. ) {
      history += time_diff_sec;
    } else {
      history  = time_diff_sec;
    }
  }

  // stop after two minutes, nobody cares after that
  if (history < -120.) {
    history = -120.;
  } else if (history > 120.) {
    history =  120.;
  }
}
