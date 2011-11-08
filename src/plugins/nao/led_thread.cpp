
/***************************************************************************
 *  led_thread.cpp - Provide NaoQi LEDs to Fawkes
 *
 *  Created: Thu Jun 30 19:52:00 2011
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

#include "led_thread.h"
#include "dcm_utils.h"

#include <utils/system/pathparser.h>

#include <alproxies/allauncherproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcore/alerror.h>
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <interfaces/LedInterface.h>
#include <interfaces/NaoJointPositionInterface.h>

#include <cmath>

using namespace fawkes;

enum LedType {
  LED_CHESTBOARD_RED, LED_CHESTBOARD_GREEN, LED_CHESTBOARD_BLUE,
  LED_EARS_LEFT_0DEG, LED_EARS_LEFT_36DEG, LED_EARS_LEFT_72DEG,
  LED_EARS_LEFT_108DEG, LED_EARS_LEFT_144DEG, LED_EARS_LEFT_180DEG,
  LED_EARS_LEFT_216DEG, LED_EARS_LEFT_252DEG, LED_EARS_LEFT_288DEG,
  LED_EARS_LEFT_324DEG,
  LED_EARS_RIGHT_0DEG, LED_EARS_RIGHT_36DEG, LED_EARS_RIGHT_72DEG,
  LED_EARS_RIGHT_108DEG, LED_EARS_RIGHT_144DEG, LED_EARS_RIGHT_180DEG,
  LED_EARS_RIGHT_216DEG, LED_EARS_RIGHT_252DEG, LED_EARS_RIGHT_288DEG,
  LED_EARS_RIGHT_324DEG,
  LED_FACE_LEFT_RED_0DEG, LED_FACE_LEFT_RED_45DEG,
  LED_FACE_LEFT_RED_90DEG, LED_FACE_LEFT_RED_135DEG, LED_FACE_LEFT_RED_180DEG,
  LED_FACE_LEFT_RED_225DEG, LED_FACE_LEFT_RED_270DEG, LED_FACE_LEFT_RED_315DEG,
  LED_FACE_LEFT_GREEN_0DEG, LED_FACE_LEFT_GREEN_45DEG, LED_FACE_LEFT_GREEN_90DEG,
  LED_FACE_LEFT_GREEN_135DEG, LED_FACE_LEFT_GREEN_180DEG,
  LED_FACE_LEFT_GREEN_225DEG, LED_FACE_LEFT_GREEN_270DEG,
  LED_FACE_LEFT_GREEN_315DEG,
  LED_FACE_LEFT_BLUE_0DEG, LED_FACE_LEFT_BLUE_45DEG,
  LED_FACE_LEFT_BLUE_90DEG, LED_FACE_LEFT_BLUE_135DEG,
  LED_FACE_LEFT_BLUE_180DEG, LED_FACE_LEFT_BLUE_225DEG,
  LED_FACE_LEFT_BLUE_270DEG, LED_FACE_LEFT_BLUE_315DEG,
  LED_FACE_RIGHT_RED_0DEG, LED_FACE_RIGHT_RED_45DEG, LED_FACE_RIGHT_RED_90DEG,
  LED_FACE_RIGHT_RED_135DEG, LED_FACE_RIGHT_RED_180DEG, LED_FACE_RIGHT_RED_225DEG,
  LED_FACE_RIGHT_RED_270DEG, LED_FACE_RIGHT_RED_315DEG,
  LED_FACE_RIGHT_GREEN_0DEG, LED_FACE_RIGHT_GREEN_45DEG, LED_FACE_RIGHT_GREEN_90DEG,
  LED_FACE_RIGHT_GREEN_135DEG, LED_FACE_RIGHT_GREEN_180DEG,
  LED_FACE_RIGHT_GREEN_225DEG,
  LED_FACE_RIGHT_GREEN_270DEG, LED_FACE_RIGHT_GREEN_315DEG,
  LED_FACE_RIGHT_BLUE_0DEG, LED_FACE_RIGHT_BLUE_45DEG,
  LED_FACE_RIGHT_BLUE_90DEG, LED_FACE_RIGHT_BLUE_135DEG,
  LED_FACE_RIGHT_BLUE_180DEG, LED_FACE_RIGHT_BLUE_225DEG,
  LED_FACE_RIGHT_BLUE_270DEG, LED_FACE_RIGHT_BLUE_315DEG,
  LED_LFOOT_RED, LED_LFOOT_GREEN, LED_LFOOT_BLUE,
  LED_RFOOT_RED, LED_RFOOT_GREEN, LED_RFOOT_BLUE,
  LedTypeN
};

/** @class NaoQiLedThread "led_thread.h"
 * Thread to synchronize with LEDs.
 * This thread registered for data changed events to a specified LED and
 * updates the blackboard interface. It also processes actuation commands.
 * if there is a reader for any of the high frequency interfaces. It
 * is also responsible for processing incoming commands.
 */


 /** Constructor. */
NaoQiLedThread::NaoQiLedThread()
  : Thread("NaoQiLedThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    BlackBoardInterfaceListener("NaoQiLedThread")
{
}


/** Destructor. */
NaoQiLedThread::~NaoQiLedThread()
{
}

void
NaoQiLedThread::init()
{
  __cfg_verbose_face = false;
  try {
    __cfg_verbose_face = config->get_bool("/hardware/nao/leds/verbose_face");
  } catch (Exception &e) {} // ignored, use default

  __dcm   = naoqi_broker->getDcmProxy();
  __almem = naoqi_broker->getMemoryProxy();

  try {
    __subd_prefix = (std::string)__dcm->getPrefix()[0];
  } catch (AL::ALError &e) {
    throw Exception("Failed to get DCM prefix: %s", e.toString().c_str());
  }
  PathParser subdpp(__subd_prefix);

  std::vector<std::string> leddevs;
  try {
    leddevs = dcm::get_devices(__dcm, __almem, "Led");
  } catch (AL::ALError &e) {
    throw Exception("Failed to get LED devices: %s", e.toString().c_str());
  }

  // Initialize fast memory access
  std::string prefix = __subd_prefix;
  std::vector<std::string> keys;
  keys.resize(LedTypeN);
  __values.resize(LedTypeN);

  keys[LED_CHESTBOARD_RED] = prefix + "ChestBoard/Led/Red/Actuator/Value";
  keys[LED_CHESTBOARD_GREEN] = prefix + "ChestBoard/Led/Green/Actuator/Value";
  keys[LED_CHESTBOARD_BLUE] = prefix + "ChestBoard/Led/Blue/Actuator/Value";

  prefix = __subd_prefix + "Ears/Led/";
  keys[LED_EARS_LEFT_0DEG] = prefix + "Left/0Deg/Actuator/Value";
  keys[LED_EARS_LEFT_36DEG] = prefix + "Left/36Deg/Actuator/Value";
  keys[LED_EARS_LEFT_72DEG] = prefix + "Left/72Deg/Actuator/Value";
  keys[LED_EARS_LEFT_108DEG] = prefix + "Left/108Deg/Actuator/Value";
  keys[LED_EARS_LEFT_144DEG] = prefix + "Left/144Deg/Actuator/Value";
  keys[LED_EARS_LEFT_180DEG] = prefix + "Left/180Deg/Actuator/Value";
  keys[LED_EARS_LEFT_216DEG] = prefix + "Left/216Deg/Actuator/Value";
  keys[LED_EARS_LEFT_252DEG] = prefix + "Left/252Deg/Actuator/Value";
  keys[LED_EARS_LEFT_288DEG] = prefix + "Left/288Deg/Actuator/Value";
  keys[LED_EARS_LEFT_324DEG] = prefix + "Left/324Deg/Actuator/Value";

  keys[LED_EARS_RIGHT_0DEG] = prefix + "Right/0Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_36DEG] = prefix + "Right/36Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_72DEG] = prefix + "Right/72Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_108DEG] = prefix + "Right/108Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_144DEG] = prefix + "Right/144Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_180DEG] = prefix + "Right/180Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_216DEG] = prefix + "Right/216Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_252DEG] = prefix + "Right/252Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_288DEG] = prefix + "Right/288Deg/Actuator/Value";
  keys[LED_EARS_RIGHT_324DEG] = prefix + "Right/324Deg/Actuator/Value";

  prefix = __subd_prefix + "Face/Led/";
  keys[LED_FACE_LEFT_RED_0DEG] = prefix + "Red/Left/0Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_45DEG] = prefix + "Red/Left/45Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_90DEG] = prefix + "Red/Left/90Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_135DEG] = prefix + "Red/Left/135Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_180DEG] = prefix + "Red/Left/180Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_225DEG] = prefix + "Red/Left/225Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_270DEG] = prefix + "Red/Left/270Deg/Actuator/Value";
  keys[LED_FACE_LEFT_RED_315DEG] = prefix + "Red/Left/315Deg/Actuator/Value";

  keys[LED_FACE_LEFT_GREEN_0DEG] = prefix + "Green/Left/0Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_45DEG] = prefix + "Green/Left/45Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_90DEG] = prefix + "Green/Left/90Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_135DEG] = prefix + "Green/Left/135Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_180DEG] = prefix + "Green/Left/180Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_225DEG] = prefix + "Green/Left/225Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_270DEG] = prefix + "Green/Left/270Deg/Actuator/Value";
  keys[LED_FACE_LEFT_GREEN_315DEG] = prefix + "Green/Left/315Deg/Actuator/Value";

  keys[LED_FACE_LEFT_BLUE_0DEG] = prefix + "Blue/Left/0Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_45DEG] = prefix + "Blue/Left/45Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_90DEG] = prefix + "Blue/Left/90Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_135DEG] = prefix + "Blue/Left/135Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_180DEG] = prefix + "Blue/Left/180Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_225DEG] = prefix + "Blue/Left/225Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_270DEG] = prefix + "Blue/Left/270Deg/Actuator/Value";
  keys[LED_FACE_LEFT_BLUE_315DEG] = prefix + "Blue/Left/315Deg/Actuator/Value";


  keys[LED_FACE_RIGHT_RED_0DEG] = prefix + "Red/Right/0Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_45DEG] = prefix + "Red/Right/45Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_90DEG] = prefix + "Red/Right/90Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_135DEG] = prefix + "Red/Right/135Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_180DEG] = prefix + "Red/Right/180Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_225DEG] = prefix + "Red/Right/225Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_270DEG] = prefix + "Red/Right/270Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_RED_315DEG] = prefix + "Red/Right/315Deg/Actuator/Value";

  keys[LED_FACE_RIGHT_GREEN_0DEG] = prefix + "Green/Right/0Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_45DEG] = prefix + "Green/Right/45Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_90DEG] = prefix + "Green/Right/90Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_135DEG] = prefix + "Green/Right/135Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_180DEG] = prefix + "Green/Right/180Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_225DEG] = prefix + "Green/Right/225Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_270DEG] = prefix + "Green/Right/270Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_GREEN_315DEG] = prefix + "Green/Right/315Deg/Actuator/Value";

  keys[LED_FACE_RIGHT_BLUE_0DEG] = prefix + "Blue/Right/0Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_45DEG] = prefix + "Blue/Right/45Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_90DEG] = prefix + "Blue/Right/90Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_135DEG] = prefix + "Blue/Right/135Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_180DEG] = prefix + "Blue/Right/180Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_225DEG] = prefix + "Blue/Right/225Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_270DEG] = prefix + "Blue/Right/270Deg/Actuator/Value";
  keys[LED_FACE_RIGHT_BLUE_315DEG] = prefix + "Blue/Right/315Deg/Actuator/Value";


  prefix = __subd_prefix;
  keys[LED_LFOOT_RED] = prefix + "LFoot/Led/Red/Actuator/Value";
  keys[LED_LFOOT_GREEN] = prefix + "LFoot/Led/Green/Actuator/Value";
  keys[LED_LFOOT_BLUE] = prefix + "LFoot/Led/Blue/Actuator/Value";

  keys[LED_RFOOT_RED] = prefix + "RFoot/Led/Red/Actuator/Value";
  keys[LED_RFOOT_GREEN] = prefix + "RFoot/Led/Green/Actuator/Value";
  keys[LED_RFOOT_BLUE] = prefix + "RFoot/Led/Blue/Actuator/Value";

  __memfa.reset(new AL::ALMemoryFastAccess());
  try {
    __memfa->ConnectToVariables(naoqi_broker, keys, false);
  } catch (AL::ALError &e) {
    throw Exception("Failed to setup fast memory access: %s",
		    e.toString().c_str());
  }

  NaoJointPositionInterface *joint_pos_if =
    blackboard->open_for_reading<NaoJointPositionInterface>("Nao Joint Positions");
  if (! joint_pos_if->has_writer()) {
    blackboard->close(joint_pos_if);
    throw Exception("Joint Position interface has no writer");
  }
  joint_pos_if->read();
  bool skip_head_leds =
    (joint_pos_if->robot_type() != NaoJointPositionInterface::ROBOTYPE_ACADEMIC);
  blackboard->close(joint_pos_if);

  std::vector<std::string>::iterator l;
  for (l = leddevs.begin(); l != leddevs.end(); ++l) {
    PathParser pp(*l);
    std::string loc = pp[subdpp.size()];

    if (! __cfg_verbose_face) {
      PathParser locpp(loc);
      if (locpp[0] == "Face")  continue;
    }
    if (skip_head_leds) {
      PathParser locpp(loc);
      if (locpp[0] == "Head")  continue;
    }

    std::string id = "Nao LED " + loc;
    PathParser::size_type i;
    for (i = subdpp.size() + 2; (i < pp.size()) && (pp[i] != "Actuator"); ++i) {
      id += "/";
      id += pp[i];
    }

    try {
      LedInterface *iface =
        blackboard->open_for_writing<LedInterface>(id.c_str());
      __leds.insert(make_pair(iface, *l + "/Value"));
    } catch (Exception &e) {
      fawkes::LedInterface *last = NULL;
      for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
        if (i->first != last) {
          blackboard->close(i->first);
          last = i->first;
        }
      }
      __leds.clear();
      throw;
    }
  }

  try {
    std::string left_right[2] = { "Left", "Right" };
    std::string rgb[3] = { "Red", "Green", "Blue" };
    std::string angles[8] = {"0", "45", "90", "135", "180", "225", "270", "315"};

    for (unsigned int lr = 0; lr < 2; ++lr) {
      for (unsigned int cl = 0; cl < 3; ++cl) {
        std::string id = "Nao LED Face/" + rgb[cl] + "/" + left_right[lr];
        LedInterface *iface =
          blackboard->open_for_writing<LedInterface>(id.c_str());

        for (unsigned int a = 0; a < 8; ++a) {
          std::string entry = "Face/Led/" + rgb[cl] + "/" + left_right[lr];
          std::string memid =
            __subd_prefix + entry + "/" + angles[a] + "Deg/Actuator/Value";

          __leds.insert(make_pair(iface, memid));
        }
      }
    }

  } catch (Exception &e) {
    fawkes::LedInterface *last = NULL;
    for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
      if (i->first != last) {
        blackboard->close(i->first);
        last = i->first;
      }
    }
    __leds.clear();
    throw;
  }

  //logger->log_debug(name(), "Interfaces and device IDs:");
  fawkes::LedInterface *last = NULL;
  for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
    if (i->first == last)  continue;

    //logger->log_debug(name(), "  %s", i->first->id());
    std::pair<LedMap::iterator, LedMap::iterator> ret =
      __leds.equal_range(i->first);
      
    for (LedMap::iterator j = ret.first; j != ret.second; ++j) {
      //logger->log_debug(name(), "    %s", j->second.c_str());

      for (unsigned int k = 0; k < keys.size(); ++k) {
        if (keys[k] == j->second) {
          __memids.insert(std::make_pair(i->first, k));
          break;
        }
      }

      last = i->first;
    }
  }

  last = NULL;
  for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
    if (i->first != last) {
      bbil_add_message_interface(i->first);
      last = i->first;
    }
  }
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}

void
NaoQiLedThread::finalize()
{
  blackboard->unregister_listener(this);

  fawkes::LedInterface *last = NULL;
  for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
    if (i->first != last) {
      blackboard->close(i->first);
      last = i->first;
    }
  }

  __dcm.reset();
  __almem.reset();
  __memfa.reset();
}

void
NaoQiLedThread::loop()
{
  __memfa->GetValues(__values);

  fawkes::LedInterface *last = NULL;
  for (LedMap::iterator i = __leds.begin(); i != __leds.end(); ++i) {
    if (i->first == last)  continue;

    float maxval = 0.;

    std::pair<LedMemMap::iterator, LedMemMap::iterator> ret =
      __memids.equal_range(i->first);
    for (LedMemMap::iterator j = ret.first; j != ret.second; ++j) {
      if (__values[j->second] > maxval)  maxval = __values[j->second];
    }

    if (maxval != i->first->intensity()) {
      i->first->set_intensity(maxval);
      i->first->write();
    }

    last = i->first;
  }
}


bool
NaoQiLedThread::bb_interface_message_received(Interface *interface,
                                              Message *message) throw()
{
  // some string magic to find the correct ALValue to write to
  std::string kind = "Merge";
  int dcm_time = __dcm->getTime(0);

  LedInterface::SetIntensityMessage *sim =
    dynamic_cast<LedInterface::SetIntensityMessage *>(message);

  LedInterface *led_if = dynamic_cast<LedInterface *>(interface);
  if (led_if == NULL) return false;

  std::pair<LedMap::iterator, LedMap::iterator> ret =
    __leds.equal_range(led_if);

  if (sim != NULL) {
    for (LedMap::iterator i = ret.first; i != ret.second; ++i) {
      printf("Set %s to %f\n", i->second.c_str(), sim->intensity());
      dcm::set_value(__dcm, i->second, kind, sim->intensity(),
                     (int)roundf(dcm_time + sim->time_sec() * 1000.));
    }
  } else if (dynamic_cast<LedInterface::TurnOnMessage *>(message) != NULL) {
    for (LedMap::iterator i = ret.first; i != ret.second; ++i) {
      dcm::set_value(__dcm, i->second, kind, 1., dcm_time);
    }
  } else if (dynamic_cast<LedInterface::TurnOffMessage *>(message) != NULL) {
    for (LedMap::iterator i = ret.first; i != ret.second; ++i) {
      dcm::set_value(__dcm, i->second, kind, 0., dcm_time);
    }
  }

  return false;
}
