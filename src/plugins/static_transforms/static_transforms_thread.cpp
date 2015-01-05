
/***************************************************************************
 *  static_transform_thread.cpp - Static transform publisher thread
 *
 *  Created: Tue Oct 25 16:36:04 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *             2014  Tobias Neumann
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

#include "static_transforms_thread.h"

#include <utils/time/time.h>
#include <set>
#include <memory>
#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class StaticTransformsThread "static_transforms_thread.h"
 * Thread to regularly publish static transforms.
 * This thread runs at the sensor hook and publishes a set of
 * transforms. The transforms are set in the configuration and
 * are static at run-time. Only the timestamp is updated between
 * writes.
 * @author Tim Niemueller
 */

#define CFG_PREFIX "/plugins/static-transforms/"

/** Constructor. */
StaticTransformsThread::StaticTransformsThread()
  : Thread("StaticTransformsThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "static"),
    ConfigurationChangeHandler(CFG_PREFIX)
{
  __last_update = new Time();
}


/** Destructor. */
StaticTransformsThread::~StaticTransformsThread()
{
  delete __last_update;
}


void
StaticTransformsThread::init()
{
  entries_get_from_config();

  __last_update->set_clock(clock);
  __last_update->set_time(0, 0);

  config->add_change_handler(this);
}


void
StaticTransformsThread::finalize()
{
  config->rem_change_handler(this);
  entries_delete();
}

void
StaticTransformsThread::entries_get_from_config()
{
  std::set<std::string> transforms;
  std::set<std::string> ignored_transforms;

  __cfg_update_interval = 1.0;
  try  {
    __cfg_update_interval = config->get_float(CFG_PREFIX"update-interval");
  } catch (Exception &e) {} // ignored, use default

  std::string prefix = CFG_PREFIX"transforms/";
  std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (transforms.find(cfg_name) == transforms.end()) &&
	 (ignored_transforms.find(cfg_name) == ignored_transforms.end()) ) {

      std::string cfg_prefix = prefix + cfg_name + "/";
      
      bool active = true;
      try {
        active = config->get_bool((cfg_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      if (active) {
        try {
          std::string frame = config->get_string((cfg_prefix + "frame").c_str());
          std::string child_frame =
            config->get_string((cfg_prefix + "child_frame").c_str());

          float tx = 0., ty = 0., tz = 0.;
          if (config->exists((cfg_prefix + "trans_x").c_str()) ||
              config->exists((cfg_prefix + "trans_y").c_str()) ||
              config->exists((cfg_prefix + "trans_z").c_str()))
          {
            tx = config->get_float((cfg_prefix + "trans_x").c_str());
            ty = config->get_float((cfg_prefix + "trans_y").c_str());
            tz = config->get_float((cfg_prefix + "trans_z").c_str());
          } // else assume no translation

          bool use_quaternion = false;
          float rx = 0., ry = 0., rz = 0., rw = 1.,
            ryaw = 0., rpitch = 0., rroll = 0.;

          if (config->exists((cfg_prefix + "rot_x").c_str()) ||
              config->exists((cfg_prefix + "rot_y").c_str()) ||
              config->exists((cfg_prefix + "rot_z").c_str()) ||
              config->exists((cfg_prefix + "rot_w").c_str()) )
          {
            use_quaternion = true;
            rx = config->get_float((cfg_prefix + "rot_x").c_str());
            ry = config->get_float((cfg_prefix + "rot_y").c_str());
            rz = config->get_float((cfg_prefix + "rot_z").c_str());
            rw = config->get_float((cfg_prefix + "rot_w").c_str());

          } else if (config->exists((cfg_prefix + "rot_roll").c_str()) ||
                     config->exists((cfg_prefix + "rot_pitch").c_str()) ||
                     config->exists((cfg_prefix + "rot_yaw").c_str()))
          {
            ryaw   = config->get_float((cfg_prefix + "rot_yaw").c_str());
            rpitch = config->get_float((cfg_prefix + "rot_pitch").c_str());
            rroll  = config->get_float((cfg_prefix + "rot_roll").c_str());
          } // else assume no rotation

          if (frame == child_frame) {
            throw Exception("Parent and child frames may not be the same");
          }

          try {
            Entry e;
            e.name = cfg_name;

            fawkes::Time time(clock);
            if (use_quaternion) {
              tf::Quaternion q(rx, ry, rz, rw);
              tf::assert_quaternion_valid(q);
              tf::Transform t(q, tf::Vector3(tx, ty, tz));
              e.transform = new tf::StampedTransform(t, time, frame, child_frame);
            } else {
              tf::Quaternion q; q.setEulerZYX(ryaw, rpitch, rroll);
              tf::Transform t(q, tf::Vector3(tx, ty, tz));
              e.transform = new tf::StampedTransform(t, time, frame, child_frame);
            }

            tf::Quaternion q = e.transform->getRotation();

            tf::assert_quaternion_valid(q);

            tf::Vector3 &v = e.transform->getOrigin();
            logger->log_debug(name(), "Adding transform '%s' (%s -> %s): "
                              "T(%f,%f,%f)  Q(%f,%f,%f,%f)", e.name.c_str(),
                              e.transform->frame_id.c_str(),
                              e.transform->child_frame_id.c_str(),
                              v.x(), v.y(), v.z(), q.x(), q.y(), q.z(), q.w());

            __entries.push_back(e);
          } catch (Exception &e) {
            entries_delete();
            throw;
          }
          
        } catch (Exception &e) {
          e.prepend("Transform %s: wrong or incomplete transform data", cfg_name.c_str());
          throw;
        }
        
        transforms.insert(cfg_name);
      } else {
        //printf("Ignoring laser config %s\n", cfg_name.c_str());
        ignored_transforms.insert(cfg_name);
      }
    }
  }
  
  if ( __entries.empty() ) {
    throw Exception("No transforms configured");
  }
}

void
StaticTransformsThread::entries_delete()
{
  std::list<Entry>::iterator i;
  for (i = __entries.begin(); i != __entries.end(); ++i) {
    delete i->transform;
  }
  __entries.clear();
}

void
StaticTransformsThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
  MutexLocker lock(loop_mutex);

  entries_delete();
  entries_get_from_config();
}

void
StaticTransformsThread::loop()
{
  fawkes::Time now(clock);
  if ((now - __last_update) > __cfg_update_interval) {
    __last_update->stamp();

    // date time stamps slightly into the future so they are valid
    // for longer and need less frequent updates.
    fawkes::Time timestamp = now + (__cfg_update_interval * 1.1);

    std::list<Entry>::iterator i;
    for (i = __entries.begin(); i != __entries.end(); ++i) {
      i->transform->stamp = timestamp;
      tf_publisher->send_transform(*(i->transform));
    }
  }
}
