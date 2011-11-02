
/***************************************************************************
 *  static_transform_thread.cpp - Static transform publisher thread
 *
 *  Created: Tue Oct 25 16:36:04 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <set>
#include <memory>

using namespace fawkes;

/** @class StaticTransformsThread "static_transforms_thread.h"
 * Thread to regularly publish static transforms.
 * This thread runs at the sensor hook and publishes a set of
 * transforms. The transforms are set in the configuration and
 * are static at run-time. Only the timestamp is updated between
 * writes.
 * @author Tim Niemueller
 */

/** Constructor. */
StaticTransformsThread::StaticTransformsThread()
  : Thread("StaticTransformsThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "static")
{
}


/** Destructor. */
StaticTransformsThread::~StaticTransformsThread()
{
}


void
StaticTransformsThread::init()
{
  std::set<std::string> transforms;
  std::set<std::string> ignored_transforms;

  std::string prefix = "/plugins/static_transforms/";

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
          float tx = config->get_float((cfg_prefix + "trans_x").c_str());
          float ty = config->get_float((cfg_prefix + "trans_y").c_str());
          float tz = config->get_float((cfg_prefix + "trans_z").c_str());

          bool use_quaternion = true;
          float rx, ry, rz, rw, ryaw, rpitch, rroll;
          try {
            rx = config->get_float((cfg_prefix + "rot_x").c_str());
            ry = config->get_float((cfg_prefix + "rot_y").c_str());
            rz = config->get_float((cfg_prefix + "rot_z").c_str());
            rw = config->get_float((cfg_prefix + "rot_w").c_str());
            logger->log_debug(name(), "r=%f,%f,%f,%f", rx, ry, rz, rw);
          } catch (Exception &e) {
            // no quaternion or incomplete quaternion, try Euler angles
            use_quaternion = false;
            ryaw   = config->get_float((cfg_prefix + "rot_yaw").c_str());
            rpitch = config->get_float((cfg_prefix + "rot_pitch").c_str());
            rroll  = config->get_float((cfg_prefix + "rot_roll").c_str());
          }

          if (frame == child_frame) {
            throw Exception("Parent and child frames may not be the same");
          }

          try {
            Entry e;
            e.name = cfg_name;

            fawkes::Time time(clock);
            if (use_quaternion) {
              tf::Transform t(tf::Quaternion(rx, ry, rz, rw),
                              tf::Vector3(tx, ty, tz));
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
            std::list<Entry>::iterator i;
            for (i = __entries.begin(); i != __entries.end(); ++i) {
              delete i->transform;
            }
            __entries.clear();
            throw;
          }
          
        } catch (Exception &e) {
          e.prepend("Transform %s: wrong or incomplete transform data",
		    cfg_name.c_str());
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
StaticTransformsThread::finalize()
{
  std::list<Entry>::iterator i;
  for (i = __entries.begin(); i != __entries.end(); ++i) {
    delete i->transform;
  }
  __entries.clear();
}


void
StaticTransformsThread::loop()
{
  std::list<Entry>::iterator i;
  for (i = __entries.begin(); i != __entries.end(); ++i) {
    i->transform->stamp.stamp();
    tf_publisher->send_transform(*(i->transform));
  }
}
