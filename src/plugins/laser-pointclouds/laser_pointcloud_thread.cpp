
/***************************************************************************
 *  laser_pointcloud_thread.cpp - Convert laser data to pointclouds
 *
 *  Created: Thu Nov 17 10:21:55 2011
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

#include "laser_pointcloud_thread.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>
#include <pcl_utils/utils.h>

#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <interfaces/Laser1080Interface.h>

using namespace fawkes;

/** @class LaserPointCloudThread "tf_thread.h"
 * Thread to exchange transforms between Fawkes and ROS.
 * This threads connects to Fawkes and ROS to read and write transforms.
 * Transforms received on one end are republished to the other side. To
 * Fawkes new frames are published during the sensor hook.
 * @author Tim Niemueller
 */

/** Constructor. */
LaserPointCloudThread::LaserPointCloudThread()
  : Thread("LaserPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE),
    BlackBoardInterfaceListener("LaserPointCloudThread")
{
}

/** Destructor. */
LaserPointCloudThread::~LaserPointCloudThread()
{
}


void
LaserPointCloudThread::init()
{
  std::list<Laser360Interface *> l360ifs =
    blackboard->open_multiple_for_reading<Laser360Interface>("*");

  std::list<Laser720Interface *> l720ifs =
    blackboard->open_multiple_for_reading<Laser720Interface>("*");

  std::list<Laser1080Interface *> l1080ifs =
    blackboard->open_multiple_for_reading<Laser1080Interface>("*");

  LockList<Laser360Interface *>::iterator i;
  for (i = l360ifs.begin(); i != l360ifs.end(); ++i) {
    InterfaceCloudMapping mapping;
    mapping.id = interface_to_pcl_name((*i)->id());
    mapping.size = 360;
    mapping.interface_typed.as360 = *i;
    mapping.interface = *i;
    mapping.interface->read();
    mapping.cloud = new pcl::PointCloud<pcl::PointXYZ>();
    mapping.cloud->points.resize(360);
    mapping.cloud->header.frame_id = (*i)->frame();
    mapping.cloud->height = 1;
    mapping.cloud->width = 360;
    pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    bbil_add_reader_interface(*i);
    bbil_add_writer_interface(*i);
    __mappings.push_back(mapping);
  }
  LockList<Laser720Interface *>::iterator j;
  for (j = l720ifs.begin(); j != l720ifs.end(); ++j) {
    InterfaceCloudMapping mapping;
    mapping.id = interface_to_pcl_name((*j)->id());
    mapping.size = 720;
    mapping.interface_typed.as720 = *j;
    mapping.interface = *j;
    mapping.interface->read();
    mapping.cloud = new pcl::PointCloud<pcl::PointXYZ>();
    mapping.cloud->points.resize(720);
    mapping.cloud->header.frame_id = (*j)->frame();
    mapping.cloud->height = 1;
    mapping.cloud->width = 720;
    pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    bbil_add_reader_interface(*j);
    bbil_add_writer_interface(*j);
    __mappings.push_back(mapping);
  }

  LockList<Laser1080Interface *>::iterator k;
  for (k = l1080ifs.begin(); k != l1080ifs.end(); ++k) {
    InterfaceCloudMapping mapping;
    mapping.id = interface_to_pcl_name((*k)->id());
    mapping.size = 1080;
    mapping.interface_typed.as1080 = *k;
    mapping.interface = *k;
    mapping.interface->read();
    mapping.cloud = new pcl::PointCloud<pcl::PointXYZ>();
    mapping.cloud->points.resize(1080);
    mapping.cloud->header.frame_id = (*k)->frame();
    mapping.cloud->height = 1;
    mapping.cloud->width = 1080;
    pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    bbil_add_reader_interface(*k);
    bbil_add_writer_interface(*k);
    __mappings.push_back(mapping);
  }

  blackboard->register_listener(this);

  bbio_add_observed_create("Laser360Interface", "*");
  bbio_add_observed_create("Laser720Interface", "*");
  bbio_add_observed_create("Laser1080Interface", "*");
  blackboard->register_observer(this);

  // Generate lookup tables for sin and cos
  for (unsigned int i = 0; i < 360; ++i) {
    sin_angles360[i] = sinf(deg2rad(i));
    cos_angles360[i] = cosf(deg2rad(i));
  }
  for (unsigned int i = 0; i < 720; ++i) {
    sin_angles720[i] = sinf(deg2rad((float)i / 2.));
    cos_angles720[i] = cosf(deg2rad((float)i / 2.));
  }
  for (unsigned int i = 0; i < 1080; ++i) {
    sin_angles1080[i] = sinf(deg2rad((float)i / 3.));
    cos_angles1080[i] = cosf(deg2rad((float)i / 3.));
  }
}


void
LaserPointCloudThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);

  LockList<InterfaceCloudMapping>::iterator m;
  for (m = __mappings.begin(); m != __mappings.end(); ++m) {
    blackboard->close(m->interface);
    pcl_manager->remove_pointcloud(m->id.c_str());
  }
  __mappings.clear();
}


void
LaserPointCloudThread::loop()
{
  MutexLocker lock(__mappings.mutex());

  LockList<InterfaceCloudMapping>::iterator m;
  for (m = __mappings.begin(); m != __mappings.end(); ++m) {
    m->interface->read();
    if (! m->interface->changed()) {
      continue;
    }
    if (m->size == 360) {
      m->cloud->header.frame_id = m->interface_typed.as360->frame();
      float *distances = m->interface_typed.as360->distances();
      for (unsigned int i = 0; i < 360; ++i) {
        m->cloud->points[i].x = distances[i] * cos_angles360[i];
        m->cloud->points[i].y = distances[i] * sin_angles360[i];
      }

    } else if (m->size == 720) {
      m->cloud->header.frame_id = m->interface_typed.as720->frame();
      float *distances = m->interface_typed.as720->distances();
      for (unsigned int i = 0; i < 720; ++i) {
        m->cloud->points[i].x = distances[i] * cos_angles720[i];
        m->cloud->points[i].y = distances[i] * sin_angles720[i];
      }

    } else if (m->size == 1080) {
      m->cloud->header.frame_id = m->interface_typed.as1080->frame();
      float *distances = m->interface_typed.as1080->distances();
      for (unsigned int i = 0; i < 1080; ++i) {
        m->cloud->points[i].x = distances[i] * cos_angles1080[i];
        m->cloud->points[i].y = distances[i] * sin_angles1080[i];
      }
    }

    pcl_utils::set_time(m->cloud, *(m->interface->timestamp()));
  }
}


void
LaserPointCloudThread::bb_interface_created(const char *type, const char *id) throw()
{
  InterfaceCloudMapping mapping;
  mapping.id = interface_to_pcl_name(id);
  mapping.cloud = new pcl::PointCloud<pcl::PointXYZ>();
  mapping.cloud->height = 1;

  if (strncmp(type, "Laser360Interface", __INTERFACE_TYPE_SIZE) == 0) {
    Laser360Interface *lif;
    try {
      lif = blackboard->open_for_reading<Laser360Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      mapping.size = 360;
      mapping.interface_typed.as360 = lif;
      mapping.interface = lif;
      mapping.cloud->points.resize(360);
      mapping.cloud->header.frame_id = lif->frame();
      mapping.cloud->width = 360;
      pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to add pointcloud %s: %s",
                       mapping.id.c_str(), e.what());
      blackboard->close(lif);
      return;
    }

  } else if (strncmp(type, "Laser720Interface", __INTERFACE_TYPE_SIZE) != 0) {
    Laser720Interface *lif;
    try {
      lif = blackboard->open_for_reading<Laser720Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      mapping.size = 720;
      mapping.interface_typed.as720 = lif;
      mapping.interface = lif;
      mapping.cloud->points.resize(720);
      mapping.cloud->header.frame_id = lif->frame();
      mapping.cloud->width = 720;
      pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to add pointcloud %s: %s",
                       mapping.id.c_str(), e.what());
      blackboard->close(lif);
      return;
    }

  } else if (strncmp(type, "Laser1080Interface", __INTERFACE_TYPE_SIZE) != 0) {
    Laser1080Interface *lif;
    try {
      lif = blackboard->open_for_reading<Laser1080Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      mapping.size = 1080;
      mapping.interface_typed.as1080 = lif;
      mapping.interface = lif;
      mapping.cloud->points.resize(1080);
      mapping.cloud->header.frame_id = lif->frame();
      mapping.cloud->width = 1080;
      pcl_manager->add_pointcloud(mapping.id.c_str(), mapping.cloud);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to add pointcloud %s: %s",
                       mapping.id.c_str(), e.what());
      blackboard->close(lif);
      return;
    }
  }

  try {
    bbil_add_data_interface(mapping.interface);
    blackboard->update_listener(this);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to register for %s:%s: %s",
                     type, id, e.what());
    try {
      bbil_remove_data_interface(mapping.interface);
      blackboard->update_listener(this);
      blackboard->close(mapping.interface);
      pcl_manager->remove_pointcloud(mapping.id.c_str());
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to deregister %s:%s during error recovery: %s",
                        type, id, e.what());
      
    }
    return;
  }

  __mappings.push_back(mapping);
}

void
LaserPointCloudThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                                   unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
LaserPointCloudThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                                   unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
LaserPointCloudThread::conditional_close(Interface *interface) throw()
{
  Laser360Interface *l360if = dynamic_cast<Laser360Interface *>(interface);
  Laser720Interface *l720if = dynamic_cast<Laser720Interface *>(interface);
  Laser1080Interface *l1080if = dynamic_cast<Laser1080Interface *>(interface);

  bool close = false;
  InterfaceCloudMapping mapping;

  MutexLocker lock(__mappings.mutex());

  fawkes::LockList<InterfaceCloudMapping>::iterator m;
  for (m = __mappings.begin(); m != __mappings.end(); ++m) {
    bool match = ((m->size == 360 && l360if && (*l360if == *m->interface_typed.as360)) ||
                  (m->size == 720 && l720if && (*l720if == *m->interface_typed.as720)) ||
                  (m->size == 1080 && l1080if && (*l1080if == *m->interface_typed.as1080)));


    if (match) {
      if (! m->interface->has_writer() && (m->interface->num_readers() == 1)) {
        // It's only us
        logger->log_info(name(), "Last on %s, closing", m->interface->uid());
	close = true;
	mapping = *m;
	__mappings.erase(m);
        break;
      }
    }
  }

  lock.unlock();

  if (close) {
    std::string uid = mapping.interface->uid();
    try {
      bbil_remove_data_interface(mapping.interface);
      blackboard->update_listener(this);
      blackboard->close(mapping.interface);
      pcl_manager->remove_pointcloud(mapping.id.c_str());
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to unregister or close %s: %s",
                        uid.c_str(), e.what());
    }
  }
}


std::string
LaserPointCloudThread::interface_to_pcl_name(const char *interface_id)
{
  std::string rv = interface_id;
  if (rv.find("Laser ") == 0) {
    // starts with "Laser ", remove it
    rv = rv.substr(strlen("Laser "));
  }

  // Replace space by dash
  std::string::size_type pos = 0;
  while ((pos = rv.find(" ", pos)) != std::string::npos) {
    rv.replace(pos, 1, "-");
  }

  return rv;
}
