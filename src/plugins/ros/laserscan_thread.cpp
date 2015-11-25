
/***************************************************************************
 *  laserscan_thread.cpp - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:41:18 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "laserscan_thread.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>

#include <ros/this_node.h>
#include <sensor_msgs/LaserScan.h>

#include <fnmatch.h>

using namespace fawkes;

/** @class RosLaserScanThread "pcl_thread.h"
 * Thread to exchange point clouds between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
RosLaserScanThread::RosLaserScanThread()
  : Thread("RosLaserScanThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    BlackBoardInterfaceListener("RosLaserScanThread")
{
  __ls_msg_queue_mutex = new Mutex();
  __seq_num_mutex = new Mutex();
}

/** Destructor. */
RosLaserScanThread::~RosLaserScanThread()
{
  delete __ls_msg_queue_mutex;
  delete __seq_num_mutex;
}


std::string
RosLaserScanThread::topic_name(const char *if_id, const char *suffix)
{
  std::string topic_name = std::string("fawkes_scans/") + if_id + "_" + suffix;
  std::string::size_type pos = 0;
  while ((pos = topic_name.find("-", pos)) != std::string::npos) {
    topic_name.replace(pos, 1, "_");
  }
  pos = 0;
  while ((pos = topic_name.find(" ", pos)) != std::string::npos) {
    topic_name.replace(pos, 1, "_");
  }
  return topic_name;
}


void
RosLaserScanThread::init()
{
  __active_queue = 0;
  __seq_num = 0;

  // Must do that before registering listener because we might already
  // get events right away
  __sub_ls = rosnode->subscribe("scan", 100,
                                &RosLaserScanThread::laser_scan_message_cb, this);

  __ls360_ifs =
    blackboard->open_multiple_for_reading<Laser360Interface>("*");
  __ls720_ifs =
    blackboard->open_multiple_for_reading<Laser720Interface>("*");
  __ls1080_ifs =
    blackboard->open_multiple_for_reading<Laser1080Interface>("*");

  std::list<Laser360Interface *>::iterator i360;
  for (i360 = __ls360_ifs.begin(); i360 != __ls360_ifs.end(); ++i360) {
    (*i360)->read();
    logger->log_info(name(), "Opened %s", (*i360)->uid());
    bbil_add_data_interface(*i360);
    bbil_add_reader_interface(*i360);
    bbil_add_writer_interface(*i360);

    std::string topname = topic_name((*i360)->id(), "360");

    PublisherInfo pi;
    pi.pub =
      rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

    logger->log_info(name(), "Publishing laser scan %s at %s",
                     (*i360)->uid(), topname.c_str());

    pi.msg.header.frame_id = (*i360)->frame();
    pi.msg.angle_min = 0;
    pi.msg.angle_max = 2*M_PI;
    pi.msg.angle_increment = deg2rad(1);
    pi.msg.ranges.resize(360);

    __pubs[(*i360)->uid()] = pi;
  }

  std::list<Laser720Interface *>::iterator i720;
  for (i720 = __ls720_ifs.begin(); i720 != __ls720_ifs.end(); ++i720) {
    logger->log_info(name(), "Opened %s", (*i720)->uid());
    bbil_add_data_interface(*i720);
    bbil_add_reader_interface(*i720);
    bbil_add_writer_interface(*i720);

    std::string topname = topic_name((*i720)->id(), "720");

    PublisherInfo pi;
    pi.pub =
      rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

    logger->log_info(name(), "Publishing laser scan %s at %s",
                     (*i720)->uid(), topname.c_str());

    pi.msg.header.frame_id = (*i720)->frame();
    pi.msg.angle_min = 0;
    pi.msg.angle_max = 2*M_PI;
    pi.msg.angle_increment = deg2rad(0.5);
    pi.msg.ranges.resize(720);

    __pubs[(*i720)->uid()] = pi;
  }

  std::list<Laser1080Interface *>::iterator i1080;
  for (i1080 = __ls1080_ifs.begin(); i1080 != __ls1080_ifs.end(); ++i1080) {
    logger->log_info(name(), "Opened %s", (*i1080)->uid());
    bbil_add_data_interface(*i1080);
    bbil_add_reader_interface(*i1080);
    bbil_add_writer_interface(*i1080);

    std::string topname = topic_name((*i1080)->id(), "1080");

    PublisherInfo pi;
    pi.pub =
      rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

    logger->log_info(name(), "Publishing laser scan %s at %s, frame %s",
                     (*i1080)->uid(), topname.c_str(), (*i1080)->frame());

    pi.msg.header.frame_id = (*i1080)->frame();
    pi.msg.angle_min = 0;
    pi.msg.angle_max = 2*M_PI;
    pi.msg.angle_increment = deg2rad(1. / 3.);
    pi.msg.ranges.resize(1080);

    __pubs[(*i1080)->uid()] = pi;
  }

  blackboard->register_listener(this);

  bbio_add_observed_create("Laser360Interface", "*");
  bbio_add_observed_create("Laser720Interface", "*");
  bbio_add_observed_create("Laser1080Interface", "*");
  blackboard->register_observer(this);
}


void
RosLaserScanThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);

  __sub_ls.shutdown();

  std::map<std::string, PublisherInfo>::iterator p;
  for (p = __pubs.begin(); p != __pubs.end(); ++p) {
    p->second.pub.shutdown();
  }

  std::list<Laser360Interface *>::iterator i360;
  for (i360 = __ls360_ifs.begin(); i360 != __ls360_ifs.end(); ++i360) {
    blackboard->close(*i360);
  }
  __ls360_ifs.clear();
  std::list<Laser720Interface *>::iterator i720;
  for (i720 = __ls720_ifs.begin(); i720 != __ls720_ifs.end(); ++i720) {
    blackboard->close(*i720);
  }
  __ls720_ifs.clear();
  std::list<Laser1080Interface *>::iterator i1080;
  for (i1080 = __ls1080_ifs.begin(); i1080 != __ls1080_ifs.end(); ++i1080) {
    blackboard->close(*i1080);
  }
  __ls1080_ifs.clear();
}


void
RosLaserScanThread::loop()
{
  __ls_msg_queue_mutex->lock();
  unsigned int queue = __active_queue;
  __active_queue = 1 - __active_queue;
  __ls_msg_queue_mutex->unlock();

  while (! __ls_msg_queues[queue].empty()) {
    const ros::MessageEvent<sensor_msgs::LaserScan const> &msg_evt =
      __ls_msg_queues[queue].front();

    sensor_msgs::LaserScan::ConstPtr msg = msg_evt.getConstMessage();

    // Check if interface exists, open if it does not
    const std::string callerid = msg_evt.getPublisherName();

    // for now we only create 360 interfaces, might add on that later
    if (callerid.empty()) {
      logger->log_warn(name(), "Received laser scan from ROS without caller ID,"
                       "ignoring");
    } else {
      bool have_interface = true;
      if (__ls360_wifs.find(callerid) == __ls360_wifs.end()) {
        try {
          std::string id = std::string("ROS Laser ") + callerid;
          Laser360Interface *ls360if =
            blackboard->open_for_writing<Laser360Interface>(id.c_str());
          __ls360_wifs[callerid] = ls360if;
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to open ROS laser interface for "
                           "message from node %s, exception follows",
                           callerid.c_str());
          logger->log_warn(name(), e);
          have_interface = false;
        }
      }

      if (have_interface) {
        // update interface with laser data
        Laser360Interface *ls360if = __ls360_wifs[callerid];
        ls360if->set_frame(msg->header.frame_id.c_str());
        float distances[360];
        for (unsigned int a = 0; a < 360; ++a) {
          float a_rad = deg2rad(a);
          if ((a_rad < msg->angle_min) || (a_rad > msg->angle_max)) {
            distances[a] = 0.;
          } else {
            // get closest ray from message
            int idx =
              (int)roundf((a_rad - msg->angle_min) / msg->angle_increment);
            distances[a] = msg->ranges[idx];
          }
        }
        ls360if->set_distances(distances);
        ls360if->write();
      }
    }

    __ls_msg_queues[queue].pop();
  }
}


void
RosLaserScanThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  Laser360Interface *ls360if = dynamic_cast<Laser360Interface *>(interface);
  Laser720Interface *ls720if = dynamic_cast<Laser720Interface *>(interface);
  Laser1080Interface *ls1080if = dynamic_cast<Laser1080Interface *>(interface);

  PublisherInfo &pi = __pubs[interface->uid()];
  sensor_msgs::LaserScan &msg = pi.msg;

  if (ls360if) {
    ls360if->read();

    const Time *time = ls360if->timestamp();

    __seq_num_mutex->lock();
    msg.header.seq = ++__seq_num;
    __seq_num_mutex->unlock();
    msg.header.stamp = ros::Time(time->get_sec(), time->get_nsec());
    msg.header.frame_id = ls360if->frame();

    msg.angle_min = 0;
    msg.angle_max = 2*M_PI;
    msg.angle_increment = deg2rad(1);
    msg.range_min = 0.;
    msg.range_max = 1000.;
    msg.ranges.resize(360);
    memcpy(&msg.ranges[0], ls360if->distances(), 360*sizeof(float));

    pi.pub.publish(pi.msg);

  } else if (ls720if) {
    ls720if->read();

    const Time *time = ls720if->timestamp();

    __seq_num_mutex->lock();
    msg.header.seq = ++__seq_num;
    __seq_num_mutex->unlock();
    msg.header.stamp = ros::Time(time->get_sec(), time->get_nsec());
    msg.header.frame_id = ls720if->frame();

    msg.angle_min = 0;
    msg.angle_max = 2*M_PI;
    msg.angle_increment = deg2rad(1./2.);
    msg.range_min = 0.;
    msg.range_max = 1000.;
    msg.ranges.resize(720);
    memcpy(&msg.ranges[0], ls720if->distances(), 720*sizeof(float));

    pi.pub.publish(pi.msg);

  } else if (ls1080if) {
    ls1080if->read();

    const Time *time = ls1080if->timestamp();

    __seq_num_mutex->lock();
    msg.header.seq = ++__seq_num;
    __seq_num_mutex->unlock();
    msg.header.stamp = ros::Time(time->get_sec(), time->get_nsec());
    msg.header.frame_id = ls1080if->frame();

    msg.angle_min = 0;
    msg.angle_max = 2*M_PI;
    msg.angle_increment = deg2rad(1./3.);
    msg.range_min = 0.;
    msg.range_max = 1000.;
    msg.ranges.resize(1080);
    memcpy(&msg.ranges[0], ls1080if->distances(), 1080*sizeof(float));

    pi.pub.publish(pi.msg);
  }

}


void
RosLaserScanThread::bb_interface_created(const char *type, const char *id) throw()
{
  // Ignore ID pattern of our own interfaces
  if (fnmatch("ROS *", id, FNM_NOESCAPE) == 0) return;

  if (strncmp(type, "Laser360Interface", __INTERFACE_TYPE_SIZE) == 0) {

    Laser360Interface *ls360if;
    try {
      logger->log_info(name(), "Opening %s:%s", type, id);
      ls360if = blackboard->open_for_reading<Laser360Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      bbil_add_data_interface(ls360if);
      bbil_add_reader_interface(ls360if);
      bbil_add_writer_interface(ls360if);

      std::string topname = topic_name(ls360if->id(), "360");

      PublisherInfo pi;
      pi.pub = rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

      logger->log_info(name(), "Publishing laser scan %s at %s",
                       ls360if->uid(), topname.c_str());

      pi.msg.header.frame_id = ls360if->frame();
      pi.msg.angle_min = 0;
      pi.msg.angle_max = 2*M_PI;
      pi.msg.angle_increment = deg2rad(1);
      pi.msg.ranges.resize(360);

      __pubs[ls360if->uid()] = pi;    

      blackboard->update_listener(this);
      __ls360_ifs.push_back(ls360if);
    } catch (Exception &e) {
      blackboard->close(ls360if);
      logger->log_warn(name(), "Failed to register for %s:%s: %s",
                       type, id, e.what());
      return;
    }

  } else if (strncmp(type, "Laser720Interface", __INTERFACE_TYPE_SIZE) == 0) {
    Laser720Interface *ls720if;
    try {
      logger->log_info(name(), "Opening %s:%s", type, id);
      ls720if = blackboard->open_for_reading<Laser720Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      bbil_add_data_interface(ls720if);
      bbil_add_reader_interface(ls720if);
      bbil_add_writer_interface(ls720if);

      std::string topname = topic_name(ls720if->id(), "720");

      PublisherInfo pi;
      pi.pub = rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

      logger->log_info(name(), "Publishing laser scan %s at %s",
                       ls720if->uid(), topname.c_str());

      pi.msg.header.frame_id = ls720if->frame();
      pi.msg.angle_min = 0;
      pi.msg.angle_max = 2*M_PI;
      pi.msg.angle_increment = deg2rad(0.5);
      pi.msg.ranges.resize(720);

      __pubs[ls720if->uid()] = pi;

      blackboard->update_listener(this);
      __ls720_ifs.push_back(ls720if);
    } catch (Exception &e) {
      blackboard->close(ls720if);
      logger->log_warn(name(), "Failed to register for %s:%s: %s",
                       type, id, e.what());
      return;
    }

  } else if (strncmp(type, "Laser1080Interface", __INTERFACE_TYPE_SIZE) == 0) {
    Laser1080Interface *ls1080if;
    try {
      logger->log_info(name(), "Opening %s:%s", type, id);
      ls1080if = blackboard->open_for_reading<Laser1080Interface>(id);
    } catch (Exception &e) {
      // ignored
      logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
      return;
    }

    try {
      bbil_add_data_interface(ls1080if);
      bbil_add_reader_interface(ls1080if);
      bbil_add_writer_interface(ls1080if);

      std::string topname = topic_name(ls1080if->id(), "1080");

      PublisherInfo pi;
      pi.pub = rosnode->advertise<sensor_msgs::LaserScan>(topname, 1);

      logger->log_info(name(), "Publishing 1080 laser scan %s at %s",
                       ls1080if->uid(), topname.c_str());

      pi.msg.header.frame_id = ls1080if->frame();
      pi.msg.angle_min = 0;
      pi.msg.angle_max = 2*M_PI;
      pi.msg.angle_increment = deg2rad(0.5);
      pi.msg.ranges.resize(1080);

      __pubs[ls1080if->uid()] = pi;

      blackboard->update_listener(this);
      __ls1080_ifs.push_back(ls1080if);
    } catch (Exception &e) {
      blackboard->close(ls1080if);
      logger->log_warn(name(), "Failed to register for %s:%s: %s",
                       type, id, e.what());
      return;
    }
  }
}

void
RosLaserScanThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosLaserScanThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosLaserScanThread::conditional_close(Interface *interface) throw()
{
  // Verify it's a laser interface
  Laser360Interface *ls360if = dynamic_cast<Laser360Interface *>(interface);
  Laser720Interface *ls720if = dynamic_cast<Laser720Interface *>(interface);
  Laser1080Interface *ls1080if = dynamic_cast<Laser1080Interface *>(interface);

  if (ls360if) {
    std::list<Laser360Interface *>::iterator i;
    for (i = __ls360_ifs.begin(); i != __ls360_ifs.end(); ++i) {
      if (*ls360if == **i) {
        if (! ls360if->has_writer() && (ls360if->num_readers() == 1)) {
          // It's only us
          logger->log_info(name(), "Last on %s, closing", ls360if->uid());
          bbil_remove_data_interface(*i);
          bbil_remove_reader_interface(*i);
          bbil_remove_writer_interface(*i);
          blackboard->update_listener(this);
          blackboard->close(*i);
          __ls360_ifs.erase(i);
          break;
        }
      }
    }
  } else if (ls720if) {
    std::list<Laser720Interface *>::iterator i;
    for (i = __ls720_ifs.begin(); i != __ls720_ifs.end(); ++i) {
      if (*ls720if == **i) {
        if (! ls720if->has_writer() && (ls720if->num_readers() == 1)) {
          // It's only us
          logger->log_info(name(), "Last on %s, closing", ls720if->uid());
          bbil_remove_data_interface(*i);
          bbil_remove_reader_interface(*i);
          bbil_remove_writer_interface(*i);
          blackboard->update_listener(this);
          blackboard->close(*i);
          __ls720_ifs.erase(i);
          break;
        }
      }
    }

  } else if (ls1080if) {
    std::list<Laser1080Interface *>::iterator i;
    for (i = __ls1080_ifs.begin(); i != __ls1080_ifs.end(); ++i) {
      if (*ls1080if == **i) {
        if (! ls1080if->has_writer() && (ls1080if->num_readers() == 1)) {
          // It's only us
          logger->log_info(name(), "Last on %s, closing", ls1080if->uid());
          bbil_remove_data_interface(*i);
          bbil_remove_reader_interface(*i);
          bbil_remove_writer_interface(*i);
          blackboard->update_listener(this);
          blackboard->close(*i);
          __ls1080_ifs.erase(i);
          break;
        }
      }
    }
  }
}


/** Callback function for ROS laser scan message subscription.
 * @param msg incoming message
 */
void
RosLaserScanThread::laser_scan_message_cb(const ros::MessageEvent<sensor_msgs::LaserScan const> &msg_evt)
{
  MutexLocker lock(__ls_msg_queue_mutex);
  __ls_msg_queues[__active_queue].push(msg_evt);
}
