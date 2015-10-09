
/***************************************************************************
 *  tf_thread.cpp - Thread to exchange transforms
 *
 *  Created: Wed Oct 26 01:02:59 2011
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

#include "tf_thread.h"

#include <core/threading/mutex_locker.h>
#include <ros/this_node.h>

using namespace fawkes;

/** @class RosTfThread "tf_thread.h"
 * Thread to exchange transforms between Fawkes and ROS.
 * This threads connects to Fawkes and ROS to read and write transforms.
 * Transforms received on one end are republished to the other side. To
 * Fawkes new frames are published during the sensor hook.
 * @author Tim Niemueller
 */

/** Constructor. */
RosTfThread::RosTfThread()
  : Thread("RosTfThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::ONLY_PUBLISHER, "ROS"),
    BlackBoardInterfaceListener("RosTfThread")
{
  __tf_msg_queue_mutex = new Mutex();
  __seq_num_mutex = new Mutex();
}

/** Destructor. */
RosTfThread::~RosTfThread()
{
  delete __tf_msg_queue_mutex;
  delete __seq_num_mutex;
}



void
RosTfThread::init()
{
  __active_queue = 0;
  __seq_num = 0;

  // Must do that before registering listener because we might already
  // get events right away
#if ROS_VERSION_MINIMUM(1,11,0)
  __sub_tf = rosnode->subscribe<::tf::tfMessage>("/tf", 100, boost::bind(&RosTfThread::tf_message_cb, this, _1));
#else
  __sub_tf = rosnode->subscribe("/tf", 100, &RosTfThread::tf_message_cb, this);
#endif
  __pub_tf = rosnode->advertise< ::tf::tfMessage >("/tf", 100);

  __tfifs = blackboard->open_multiple_for_reading<TransformInterface>("TF *");

  std::list<TransformInterface *>::iterator i;
  std::list<TransformInterface *>::iterator own_if = __tfifs.end();
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
    //logger->log_info(name(), "Opened %s", (*i)->uid());
    if (strcmp((*i)->id(), "TF ROS") == 0) {
      // that's our own Fawkes publisher, do NOT republish what we receive...
      own_if = i;
      blackboard->close(*i);
    } else {
      bbil_add_data_interface(*i);
      bbil_add_reader_interface(*i);
      bbil_add_writer_interface(*i);
    }
  }
  if (own_if != __tfifs.end()) __tfifs.erase(own_if);
  blackboard->register_listener(this);

  bbio_add_observed_create("TransformInterface", "TF *");
  blackboard->register_observer(this);

}


void
RosTfThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);

  __sub_tf.shutdown();
  __pub_tf.shutdown();

  std::list<TransformInterface *>::iterator i;
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
    blackboard->close(*i);
  }
  __tfifs.clear();
}


void
RosTfThread::loop()
{
  __tf_msg_queue_mutex->lock();
  unsigned int queue = __active_queue;
  __active_queue = 1 - __active_queue;
  __tf_msg_queue_mutex->unlock();

  while (! __tf_msg_queues[queue].empty()) {
    const ::tf::tfMessage::ConstPtr &msg = __tf_msg_queues[queue].front();
    const size_t tsize = msg->transforms.size();
    for (size_t i = 0; i < tsize; ++i) {
      const geometry_msgs::TransformStamped &ts = msg->transforms[i];
      const geometry_msgs::Vector3 &t = ts.transform.translation;
      const geometry_msgs::Quaternion &r = ts.transform.rotation;

      fawkes::Time time(ts.header.stamp.sec, ts.header.stamp.nsec / 1000);

      fawkes::tf::Transform tr(fawkes::tf::Quaternion(r.x, r.y, r.z, r.w),
                               fawkes::tf::Vector3(t.x, t.y, t.z));
      fawkes::tf::StampedTransform
        st(tr, time, ts.header.frame_id, ts.child_frame_id);

      tf_publisher->send_transform(st);
    }
    __tf_msg_queues[queue].pop();
  }
}


void
RosTfThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;

  tfif->read();


  double *translation = tfif->translation();
  double *rotation = tfif->rotation();
  const Time *time = tfif->timestamp();

  geometry_msgs::Vector3 t;
  t.x = translation[0]; t.y = translation[1]; t.z = translation[2];
  geometry_msgs::Quaternion r;
  r.x = rotation[0]; r.y = rotation[1]; r.z = rotation[2]; r.w = rotation[3];
  geometry_msgs::Transform tr;
  tr.translation = t;
  tr.rotation = r;

  geometry_msgs::TransformStamped ts;
  __seq_num_mutex->lock();
  ts.header.seq = ++__seq_num;
  __seq_num_mutex->unlock();
  ts.header.stamp = ros::Time(time->get_sec(), time->get_nsec());
  ts.header.frame_id = tfif->frame();
  ts.child_frame_id = tfif->child_frame();
  ts.transform = tr;

  ::tf::tfMessage tmsg;
  tmsg.transforms.push_back(ts);

  __pub_tf.publish(tmsg);
}


void
RosTfThread::bb_interface_created(const char *type, const char *id) throw()
{
  if (strncmp(type, "TransformInterface", __INTERFACE_TYPE_SIZE) != 0)  return;

  TransformInterface *tfif;
  try {
    //logger->log_info(name(), "Opening %s:%s", type, id);
    tfif = blackboard->open_for_reading<TransformInterface>(id);
  } catch (Exception &e) {
    // ignored
    logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
    return;
  }

  try {
    bbil_add_data_interface(tfif);
    bbil_add_reader_interface(tfif);
    bbil_add_writer_interface(tfif);
    blackboard->update_listener(this);
    __tfifs.push_back(tfif);
  } catch (Exception &e) {
    blackboard->close(tfif);
    logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
    return;
  }
}

void
RosTfThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosTfThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosTfThread::conditional_close(Interface *interface) throw()
{
  // Verify it's a TransformInterface
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;
  
  std::list<TransformInterface *>::iterator i;
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
    if (*interface == **i) {
      if (! interface->has_writer() && (interface->num_readers() == 1)) {
        // It's only us
        logger->log_info(name(), "Last on %s, closing", interface->uid());
        bbil_remove_data_interface(*i);
        bbil_remove_reader_interface(*i);
        bbil_remove_writer_interface(*i);
        blackboard->update_listener(this);
        blackboard->close(*i);
        __tfifs.erase(i);
        break;
      }
    }
  }
}


/** Callback function for ROS tf message subscription.
 * @param msg incoming message
 */
#if ROS_VERSION_MINIMUM(1,11,0)
void
RosTfThread::tf_message_cb(const ros::MessageEvent<::tf::tfMessage const> &msg_evt)
{
  MutexLocker lock(__tf_msg_queue_mutex);

  const ::tf::tfMessage::ConstPtr &msg = msg_evt.getConstMessage();
  std::string authority = msg_evt.getPublisherName();

  if (authority == "") {
    logger->log_warn(name(), "Message received without callerid");
  } else if (authority != ros::this_node::getName()) {
    __tf_msg_queues[__active_queue].push(msg);
  }
}
#else
void
RosTfThread::tf_message_cb(const ::tf::tfMessage::ConstPtr &msg)
{
  MutexLocker lock(__tf_msg_queue_mutex);

  std::map<std::string, std::string> *msg_header_map =
    msg->__connection_header.get();
  std::map<std::string, std::string>::iterator it =
    msg_header_map->find("callerid");

  if (it == msg_header_map->end()) {
    logger->log_warn(name(), "Message received without callerid");
  } else if (it->second != ros::this_node::getName()) {
    __tf_msg_queues[__active_queue].push(msg);
  }
}
#endif
