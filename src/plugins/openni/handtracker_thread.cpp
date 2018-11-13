
/***************************************************************************
 *  handtracker_thread.cpp - OpenNI hand tracker thread
 *
 *  Created: Sun Feb 27 17:53:38 2011
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

#include "handtracker_thread.h"
#include "utils/setup.h"
#include "utils/conversions.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/ObjectPositionInterface.h>

#include <memory>

using namespace fawkes;

/** @class OpenNiHandTrackerThread "handtracker_thread.h"
 * OpenNI Hand Tracker Thread.
 * This thread requests a hand tracker node from OpenNI and publishes the
 * retrieved information via the blackboard.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiHandTrackerThread::OpenNiHandTrackerThread()
  : Thread("OpenNiHandTrackerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


/** Destructor. */
OpenNiHandTrackerThread::~OpenNiHandTrackerThread()
{
}


static void XN_CALLBACK_TYPE
cb_hand_create(xn::HandsGenerator &generator, XnUserID user,
	       const XnPoint3D *position, XnFloat time, void *cookie)
{
  OpenNiHandTrackerThread *t = static_cast<OpenNiHandTrackerThread *>(cookie);
  t->hand_create(user, position, time);
}

static void XN_CALLBACK_TYPE
cb_hand_destroy(xn::HandsGenerator &generator, XnUserID user,
		XnFloat time, void *cookie)
{
  OpenNiHandTrackerThread *t = static_cast<OpenNiHandTrackerThread *>(cookie);
  t->hand_destroy(user, time);
}

static void XN_CALLBACK_TYPE
cb_hand_update(xn::HandsGenerator &generator, XnUserID user,
	       const XnPoint3D *position, XnFloat time, void *cookie)
{
  OpenNiHandTrackerThread *t = static_cast<OpenNiHandTrackerThread *>(cookie);
  t->hand_update(user, position, time);
}


static void XN_CALLBACK_TYPE
cb_gesture_recognized(xn::GestureGenerator &generator,
		      const XnChar *gesture_name, const XnPoint3D *position,
		      const XnPoint3D *end_position, void *cookie)
{
  OpenNiHandTrackerThread *t = static_cast<OpenNiHandTrackerThread *>(cookie);
  t->gesture_recognized(gesture_name, position, end_position);
}

static void XN_CALLBACK_TYPE
cb_gesture_progress(xn::GestureGenerator &generator,
		    const XnChar *gesture_name, const XnPoint3D *position,
		    XnFloat progress, void *cookie)
{
  OpenNiHandTrackerThread *t = static_cast<OpenNiHandTrackerThread *>(cookie);
  t->gesture_progress(gesture_name, position, progress);
}


void
OpenNiHandTrackerThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  hand_gen_ = new xn::HandsGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::HandsGenerator> handgen_uniqueptr(hand_gen_);
  std::unique_ptr<xn::GestureGenerator> gesturegen_uniqueptr(gesture_gen_);
  std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
#else
  std::auto_ptr<xn::HandsGenerator> handgen_uniqueptr(hand_gen_);
  std::auto_ptr<xn::GestureGenerator> gesturegen_uniqueptr(gesture_gen_);
  std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(depth_gen_);
#endif

  gesture_gen_ = new xn::GestureGenerator();
  depth_gen_ = new xn::DepthGenerator();

  XnStatus st;

  fawkes::openni::get_resolution(config, width_, height_);

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_HANDS, hand_gen_);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, depth_gen_);
  //fawkes::openni::setup_map_generator(*depth_gen_, config);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_GESTURE, gesture_gen_);

  st = hand_gen_->RegisterHandCallbacks(cb_hand_create, cb_hand_update,
					 cb_hand_destroy, this, hand_cb_handle_);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register hand callbacks (%s)",
		    xnGetStatusString(st));
  }

  st = gesture_gen_->RegisterGestureCallbacks(cb_gesture_recognized,
					       cb_gesture_progress,
					       this, gesture_cb_handle_);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register gesture callbacks (%s)",
		    xnGetStatusString(st));
  }

  XnUInt16 num_g = 64;
  XnChar *gest[64];
  for (unsigned int i = 0; i < num_g; ++i) {
    gest[i] = new XnChar[64];
  }
  if ((st = gesture_gen_->EnumerateAllGestures(gest, 64, num_g)) != XN_STATUS_OK)
  {
    logger->log_warn(name(), "Failed to enumerate gestures: %s",
		     xnGetStatusString(st));
  } else {
    for (unsigned int i = 0; i < num_g; ++i) {
      logger->log_debug(name(), "Supported gesture: %s", gest[i]);

    }
  }
  for (unsigned int i = 0; i < num_g; ++i) {
    delete[] gest[i];
  }

  logger->log_debug(name(), "Enabling gesture 'Wave'");
  gesture_gen_->AddGesture("Wave", NULL);
  enabled_gesture_["Wave"] = true;
  logger->log_debug(name(), "Enabling gesture 'Click'");
  gesture_gen_->AddGesture("Click", NULL);
  enabled_gesture_["Click"] = true;

  hand_gen_->StartGenerating();
  gesture_gen_->StartGenerating();

  // XnChar tmp[1000];
  // XnUInt64 tmpi;
  // //  if (gesture_gen_->GetIntProperty("AdaptiveDownscaleClosestVGA", tmpi) != XN_STATUS_OK) {
  // if ((st = gesture_gen_->GetStringProperty("Resolution", tmp, 1000)) == XN_STATUS_OK) {
  //   logger->log_debug(name(), "Resolution: %u", tmp);
  // } else {
  //   logger->log_debug(name(), "Failed to get resolution: %s",
  // 		      xnGetStatusString(st));
  // }

  handgen_uniqueptr.release();
  depthgen_uniqueptr.release();
  gesturegen_uniqueptr.release();
}


void
OpenNiHandTrackerThread::finalize()
{
  HandMap::iterator i;
  for (i = hands_.begin(); i != hands_.end(); ++i) {
    hand_gen_->StopTracking(i->first);
    i->second->set_visible(false);
    i->second->write();
    blackboard->close(i->second);
  }
  hands_.clear();

  std::map<std::string, bool>::iterator g;
  for (g = enabled_gesture_.begin(); g != enabled_gesture_.end(); ++g) {
    if (g->second) {
      gesture_gen_->RemoveGesture(g->first.c_str());
    }
  }

  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete hand_gen_;
  delete gesture_gen_;
}


void
OpenNiHandTrackerThread::loop()
{
  if (! hand_gen_->IsDataNew())  return;

  HandMap::iterator i;
  for (i = hands_.begin(); i != hands_.end(); ++i) {
    if (needs_write_[i->first]) {
      i->second->write();
      needs_write_[i->first] = false;
    }
  }
}

void
OpenNiHandTrackerThread::update_hand(XnUserID &user, const XnPoint3D *position)
{
  // convert to Fawkes coordinates
  hands_[user]->set_visible(true);
  hands_[user]->set_relative_x( position->Z * 0.001);
  hands_[user]->set_relative_y(-position->X * 0.001);
  hands_[user]->set_relative_z( position->Y * 0.001);

  XnPoint3D proj;
  fawkes::openni::world2projection(depth_gen_, 1, position, &proj,
				   width_, height_);
  hands_[user]->set_world_x(proj.X);
  hands_[user]->set_world_y(proj.Y);
  hands_[user]->set_world_z(user);

  needs_write_[user] = true;

  //logger->log_debug(name(), "New hand pos: (%f,%f,%f)",
  //		    hands_[user]->relative_x(), hands_[user]->relative_y(),
  //		    hands_[user]->relative_z());
}


/** Notify of new hand.
 * This is called by the OpenNI callback when a new hand has been detected.
 * @param user new user's ID
 * @param position hand position
 * @param time timestamp in seconds
 */
void
OpenNiHandTrackerThread::hand_create(XnUserID &user, const XnPoint3D *position,
				     XnFloat &time)
{
  if (hands_.find(user) != hands_.end()) {
    logger->log_error(name(), "New hand ID %u, but interface already exists", user);
    return;
  }

  char *ifid;
  if (asprintf(&ifid, "OpenNI Hand %u", user) == -1) {
    logger->log_warn(name(), "New hand ID %u, but cannot generate "
		     "interface ID", user);
    return;
  }
  try {
    logger->log_debug(name(), "Opening interface 'ObjectPositionInterface::%s'",
		      ifid);
    hands_[user] = blackboard->open_for_writing<ObjectPositionInterface>(ifid);
    update_hand(user, position);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to open interface, exception follows");
    logger->log_warn(name(), e);
  }
  free(ifid);
}


/** Notify of hand update.
 * This is called by the OpenNI callback when a new hand has been detected.
 * @param user new user's ID
 * @param position hand position
 * @param time timestamp in seconds
 */
void
OpenNiHandTrackerThread::hand_update(XnUserID &user, const XnPoint3D *position,
				     XnFloat &time)
{
  if (hands_.find(user) == hands_.end()) {
    logger->log_error(name(), "Got update for untracked hand %u", user);
    return;
  }

  update_hand(user, position);
}


/** Notify of disappeared hand.
 * This is called by the OpenNI callback when a new hand has been detected.
 * @param user new user's ID
 * @param time timestamp in seconds
 */
void
OpenNiHandTrackerThread::hand_destroy(XnUserID &user, XnFloat &time)
{
  if (hands_.find(user) == hands_.end()) {
    logger->log_error(name(), "Got destroy for untracked hand %u", user);
    return;
  }

  //hand_gen_->StopTracking(user);

  hands_[user]->set_visible(false);
  hands_[user]->write();

  logger->log_error(name(), "Lost hand ID %u, closing interface '%s'",
		    user, hands_[user]->uid());

  blackboard->close(hands_[user]);
  needs_write_.erase(user);
  hands_.erase(user);

  std::map<std::string, bool>::iterator i;
  for (i = enabled_gesture_.begin(); i != enabled_gesture_.end(); ++i) {
    if (! i->second) {
      logger->log_debug(name(), "Enabling gesture '%s'", i->first.c_str());
      i->second = true;
      gesture_gen_->AddGesture(i->first.c_str(), NULL);
    }
  }
}


/** Notify of recognized gesture.
 * @param gesture_name name of the recognized gesture
 * @param position gesture position
 * @param end_position final hand position when completing the gesture
 */
void
OpenNiHandTrackerThread::gesture_recognized(const XnChar *gesture_name,
					    const XnPoint3D *position,
					    const XnPoint3D *end_position)
{
  logger->log_debug(name(), "Gesture %s recognized, starting tracking",
		    gesture_name);

  std::map<std::string, bool>::iterator i;
  for (i = enabled_gesture_.begin(); i != enabled_gesture_.end(); ++i) {
    if (i->second) {
      logger->log_debug(name(), "Disabling gesture '%s'", i->first.c_str());
      i->second = false;
      gesture_gen_->RemoveGesture(i->first.c_str());
    }
  }
  hand_gen_->StartTracking(*end_position);
}


/** Notify of gesture progress.
 * @param gesture_name name of the recognized gesture
 * @param position gesture position
 * @param progress current progress of the recognition
 */
void
OpenNiHandTrackerThread::gesture_progress(const XnChar *gesture_name,
					  const XnPoint3D *position,
					  XnFloat progress)
{
  logger->log_debug(name(), "Gesture %s progress %f", gesture_name, progress);
}
