
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

  __hand_gen = new xn::HandsGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::HandsGenerator> handgen_uniqueptr(__hand_gen);
  std::unique_ptr<xn::GestureGenerator> gesturegen_uniqueptr(__gesture_gen);
  std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
#else
  std::auto_ptr<xn::HandsGenerator> handgen_uniqueptr(__hand_gen);
  std::auto_ptr<xn::GestureGenerator> gesturegen_uniqueptr(__gesture_gen);
  std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
#endif

  __gesture_gen = new xn::GestureGenerator();
  __depth_gen = new xn::DepthGenerator();

  XnStatus st;

  fawkes::openni::get_resolution(config, __width, __height);

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_HANDS, __hand_gen);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, __depth_gen);
  //fawkes::openni::setup_map_generator(*__depth_gen, config);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_GESTURE, __gesture_gen);

  st = __hand_gen->RegisterHandCallbacks(cb_hand_create, cb_hand_update,
					 cb_hand_destroy, this, __hand_cb_handle);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register hand callbacks (%s)",
		    xnGetStatusString(st));
  }

  st = __gesture_gen->RegisterGestureCallbacks(cb_gesture_recognized,
					       cb_gesture_progress,
					       this, __gesture_cb_handle);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register gesture callbacks (%s)",
		    xnGetStatusString(st));
  }

  XnUInt16 num_g = 64;
  XnChar *gest[64];
  for (unsigned int i = 0; i < num_g; ++i) {
    gest[i] = new XnChar[64];
  }
  if ((st = __gesture_gen->EnumerateAllGestures(gest, 64, num_g)) != XN_STATUS_OK)
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
  __gesture_gen->AddGesture("Wave", NULL);
  __enabled_gesture["Wave"] = true;
  logger->log_debug(name(), "Enabling gesture 'Click'");
  __gesture_gen->AddGesture("Click", NULL);
  __enabled_gesture["Click"] = true;

  __hand_gen->StartGenerating();
  __gesture_gen->StartGenerating();

  // XnChar tmp[1000];
  // XnUInt64 tmpi;
  // //  if (__gesture_gen->GetIntProperty("AdaptiveDownscaleClosestVGA", tmpi) != XN_STATUS_OK) {
  // if ((st = __gesture_gen->GetStringProperty("Resolution", tmp, 1000)) == XN_STATUS_OK) {
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
  for (i = __hands.begin(); i != __hands.end(); ++i) {
    __hand_gen->StopTracking(i->first);
    i->second->set_visible(false);
    i->second->write();
    blackboard->close(i->second);
  }
  __hands.clear();

  std::map<std::string, bool>::iterator g;
  for (g = __enabled_gesture.begin(); g != __enabled_gesture.end(); ++g) {
    if (g->second) {
      __gesture_gen->RemoveGesture(g->first.c_str());
    }
  }

  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __hand_gen;
  delete __gesture_gen;
}


void
OpenNiHandTrackerThread::loop()
{
  if (! __hand_gen->IsDataNew())  return;

  HandMap::iterator i;
  for (i = __hands.begin(); i != __hands.end(); ++i) {
    if (__needs_write[i->first]) {
      i->second->write();
      __needs_write[i->first] = false;
    }
  }
}

void
OpenNiHandTrackerThread::update_hand(XnUserID &user, const XnPoint3D *position)
{
  // convert to Fawkes coordinates
  __hands[user]->set_visible(true);
  __hands[user]->set_relative_x( position->Z * 0.001);
  __hands[user]->set_relative_y(-position->X * 0.001);
  __hands[user]->set_relative_z( position->Y * 0.001);

  XnPoint3D proj;
  fawkes::openni::world2projection(__depth_gen, 1, position, &proj,
				   __width, __height);
  __hands[user]->set_world_x(proj.X);
  __hands[user]->set_world_y(proj.Y);
  __hands[user]->set_world_z(user);

  __needs_write[user] = true;

  //logger->log_debug(name(), "New hand pos: (%f,%f,%f)",
  //		    __hands[user]->relative_x(), __hands[user]->relative_y(),
  //		    __hands[user]->relative_z());
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
  if (__hands.find(user) != __hands.end()) {
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
    __hands[user] = blackboard->open_for_writing<ObjectPositionInterface>(ifid);
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
  if (__hands.find(user) == __hands.end()) {
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
  if (__hands.find(user) == __hands.end()) {
    logger->log_error(name(), "Got destroy for untracked hand %u", user);
    return;
  }

  //__hand_gen->StopTracking(user);

  __hands[user]->set_visible(false);
  __hands[user]->write();

  logger->log_error(name(), "Lost hand ID %u, closing interface '%s'",
		    user, __hands[user]->uid());

  blackboard->close(__hands[user]);
  __needs_write.erase(user);
  __hands.erase(user);

  std::map<std::string, bool>::iterator i;
  for (i = __enabled_gesture.begin(); i != __enabled_gesture.end(); ++i) {
    if (! i->second) {
      logger->log_debug(name(), "Enabling gesture '%s'", i->first.c_str());
      i->second = true;
      __gesture_gen->AddGesture(i->first.c_str(), NULL);
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
  for (i = __enabled_gesture.begin(); i != __enabled_gesture.end(); ++i) {
    if (i->second) {
      logger->log_debug(name(), "Disabling gesture '%s'", i->first.c_str());
      i->second = false;
      __gesture_gen->RemoveGesture(i->first.c_str());
    }
  }
  __hand_gen->StartTracking(*end_position);
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
