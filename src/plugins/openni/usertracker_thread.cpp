
/***************************************************************************
 *  usertracker_thread.cpp - OpenNI user tracker thread
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

#include "usertracker_thread.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>

#include <memory>

using namespace fawkes;

/** @class OpenNiUserTrackerThread "usertracker_thread.h"
 * OpenNI User Tracker Thread.
 * This thread requests a user tracker node from OpenNI and publishes the
 * retrieved information via the blackboard.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiUserTrackerThread::OpenNiUserTrackerThread()
  : Thread("OpenNiUserTrackerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


/** Destructor. */
OpenNiUserTrackerThread::~OpenNiUserTrackerThread()
{
}


static void XN_CALLBACK_TYPE
cb_new_user(xn::UserGenerator &generator, XnUserID id, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->new_user(id);
}

static void XN_CALLBACK_TYPE
cb_lost_user(xn::UserGenerator &generator, XnUserID id, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->lost_user(id);
}

static void XN_CALLBACK_TYPE
cb_pose_start(xn::PoseDetectionCapability &capability,
	      const XnChar *pose_name, XnUserID id, void* cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->pose_start(id, pose_name);
}

static void XN_CALLBACK_TYPE
cb_pose_end(xn::PoseDetectionCapability &capability,
	    const XnChar *pose_name, XnUserID id, void* cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->pose_end(id, pose_name);
}

static void XN_CALLBACK_TYPE
cb_calibration_start(xn::SkeletonCapability &capability, XnUserID id, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->calibration_start(id);
}

static void XN_CALLBACK_TYPE
cb_calibration_end(xn::SkeletonCapability &capability, XnUserID id,
		   XnBool success, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->calibration_end(id, success);
}


void
OpenNiUserTrackerThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  __user_gen = new xn::UserGenerator();
  std::auto_ptr<xn::UserGenerator> usergen_autoptr(__user_gen);

  __depth_gen = new xn::DepthGenerator();
  std::auto_ptr<xn::DepthGenerator> depthgen_autoptr(__depth_gen);

  XnStatus st;

  if ((st = openni->FindExistingNode(XN_NODE_TYPE_USER, *__user_gen)) != XN_STATUS_OK) {
    if (__user_gen->Create(*(openni.operator->())) != XN_STATUS_OK) {
      throw Exception("Failed to acquire user generator (%s)", xnGetStatusString(st));
    }
  }

  if ((st = openni->FindExistingNode(XN_NODE_TYPE_DEPTH, *__depth_gen)) != XN_STATUS_OK) {
    throw Exception("Failed to acquire depth generator (%s)", xnGetStatusString(st));
  }

  if (!__user_gen->IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
    throw Exception("User generator does not support skeleton capability");
  }

  st = __user_gen->RegisterUserCallbacks(cb_new_user, cb_lost_user,
					 this, __user_cb_handle);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register user callbacks (%s)", xnGetStatusString(st));
  }

  xn::SkeletonCapability skelcap = __user_gen->GetSkeletonCap();

  st = skelcap.RegisterCalibrationCallbacks(cb_calibration_start,
					    cb_calibration_end,
					    this, __calib_cb_handle);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register calibration callbacks (%s)", xnGetStatusString(st));
  }

  __skel_need_calib_pose = skelcap.NeedPoseForCalibration();

  if (__skel_need_calib_pose) {
    if (! __user_gen->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
      throw Exception("Calibration requires pose, but not supported by node");
    }
    skelcap.GetCalibrationPose(__calib_pose_name);

    xn::PoseDetectionCapability posecap = __user_gen->GetPoseDetectionCap();
    st = posecap.RegisterToPoseCallbacks(cb_pose_start, cb_pose_end,
					   this, __pose_cb_handle);
    if (st != XN_STATUS_OK) {
      throw Exception("Failed to register pose callbacks (%s)", xnGetStatusString(st));
    }
  }

  skelcap.SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
  __user_gen->StartGenerating();

  usergen_autoptr.release();
  depthgen_autoptr.release();
}


void
OpenNiUserTrackerThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __user_gen;

  UserMap::iterator i;
  for (i = __users.begin(); i != __users.end(); ++i) {
    blackboard->close(i->second.skel_if);
    blackboard->close(i->second.proj_if);
  }
}


void
OpenNiUserTrackerThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());

  //__user_gen->WaitAndUpdateData(); 
  if (! __user_gen->IsDataNew()) {
    //logger->log_debug(name(), "No new data, skipping loop");
    return;
  }

  xn::SkeletonCapability skelcap = __user_gen->GetSkeletonCap();

  UserMap::iterator i;
  for (i = __users.begin(); i != __users.end(); ++i) {
    bool needs_write = false;


    HumanSkeletonInterface::State new_state = i->second.skel_if->state();
    if (skelcap.IsTracking(i->first)) {
      new_state = HumanSkeletonInterface::STATE_TRACKING;
    } else if (skelcap.IsCalibrating(i->first)) {
      new_state = HumanSkeletonInterface::STATE_CALIBRATING;
    } else {
      new_state = HumanSkeletonInterface::STATE_DETECTING_POSE;
    }

    if (new_state != i->second.skel_if->state()) {
      i->second.skel_if->set_state(new_state);
      needs_write = true;
    }

    if (new_state == HumanSkeletonInterface::STATE_TRACKING) {
      // update skeleton information
      try {
	update_user(i->first, i->second);
	needs_write = true;
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to update skeleton data for %u, "
			 "exception follows", i->first);
	logger->log_warn(name(), e);
      }
    }

    if (needs_write) {
      i->second.skel_if->write();
      i->second.proj_if->write();
    }
  }
}




// Very noisy when added to st != XN_STATUS_OK case
//logger->log_warn(name(), "Failed to get joint transformation for "
//		     "%s joint (%s)",
//		     joint_name, xnGetStatusString(st));

#define SET_JTF(id, joint, joint_name, bbfield)				\
  st = skelcap.GetSkeletonJoint(id, joint, jtf);			\
  if (st != XN_STATUS_OK) {						\
    ori[0] = ori[1] = ori[2] = ori[3] = ori[4] = ori[5] = 0.;		\
    ori[6] = ori[7] = ori[8] = ori_confidence = 0.;			\
    proj[0] = proj[1] = 0;						\
  } else {								\
    pos[0] = jtf.position.position.X;					\
    pos[1] = jtf.position.position.Y;					\
    pos[2] = jtf.position.position.Z;					\
    pos_confidence = jtf.position.fConfidence;				\
									\
    for (unsigned char i = 0; i < 9; ++i) {				\
      ori[i] = jtf.orientation.orientation.elements[i];			\
    }									\
    ori_confidence = jtf.orientation.fConfidence;			\
									\
    XnPoint3D pt;							\
    pt = jtf.position.position;						\
    __depth_gen->ConvertRealWorldToProjective(1, &pt, &pt);		\
    proj[0] = pt.X;							\
    proj[1] = pt.Y;							\
  }									\
  user.skel_if->set_pos_##bbfield(pos);					\
  user.skel_if->set_pos_##bbfield##_confidence(pos_confidence);		\
  user.skel_if->set_ori_##bbfield(ori);					\
  user.skel_if->set_ori_##bbfield##_confidence(ori_confidence);		\
									\
  user.proj_if->set_proj_##bbfield(proj);



void
OpenNiUserTrackerThread::update_user(XnUserID id, UserInfo &user)
{
  XnSkeletonJointTransformation jtf;
  XnStatus st;

  xn::SkeletonCapability skelcap = __user_gen->GetSkeletonCap();

  float pos[3], ori[9], proj[2], pos_confidence, ori_confidence;

  SET_JTF(id, XN_SKEL_HEAD, "head", head);
  SET_JTF(id, XN_SKEL_NECK, "neck", neck);
  SET_JTF(id, XN_SKEL_TORSO, "torso", torso);
  SET_JTF(id, XN_SKEL_WAIST, "waist", waist);
  SET_JTF(id, XN_SKEL_LEFT_COLLAR, "left collar", left_collar);
  SET_JTF(id, XN_SKEL_LEFT_SHOULDER, "left shoulder", left_shoulder);
  SET_JTF(id, XN_SKEL_LEFT_ELBOW, "left elbow", left_elbow);
  SET_JTF(id, XN_SKEL_LEFT_WRIST, "left wrist", left_wrist);
  SET_JTF(id, XN_SKEL_LEFT_HAND, "left hand", left_hand);
  SET_JTF(id, XN_SKEL_LEFT_FINGERTIP, "left finger tip", left_fingertip);
  SET_JTF(id, XN_SKEL_RIGHT_COLLAR, "right collar", right_collar);
  SET_JTF(id, XN_SKEL_RIGHT_SHOULDER, "right shoulder", right_shoulder);
  SET_JTF(id, XN_SKEL_RIGHT_ELBOW, "right elbow", right_elbow);
  SET_JTF(id, XN_SKEL_RIGHT_WRIST, "right wrist", right_wrist);
  SET_JTF(id, XN_SKEL_RIGHT_HAND, "right hand", right_hand);
  SET_JTF(id, XN_SKEL_RIGHT_FINGERTIP, "right finger tip", right_fingertip);
  SET_JTF(id, XN_SKEL_LEFT_HIP, "left hip", left_hip);
  SET_JTF(id, XN_SKEL_LEFT_KNEE, "left knee", left_knee);
  SET_JTF(id, XN_SKEL_LEFT_ANKLE, "left ankle", left_ankle);
  SET_JTF(id, XN_SKEL_LEFT_FOOT, "left foot", left_foot);
  SET_JTF(id, XN_SKEL_RIGHT_HIP, "right hip", right_hip);
  SET_JTF(id, XN_SKEL_RIGHT_KNEE, "right knee", right_knee);
  SET_JTF(id, XN_SKEL_RIGHT_ANKLE, "right ankle", right_ankle);
  SET_JTF(id, XN_SKEL_RIGHT_FOOT, "right foot", right_foot);

  XnPoint3D compt, compt_proj;
  if (__user_gen->GetCoM(id, compt) == XN_STATUS_OK) {
    float com[3], com_proj[2];

    com[0] = compt.X;
    com[1] = compt.Y;
    com[2] = compt.Z;

    __depth_gen->ConvertRealWorldToProjective(1, &compt, &compt_proj);
    com_proj[0] = compt_proj.X;
    com_proj[1] = compt_proj.Y;

    user.skel_if->set_com(com);
    user.proj_if->set_proj_com(com_proj);
  }

}


/** Notify of new user.
 * This is called by the OpenNI callback when a new user has been detected.
 * @param id new user's ID
 */
void
OpenNiUserTrackerThread::new_user(XnUserID id)
{
  if (__users.find(id) != __users.end()) {
    logger->log_error(name(), "New user ID %u, but interface already exists", id);
    return;
  }

  char *ifid;
  if (asprintf(&ifid, "OpenNI Human %u", id) == -1) {
    logger->log_warn(name(), "New user ID %u, but cannot generate "
		     "interface ID", id);
    return;
  }
  try {
    logger->log_debug(name(), "Opening interface 'HumanSkeletonInterface::%s'", ifid);
    __users[id].skel_if = blackboard->open_for_writing<HumanSkeletonInterface>(ifid);
    __users[id].skel_if->set_user_id(id);
    __users[id].skel_if->write();
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to open interface, exception follows");
    logger->log_warn(name(), e);
  }

  try {
    logger->log_debug(name(), "Opening interface 'HumanSkeletonProjectionInterface::%s'", ifid);
    __users[id].proj_if = blackboard->open_for_writing<HumanSkeletonProjectionInterface>(ifid);
    XnFieldOfView fov;
    XnStatus st;
    if ((st = __depth_gen->GetFieldOfView(fov)) != XN_STATUS_OK) {
      logger->log_error(name(), "Failed to get field of view, ignoring. (%s)",
			xnGetStatusString(st));
    } else {
      __users[id].proj_if->set_horizontal_fov(fov.fHFOV);
      __users[id].proj_if->set_vertical_fov(fov.fVFOV);
    }

    xn::DepthMetaData dmd;
    __depth_gen->GetMetaData(dmd);
    __users[id].proj_if->set_res_x(dmd.XRes());
    __users[id].proj_if->set_res_y(dmd.YRes());
    __users[id].proj_if->set_max_depth(__depth_gen->GetDeviceMaxDepth());
    __users[id].proj_if->write();
  } catch (Exception &e) {
    blackboard->close(__users[id].proj_if);
    __users.erase(id);
    logger->log_warn(name(), "Failed to open interface, exception follows");
    logger->log_warn(name(), e);
  }


  free(ifid);

  if (__skel_need_calib_pose) {
    __user_gen->GetPoseDetectionCap().StartPoseDetection(__calib_pose_name, id);
  } else {
    __user_gen->GetSkeletonCap().RequestCalibration(id, TRUE);
  }
}


/** Notify of lost user.
 * This is called by the OpenNI callback when a user has been lost,
 * i.e. it has not been visible for some time.
 * @param id lost user's ID
 */
void
OpenNiUserTrackerThread::lost_user(XnUserID id)
{
  if (__users.find(id) == __users.end()) {
    logger->log_error(name(), "Lost user ID %u, but interface does not exist", id);
    return;
  }

  logger->log_error(name(), "Lost user ID %u, closing interface '%s'",
		    id, __users[id].skel_if->uid());
  blackboard->close(__users[id].skel_if);
  blackboard->close(__users[id].proj_if);
  __users.erase(id);
}


/** Notify of detected pose.
 * This is called if a pose has been detected.
 * @param id ID of user who is in the pose
 * @param pose_name name of the detected pose
 */
void
OpenNiUserTrackerThread::pose_start(XnUserID id, const char *pose_name)
{
  if (__users.find(id) == __users.end()) {
    logger->log_error(name(), "Pose start for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  logger->log_info(name(), "Pose %s detected for user %u", pose_name, id);

  __users[id].skel_if->set_pose(pose_name);
  __user_gen->GetPoseDetectionCap().StopPoseDetection(id);
  __user_gen->GetSkeletonCap().RequestCalibration(id, TRUE);
}

/** Notify of pose detection end.
 * This is called if a pose is no longer detected. The NITE middleware seems
 * not to call this.
 * @param id ID of user who is in the pose
 * @param pose_name name of the no longer detected pose
 */
void
OpenNiUserTrackerThread::pose_end(XnUserID id, const char *pose_name)
{
  if (__users.find(id) == __users.end()) {
    logger->log_error(name(), "Pose end for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  __users[id].skel_if->set_pose("");
}

/** Notify of calibration start.
 * This is called when tracking for a user has been started.
 * @param id ID of user who is being calibrated.
 */
void
OpenNiUserTrackerThread::calibration_start(XnUserID id)
{
  if (__users.find(id) == __users.end()) {
    logger->log_error(name(), "Pose end for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  logger->log_info(name(), "Calibration started for user %u", id);
}


/** Notify of calibration end.
 * This is called when tracking for a user has finished.
 * @param id ID of user who was being calibrated
 * @param success true if the calibration was successful, false otherwise
 */
void
OpenNiUserTrackerThread::calibration_end(XnUserID id, bool success)
{
  if (__users.find(id) == __users.end()) {
    logger->log_error(name(), "Pose end for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  __users[id].skel_if->set_pose("");

  if (success) {
    logger->log_info(name(), "Calibration successful for user %u, "
		     "starting tracking", id);
    __user_gen->GetSkeletonCap().StartTracking(id);
  } else {
    logger->log_info(name(), "Calibration failed for user %u, restarting", id);
    if (__skel_need_calib_pose) {
      __user_gen->GetPoseDetectionCap().StartPoseDetection(__calib_pose_name, id);
    } else {
      __user_gen->GetSkeletonCap().RequestCalibration(id, TRUE);
    }
  }
}
