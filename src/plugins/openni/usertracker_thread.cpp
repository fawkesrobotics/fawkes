
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
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <interfaces/HumanSkeletonInterface.h>
#include <interfaces/HumanSkeletonProjectionInterface.h>
#include <fvutils/ipc/shm_image.h>

#include <memory>

using namespace fawkes;
using namespace firevision;

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

#if XN_VERSION_GE(1,3,2,0)
static void XN_CALLBACK_TYPE
cb_calibration_complete(xn::SkeletonCapability &capability, XnUserID id,
                        XnCalibrationStatus status, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->calibration_end(id, status == XN_CALIBRATION_STATUS_OK);
}
#else
static void XN_CALLBACK_TYPE
cb_calibration_end(xn::SkeletonCapability &capability, XnUserID id,
		   XnBool success, void *cookie)
{
  OpenNiUserTrackerThread *t = static_cast<OpenNiUserTrackerThread *>(cookie);
  t->calibration_end(id, success);
}
#endif


void
OpenNiUserTrackerThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  user_gen_ = new xn::UserGenerator();
  std::auto_ptr<xn::UserGenerator> usergen_autoptr(user_gen_);

  depth_gen_ = new xn::DepthGenerator();
  std::auto_ptr<xn::DepthGenerator> depthgen_autoptr(depth_gen_);

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, depth_gen_);
  fawkes::openni::setup_map_generator(*depth_gen_, config);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_USER, user_gen_);

  if (!user_gen_->IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
    throw Exception("User generator does not support skeleton capability");
  }

  scene_md_ = new xn::SceneMetaData();
  std::auto_ptr<xn::SceneMetaData> scenemd_autoptr(scene_md_);
  if ((st = user_gen_->GetUserPixels(0, *scene_md_)) != XN_STATUS_OK) {
    throw Exception("Failed to get scene meta data (%s)", xnGetStatusString(st));
  }

  st = user_gen_->RegisterUserCallbacks(cb_new_user, cb_lost_user,
					 this, user_cb_handle_);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register user callbacks (%s)",
		    xnGetStatusString(st));
  }

  skelcap_ = new xn::SkeletonCapability(user_gen_->GetSkeletonCap());

#if XN_VERSION_GE(1,3,2,0)
  st = skelcap_->RegisterToCalibrationStart(cb_calibration_start,
                                             this, calib_start_cb_handle_);
  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register calibration start event (%s)",
                    xnGetStatusString(st));
  }
  st = skelcap_->RegisterToCalibrationComplete(cb_calibration_complete,
                                                this, calib_complete_cb_handle_);
#else
  st = skelcap_->RegisterCalibrationCallbacks(cb_calibration_start,
					       cb_calibration_end,
					       this, calib_cb_handle_);
#endif

  if (st != XN_STATUS_OK) {
    throw Exception("Failed to register calibration callback (%s)",
                    xnGetStatusString(st));
  }

  skel_need_calib_pose_ = skelcap_->NeedPoseForCalibration();

  if (skel_need_calib_pose_) {
    if (! user_gen_->IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
      throw Exception("Calibration requires pose, but not supported by node");
    }
    skelcap_->GetCalibrationPose(calib_pose_name_);

    xn::PoseDetectionCapability posecap = user_gen_->GetPoseDetectionCap();

#if XN_VERSION_GE(1,3,2,0)
    st = posecap.RegisterToPoseDetected(cb_pose_start,
                                        this, pose_start_cb_handle_);
    if (st != XN_STATUS_OK) {
      throw Exception("Failed to register pose detect event (%s)",
                      xnGetStatusString(st));
    }
    st = posecap.RegisterToOutOfPose(cb_pose_end,
                                     this, pose_end_cb_handle_);
#else
    st = posecap.RegisterToPoseCallbacks(cb_pose_start, cb_pose_end,
					   this, pose_cb_handle_);
#endif
    if (st != XN_STATUS_OK) {
      throw Exception("Failed to register pose callbacks (%s)", xnGetStatusString(st));
    }
  }

  skelcap_->SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  depth_gen_->StartGenerating();
  user_gen_->StartGenerating();

  label_buf_ = new SharedMemoryImageBuffer("openni-labels", RAW16,
  					    scene_md_->XRes(),
					    scene_md_->YRes());
  label_bufsize_ = colorspace_buffer_size(RAW16,
					   scene_md_->XRes(), scene_md_->YRes());

  usergen_autoptr.release();
  depthgen_autoptr.release();
  scenemd_autoptr.release();
}


void
OpenNiUserTrackerThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete user_gen_;
  delete scene_md_;
  delete skelcap_;
  delete label_buf_;

  UserMap::iterator i;
  for (i = users_.begin(); i != users_.end(); ++i) {
    blackboard->close(i->second.skel_if);
    blackboard->close(i->second.proj_if);
  }
}


void
OpenNiUserTrackerThread::loop()
{
  // we do not lock here, we are only operating on our user generator copy
  // and the update happens in a different main loop hook

  if (! user_gen_->IsDataNew())  return;

  UserMap::iterator i;
  for (i = users_.begin(); i != users_.end(); ++i) {

    if (!i->second.valid)  continue;

    bool needs_write = false;

    HumanSkeletonInterface::State new_state = i->second.skel_if->state();
    if (skelcap_->IsTracking(i->first)) {
      new_state = HumanSkeletonInterface::STATE_TRACKING;
    } else if (skelcap_->IsCalibrating(i->first)) {
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
	update_com(i->first, i->second);
	needs_write = true;
      } catch (Exception &e) {
	logger->log_warn(name(), "Failed to update skeleton data for %u, "
			 "exception follows", i->first);
	logger->log_warn(name(), e);
      }
    } else if (new_state == HumanSkeletonInterface::STATE_DETECTING_POSE) {
      update_com(i->first, i->second);
      needs_write = true;
    } else if (new_state == HumanSkeletonInterface::STATE_CALIBRATING) {
      update_com(i->first, i->second);
      needs_write = true;
    }

    if (needs_write) {
      i->second.skel_if->write();
      i->second.proj_if->write();
    }
  }

  if (label_buf_->num_attached() > 1) {
    memcpy(label_buf_->buffer(), scene_md_->Data(), label_bufsize_);
  }

}




// Very noisy when added to st != XN_STATUS_OK case
//logger->log_warn(name(), "Failed to get joint transformation for "
//		     "%s joint (%s)",
//		     joint_name, xnGetStatusString(st));


// change from mm to m
// translating to Fawkes coordinates, empirically verified
// permute ori columns to match our coordinate system, empirically verified
#define SET_JTF(id, joint, joint_name, bbfield)				\
  st = skelcap_->GetSkeletonJoint(id, joint, jtf);			\
  if (st != XN_STATUS_OK) {						\
    ori[0] = ori[1] = ori[2] = ori[3] = ori[4] = ori[5] = 0.;		\
    ori[6] = ori[7] = ori[8] = ori_confidence = pos_confidence = 0.;	\
    proj[0] = proj[1] = 0;						\
  } else {								\
    pos[0] =  jtf.position.position.Z * 0.001;				\
    pos[1] = -jtf.position.position.X * 0.001;				\
    pos[2] =  jtf.position.position.Y * 0.001;				\
    pos_confidence = jtf.position.fConfidence;				\
									\
    ori[0] =  jtf.orientation.orientation.elements[2];			\
    ori[1] = -jtf.orientation.orientation.elements[0];			\
    ori[2] =  jtf.orientation.orientation.elements[1];			\
    ori[3] =  jtf.orientation.orientation.elements[5];			\
    ori[4] = -jtf.orientation.orientation.elements[3];			\
    ori[5] =  jtf.orientation.orientation.elements[4];			\
    ori[6] =  jtf.orientation.orientation.elements[8];			\
    ori[7] = -jtf.orientation.orientation.elements[6];			\
    ori[8] =  jtf.orientation.orientation.elements[7];			\
    ori_confidence = jtf.orientation.fConfidence;			\
									\
    XnPoint3D pt;							\
    pt = jtf.position.position;						\
    depth_gen_->ConvertRealWorldToProjective(1, &pt, &pt);		\
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

}


void
OpenNiUserTrackerThread::update_com(XnUserID id, UserInfo &user)
{
  XnPoint3D compt, compt_proj;
  XnStatus st;
  float com[3], com_proj[2];
  com[0] = com[1] = com[2] = com_proj[0] = com_proj[1] = 0.;
  if ((st = user_gen_->GetCoM(id, compt)) == XN_STATUS_OK) {

    // translating to Fawkes coordinates, empirically verified
    com[0] =  compt.Z * 0.001;
    com[1] = -compt.X * 0.001;
    com[2] =  compt.Y * 0.001;

    depth_gen_->ConvertRealWorldToProjective(1, &compt, &compt_proj);
    com_proj[0] = compt_proj.X;
    com_proj[1] = compt_proj.Y;
  } else {
    logger->log_warn(name(), "GetCoM failed: %s", xnGetStatusString(st));
  }

  user.skel_if->set_com(com);
  user.proj_if->set_proj_com(com_proj);

  int current_vishist = user.skel_if->visibility_history();
  if ((com[0] == 0.) && (com[1] == 0.) && (com[2] == 0.)) {
    if (current_vishist < 0) {
      user.skel_if->set_visibility_history(--current_vishist);
    } else {
      user.skel_if->set_visibility_history(-1);
    }
  } else {
    if (current_vishist > 0) {
      user.skel_if->set_visibility_history(++current_vishist);
    } else {
      user.skel_if->set_visibility_history(1);
    }
  }
}

/** Notify of new user.
 * This is called by the OpenNI callback when a new user has been detected.
 * @param id new user's ID
 */
void
OpenNiUserTrackerThread::new_user(XnUserID id)
{
  if (users_.find(id) != users_.end()) {
    logger->log_error(name(), "New user ID %u, interface already exists", id);
  } else {
    char *ifid;
    if (asprintf(&ifid, "OpenNI Human %u", id) == -1) {
      logger->log_warn(name(), "New user ID %u, but cannot generate "
		       "interface ID", id);
      return;
    }
    try {
      logger->log_debug(name(), "Opening interface 'HumanSkeletonInterface::%s'", ifid);
      users_[id].skel_if = blackboard->open_for_writing<HumanSkeletonInterface>(ifid);
      users_[id].skel_if->set_user_id(id);
      users_[id].skel_if->write();
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to open interface, exception follows");
      logger->log_warn(name(), e);
    }

    try {
      logger->log_debug(name(), "Opening interface 'HumanSkeletonProjectionInterface::%s'", ifid);
      users_[id].proj_if = blackboard->open_for_writing<HumanSkeletonProjectionInterface>(ifid);
      XnFieldOfView fov;
      XnStatus st;
      if ((st = depth_gen_->GetFieldOfView(fov)) != XN_STATUS_OK) {
        logger->log_error(name(), "Failed to get field of view, ignoring. (%s)",
			  xnGetStatusString(st));
      } else {
        users_[id].proj_if->set_horizontal_fov(fov.fHFOV);
        users_[id].proj_if->set_vertical_fov(fov.fVFOV);
      }

      xn::DepthMetaData dmd;
      depth_gen_->GetMetaData(dmd);
      users_[id].proj_if->set_res_x(dmd.XRes());
      users_[id].proj_if->set_res_y(dmd.YRes());
      users_[id].proj_if->set_max_depth(depth_gen_->GetDeviceMaxDepth());
      users_[id].proj_if->write();
    } catch (Exception &e) {
      blackboard->close(users_[id].proj_if);
      users_.erase(id);
      logger->log_warn(name(), "Failed to open interface, exception follows");
      logger->log_warn(name(), e);
    }

    free(ifid);
  }

  users_[id].valid = true;

  if (skel_need_calib_pose_) {
    user_gen_->GetPoseDetectionCap().StartPoseDetection(calib_pose_name_, id);
  } else {
    user_gen_->GetSkeletonCap().RequestCalibration(id, TRUE);
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
  if (users_.find(id) == users_.end()) {
    logger->log_error(name(), "Lost user ID %u, but interface does not exist", id);
    return;
  }

  logger->log_error(name(), "Lost user ID %u, setting interface '%s' to invalid",
		    id, users_[id].skel_if->uid());
  // write invalid, a reader might still be open
  users_[id].skel_if->set_state(HumanSkeletonInterface::STATE_INVALID);
  users_[id].skel_if->write();
  users_[id].valid = false;
  //blackboard->close(users_[id].skel_if);
  //blackboard->close(users_[id].proj_if);
  //users_.erase(id);
}


/** Notify of detected pose.
 * This is called if a pose has been detected.
 * @param id ID of user who is in the pose
 * @param pose_name name of the detected pose
 */
void
OpenNiUserTrackerThread::pose_start(XnUserID id, const char *pose_name)
{
  if (users_.find(id) == users_.end()) {
    logger->log_error(name(), "Pose start for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  logger->log_info(name(), "Pose %s detected for user %u", pose_name, id);

  users_[id].skel_if->set_pose(pose_name);
  user_gen_->GetPoseDetectionCap().StopPoseDetection(id);
  user_gen_->GetSkeletonCap().RequestCalibration(id, TRUE);
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
  if (users_.find(id) == users_.end()) {
    logger->log_error(name(), "Pose end for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  users_[id].skel_if->set_pose("");
}

/** Notify of calibration start.
 * This is called when tracking for a user has been started.
 * @param id ID of user who is being calibrated.
 */
void
OpenNiUserTrackerThread::calibration_start(XnUserID id)
{
  if (users_.find(id) == users_.end()) {
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
  if (users_.find(id) == users_.end()) {
    logger->log_error(name(), "Pose end for user ID %u, "
		      "but interface does not exist", id);
    return;
  }

  users_[id].skel_if->set_pose("");

  if (success) {
    logger->log_info(name(), "Calibration successful for user %u, "
		     "starting tracking", id);
    user_gen_->GetSkeletonCap().StartTracking(id);
  } else {
    logger->log_info(name(), "Calibration failed for user %u, restarting", id);
    if (skel_need_calib_pose_) {
      user_gen_->GetPoseDetectionCap().StartPoseDetection(calib_pose_name_, id);
    } else {
      user_gen_->GetSkeletonCap().RequestCalibration(id, TRUE);
    }
  }
}
