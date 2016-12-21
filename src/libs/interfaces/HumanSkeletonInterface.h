
/***************************************************************************
 *  HumanSkeletonInterface.h - Fawkes BlackBoard Interface - HumanSkeletonInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2011  Tim Niemueller
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __INTERFACES_HUMANSKELETONINTERFACE_H_
#define __INTERFACES_HUMANSKELETONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class HumanSkeletonInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(HumanSkeletonInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Current tracking state for the skeleton.
       */
  typedef enum {
    STATE_INVALID /**< 
        This interface does not represent a valid skeleton at the moment.
       */,
    STATE_DETECTING_POSE /**< 
	The user's pose is currently being determined. This usually indicates
	that the tracker is looking for a particular calibration pose.
       */,
    STATE_CALIBRATING /**< 
        The tracker is currently calibrating for the recognized human.
       */,
    STATE_TRACKING /**< 
        The user is being tracked and the skeleton contains valid data.
       */
  } State;
  const char * tostring_State(State value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t state; /**< Current state. */
    uint32_t user_id; /**< Tracking ID of this user. */
    int32_t visibility_history; /**< 
      The visibility history indicates the persistence of user sightings.
      A positive value indicates the number of successful consecutive sightings
      of the user (center of mass not equal to zero), the absolute of a negative
      value gives the number of consecutive negative (non-) sightings. The value
      is zero only if uninitialized.
     */
    char pose[32]; /**< Detected user pose. */
    float com[3]; /**< Center of mass. */
    float pos_head[3]; /**< Head position vector. */
    float pos_head_confidence; /**< 
      Head position confidence. */
    float pos_neck[3]; /**< Neck position vector. */
    float pos_neck_confidence; /**< 
      Neck position confidence. */
    float pos_torso[3]; /**< Torso position vector. */
    float pos_torso_confidence; /**< 
      Torso position confidence. */
    float pos_waist[3]; /**< Waist position vector. */
    float pos_waist_confidence; /**< 
      Waist position confidence. */
    float pos_left_collar[3]; /**< 
      Left position vector. */
    float pos_left_collar_confidence; /**< 
      Left position confidence. */
    float pos_left_shoulder[3]; /**< 
      Left shoulder position vector. */
    float pos_left_shoulder_confidence; /**< 
      Left shoulder position confidence. */
    float pos_left_elbow[3]; /**< 
      Left elbow position vector. */
    float pos_left_elbow_confidence; /**< 
      Left elbow position confidence. */
    float pos_left_wrist[3]; /**< 
      Left wrist position vector. */
    float pos_left_wrist_confidence; /**< 
      Left wrist position confidence. */
    float pos_left_hand[3]; /**< 
      Left hand position vector. */
    float pos_left_hand_confidence; /**< 
      Left hand position confidence. */
    float pos_left_fingertip[3]; /**< 
      Left fingertip position vector. */
    float pos_left_fingertip_confidence; /**< 
      Left fingertip position confidence. */
    float pos_right_collar[3]; /**< 
      Right collar position vector. */
    float pos_right_collar_confidence; /**< 
      Right collar position confidence. */
    float pos_right_shoulder[3]; /**< 
      Right shoulder position vector. */
    float pos_right_shoulder_confidence; /**< 
      Right shoulder position confidence. */
    float pos_right_elbow[3]; /**< 
      Right elbow position vector. */
    float pos_right_elbow_confidence; /**< 
      Right elbow position confidence. */
    float pos_right_wrist[3]; /**< 
      Right wrist position vector. */
    float pos_right_wrist_confidence; /**< 
      Right wrist position confidence. */
    float pos_right_hand[3]; /**< 
      Right hand position vector. */
    float pos_right_hand_confidence; /**< 
      Right hand position confidence. */
    float pos_right_fingertip[3]; /**< 
      Right fingertip position vector. */
    float pos_right_fingertip_confidence; /**< 
      Right fingertip position confidence. */
    float pos_left_hip[3]; /**< 
      Left hip position vector. */
    float pos_left_hip_confidence; /**< 
      Left hip position confidence. */
    float pos_left_knee[3]; /**< 
      Left knee position vector. */
    float pos_left_knee_confidence; /**< 
      Left knee position confidence. */
    float pos_left_ankle[3]; /**< 
      Left ankle position vector. */
    float pos_left_ankle_confidence; /**< 
      Left ankle position confidence. */
    float pos_left_foot[3]; /**< 
      Left foot position vector. */
    float pos_left_foot_confidence; /**< 
      Left foot position confidence. */
    float pos_right_hip[3]; /**< 
      Right hip position vector. */
    float pos_right_hip_confidence; /**< 
      Right hip position confidence. */
    float pos_right_knee[3]; /**< 
      Right knee position vector. */
    float pos_right_knee_confidence; /**< 
      Right knee position confidence. */
    float pos_right_ankle[3]; /**< 
      Right ankle position vector. */
    float pos_right_ankle_confidence; /**< 
      Right ankle position confidence. */
    float pos_right_foot[3]; /**< 
      Right foot position vector. */
    float pos_right_foot_confidence; /**< 
      Right foot position confidence. */
    float ori_head[9]; /**< Head position vector. */
    float ori_head_confidence; /**< 
      Head position confidence. */
    float ori_neck[9]; /**< Neck position vector. */
    float ori_neck_confidence; /**< 
      Neck position confidence. */
    float ori_torso[9]; /**< Torso position vector. */
    float ori_torso_confidence; /**< 
      Torso position confidence. */
    float ori_waist[9]; /**< Waist position vector. */
    float ori_waist_confidence; /**< 
      Waist position confidence. */
    float ori_left_collar[9]; /**< 
      Left position vector. */
    float ori_left_collar_confidence; /**< 
      Left position confidence. */
    float ori_left_shoulder[9]; /**< 
      Left shoulder position vector. */
    float ori_left_shoulder_confidence; /**< 
      Left shoulder position confidence. */
    float ori_left_elbow[9]; /**< 
      Left elbow position vector. */
    float ori_left_elbow_confidence; /**< 
      Left elbow position confidence. */
    float ori_left_wrist[9]; /**< 
      Left wrist position vector. */
    float ori_left_wrist_confidence; /**< 
      Left wrist position confidence. */
    float ori_left_hand[9]; /**< 
      Left hand position vector. */
    float ori_left_hand_confidence; /**< 
      Left hand position confidence. */
    float ori_left_fingertip[9]; /**< 
      Left fingertip position vector. */
    float ori_left_fingertip_confidence; /**< 
      Left fingertip position confidence. */
    float ori_right_collar[9]; /**< 
      Right collar position vector. */
    float ori_right_collar_confidence; /**< 
      Right collar position confidence. */
    float ori_right_shoulder[9]; /**< 
      Right shoulder position vector. */
    float ori_right_shoulder_confidence; /**< 
      Right shoulder position confidence. */
    float ori_right_elbow[9]; /**< 
      Right elbow position vector. */
    float ori_right_elbow_confidence; /**< 
      Right elbow position confidence. */
    float ori_right_wrist[9]; /**< 
      Right wrist position vector. */
    float ori_right_wrist_confidence; /**< 
      Right wrist position confidence. */
    float ori_right_hand[9]; /**< 
      Right hand position vector. */
    float ori_right_hand_confidence; /**< 
      Right hand position confidence. */
    float ori_right_fingertip[9]; /**< 
      Right fingertip position vector. */
    float ori_right_fingertip_confidence; /**< 
      Right fingertip position confidence. */
    float ori_left_hip[9]; /**< 
      Left hip position vector. */
    float ori_left_hip_confidence; /**< 
      Left hip position confidence. */
    float ori_left_knee[9]; /**< 
      Left knee position vector. */
    float ori_left_knee_confidence; /**< 
      Left knee position confidence. */
    float ori_left_ankle[9]; /**< 
      Left ankle position vector. */
    float ori_left_ankle_confidence; /**< 
      Left ankle position confidence. */
    float ori_left_foot[9]; /**< 
      Left foot position vector. */
    float ori_left_foot_confidence; /**< 
      Left foot position confidence. */
    float ori_right_hip[9]; /**< 
      Right hip position vector. */
    float ori_right_hip_confidence; /**< 
      Right hip position confidence. */
    float ori_right_knee[9]; /**< 
      Right knee position vector. */
    float ori_right_knee_confidence; /**< 
      Right knee position confidence. */
    float ori_right_ankle[9]; /**< 
      Right ankle position vector. */
    float ori_right_ankle_confidence; /**< 
      Right ankle position confidence. */
    float ori_right_foot[9]; /**< 
      Right foot position vector. */
    float ori_right_foot_confidence; /**< 
      Right foot position confidence. */
  } HumanSkeletonInterface_data_t;

  HumanSkeletonInterface_data_t *data;

  interface_enum_map_t enum_map_State;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  HumanSkeletonInterface();
  ~HumanSkeletonInterface();

 public:
  /* Methods */
  State state() const;
  void set_state(const State new_state);
  size_t maxlenof_state() const;
  uint32_t user_id() const;
  void set_user_id(const uint32_t new_user_id);
  size_t maxlenof_user_id() const;
  int32_t visibility_history() const;
  void set_visibility_history(const int32_t new_visibility_history);
  size_t maxlenof_visibility_history() const;
  char * pose() const;
  void set_pose(const char * new_pose);
  size_t maxlenof_pose() const;
  float * com() const;
  float com(unsigned int index) const;
  void set_com(unsigned int index, const float new_com);
  void set_com(const float * new_com);
  size_t maxlenof_com() const;
  float * pos_head() const;
  float pos_head(unsigned int index) const;
  void set_pos_head(unsigned int index, const float new_pos_head);
  void set_pos_head(const float * new_pos_head);
  size_t maxlenof_pos_head() const;
  float pos_head_confidence() const;
  void set_pos_head_confidence(const float new_pos_head_confidence);
  size_t maxlenof_pos_head_confidence() const;
  float * pos_neck() const;
  float pos_neck(unsigned int index) const;
  void set_pos_neck(unsigned int index, const float new_pos_neck);
  void set_pos_neck(const float * new_pos_neck);
  size_t maxlenof_pos_neck() const;
  float pos_neck_confidence() const;
  void set_pos_neck_confidence(const float new_pos_neck_confidence);
  size_t maxlenof_pos_neck_confidence() const;
  float * pos_torso() const;
  float pos_torso(unsigned int index) const;
  void set_pos_torso(unsigned int index, const float new_pos_torso);
  void set_pos_torso(const float * new_pos_torso);
  size_t maxlenof_pos_torso() const;
  float pos_torso_confidence() const;
  void set_pos_torso_confidence(const float new_pos_torso_confidence);
  size_t maxlenof_pos_torso_confidence() const;
  float * pos_waist() const;
  float pos_waist(unsigned int index) const;
  void set_pos_waist(unsigned int index, const float new_pos_waist);
  void set_pos_waist(const float * new_pos_waist);
  size_t maxlenof_pos_waist() const;
  float pos_waist_confidence() const;
  void set_pos_waist_confidence(const float new_pos_waist_confidence);
  size_t maxlenof_pos_waist_confidence() const;
  float * pos_left_collar() const;
  float pos_left_collar(unsigned int index) const;
  void set_pos_left_collar(unsigned int index, const float new_pos_left_collar);
  void set_pos_left_collar(const float * new_pos_left_collar);
  size_t maxlenof_pos_left_collar() const;
  float pos_left_collar_confidence() const;
  void set_pos_left_collar_confidence(const float new_pos_left_collar_confidence);
  size_t maxlenof_pos_left_collar_confidence() const;
  float * pos_left_shoulder() const;
  float pos_left_shoulder(unsigned int index) const;
  void set_pos_left_shoulder(unsigned int index, const float new_pos_left_shoulder);
  void set_pos_left_shoulder(const float * new_pos_left_shoulder);
  size_t maxlenof_pos_left_shoulder() const;
  float pos_left_shoulder_confidence() const;
  void set_pos_left_shoulder_confidence(const float new_pos_left_shoulder_confidence);
  size_t maxlenof_pos_left_shoulder_confidence() const;
  float * pos_left_elbow() const;
  float pos_left_elbow(unsigned int index) const;
  void set_pos_left_elbow(unsigned int index, const float new_pos_left_elbow);
  void set_pos_left_elbow(const float * new_pos_left_elbow);
  size_t maxlenof_pos_left_elbow() const;
  float pos_left_elbow_confidence() const;
  void set_pos_left_elbow_confidence(const float new_pos_left_elbow_confidence);
  size_t maxlenof_pos_left_elbow_confidence() const;
  float * pos_left_wrist() const;
  float pos_left_wrist(unsigned int index) const;
  void set_pos_left_wrist(unsigned int index, const float new_pos_left_wrist);
  void set_pos_left_wrist(const float * new_pos_left_wrist);
  size_t maxlenof_pos_left_wrist() const;
  float pos_left_wrist_confidence() const;
  void set_pos_left_wrist_confidence(const float new_pos_left_wrist_confidence);
  size_t maxlenof_pos_left_wrist_confidence() const;
  float * pos_left_hand() const;
  float pos_left_hand(unsigned int index) const;
  void set_pos_left_hand(unsigned int index, const float new_pos_left_hand);
  void set_pos_left_hand(const float * new_pos_left_hand);
  size_t maxlenof_pos_left_hand() const;
  float pos_left_hand_confidence() const;
  void set_pos_left_hand_confidence(const float new_pos_left_hand_confidence);
  size_t maxlenof_pos_left_hand_confidence() const;
  float * pos_left_fingertip() const;
  float pos_left_fingertip(unsigned int index) const;
  void set_pos_left_fingertip(unsigned int index, const float new_pos_left_fingertip);
  void set_pos_left_fingertip(const float * new_pos_left_fingertip);
  size_t maxlenof_pos_left_fingertip() const;
  float pos_left_fingertip_confidence() const;
  void set_pos_left_fingertip_confidence(const float new_pos_left_fingertip_confidence);
  size_t maxlenof_pos_left_fingertip_confidence() const;
  float * pos_right_collar() const;
  float pos_right_collar(unsigned int index) const;
  void set_pos_right_collar(unsigned int index, const float new_pos_right_collar);
  void set_pos_right_collar(const float * new_pos_right_collar);
  size_t maxlenof_pos_right_collar() const;
  float pos_right_collar_confidence() const;
  void set_pos_right_collar_confidence(const float new_pos_right_collar_confidence);
  size_t maxlenof_pos_right_collar_confidence() const;
  float * pos_right_shoulder() const;
  float pos_right_shoulder(unsigned int index) const;
  void set_pos_right_shoulder(unsigned int index, const float new_pos_right_shoulder);
  void set_pos_right_shoulder(const float * new_pos_right_shoulder);
  size_t maxlenof_pos_right_shoulder() const;
  float pos_right_shoulder_confidence() const;
  void set_pos_right_shoulder_confidence(const float new_pos_right_shoulder_confidence);
  size_t maxlenof_pos_right_shoulder_confidence() const;
  float * pos_right_elbow() const;
  float pos_right_elbow(unsigned int index) const;
  void set_pos_right_elbow(unsigned int index, const float new_pos_right_elbow);
  void set_pos_right_elbow(const float * new_pos_right_elbow);
  size_t maxlenof_pos_right_elbow() const;
  float pos_right_elbow_confidence() const;
  void set_pos_right_elbow_confidence(const float new_pos_right_elbow_confidence);
  size_t maxlenof_pos_right_elbow_confidence() const;
  float * pos_right_wrist() const;
  float pos_right_wrist(unsigned int index) const;
  void set_pos_right_wrist(unsigned int index, const float new_pos_right_wrist);
  void set_pos_right_wrist(const float * new_pos_right_wrist);
  size_t maxlenof_pos_right_wrist() const;
  float pos_right_wrist_confidence() const;
  void set_pos_right_wrist_confidence(const float new_pos_right_wrist_confidence);
  size_t maxlenof_pos_right_wrist_confidence() const;
  float * pos_right_hand() const;
  float pos_right_hand(unsigned int index) const;
  void set_pos_right_hand(unsigned int index, const float new_pos_right_hand);
  void set_pos_right_hand(const float * new_pos_right_hand);
  size_t maxlenof_pos_right_hand() const;
  float pos_right_hand_confidence() const;
  void set_pos_right_hand_confidence(const float new_pos_right_hand_confidence);
  size_t maxlenof_pos_right_hand_confidence() const;
  float * pos_right_fingertip() const;
  float pos_right_fingertip(unsigned int index) const;
  void set_pos_right_fingertip(unsigned int index, const float new_pos_right_fingertip);
  void set_pos_right_fingertip(const float * new_pos_right_fingertip);
  size_t maxlenof_pos_right_fingertip() const;
  float pos_right_fingertip_confidence() const;
  void set_pos_right_fingertip_confidence(const float new_pos_right_fingertip_confidence);
  size_t maxlenof_pos_right_fingertip_confidence() const;
  float * pos_left_hip() const;
  float pos_left_hip(unsigned int index) const;
  void set_pos_left_hip(unsigned int index, const float new_pos_left_hip);
  void set_pos_left_hip(const float * new_pos_left_hip);
  size_t maxlenof_pos_left_hip() const;
  float pos_left_hip_confidence() const;
  void set_pos_left_hip_confidence(const float new_pos_left_hip_confidence);
  size_t maxlenof_pos_left_hip_confidence() const;
  float * pos_left_knee() const;
  float pos_left_knee(unsigned int index) const;
  void set_pos_left_knee(unsigned int index, const float new_pos_left_knee);
  void set_pos_left_knee(const float * new_pos_left_knee);
  size_t maxlenof_pos_left_knee() const;
  float pos_left_knee_confidence() const;
  void set_pos_left_knee_confidence(const float new_pos_left_knee_confidence);
  size_t maxlenof_pos_left_knee_confidence() const;
  float * pos_left_ankle() const;
  float pos_left_ankle(unsigned int index) const;
  void set_pos_left_ankle(unsigned int index, const float new_pos_left_ankle);
  void set_pos_left_ankle(const float * new_pos_left_ankle);
  size_t maxlenof_pos_left_ankle() const;
  float pos_left_ankle_confidence() const;
  void set_pos_left_ankle_confidence(const float new_pos_left_ankle_confidence);
  size_t maxlenof_pos_left_ankle_confidence() const;
  float * pos_left_foot() const;
  float pos_left_foot(unsigned int index) const;
  void set_pos_left_foot(unsigned int index, const float new_pos_left_foot);
  void set_pos_left_foot(const float * new_pos_left_foot);
  size_t maxlenof_pos_left_foot() const;
  float pos_left_foot_confidence() const;
  void set_pos_left_foot_confidence(const float new_pos_left_foot_confidence);
  size_t maxlenof_pos_left_foot_confidence() const;
  float * pos_right_hip() const;
  float pos_right_hip(unsigned int index) const;
  void set_pos_right_hip(unsigned int index, const float new_pos_right_hip);
  void set_pos_right_hip(const float * new_pos_right_hip);
  size_t maxlenof_pos_right_hip() const;
  float pos_right_hip_confidence() const;
  void set_pos_right_hip_confidence(const float new_pos_right_hip_confidence);
  size_t maxlenof_pos_right_hip_confidence() const;
  float * pos_right_knee() const;
  float pos_right_knee(unsigned int index) const;
  void set_pos_right_knee(unsigned int index, const float new_pos_right_knee);
  void set_pos_right_knee(const float * new_pos_right_knee);
  size_t maxlenof_pos_right_knee() const;
  float pos_right_knee_confidence() const;
  void set_pos_right_knee_confidence(const float new_pos_right_knee_confidence);
  size_t maxlenof_pos_right_knee_confidence() const;
  float * pos_right_ankle() const;
  float pos_right_ankle(unsigned int index) const;
  void set_pos_right_ankle(unsigned int index, const float new_pos_right_ankle);
  void set_pos_right_ankle(const float * new_pos_right_ankle);
  size_t maxlenof_pos_right_ankle() const;
  float pos_right_ankle_confidence() const;
  void set_pos_right_ankle_confidence(const float new_pos_right_ankle_confidence);
  size_t maxlenof_pos_right_ankle_confidence() const;
  float * pos_right_foot() const;
  float pos_right_foot(unsigned int index) const;
  void set_pos_right_foot(unsigned int index, const float new_pos_right_foot);
  void set_pos_right_foot(const float * new_pos_right_foot);
  size_t maxlenof_pos_right_foot() const;
  float pos_right_foot_confidence() const;
  void set_pos_right_foot_confidence(const float new_pos_right_foot_confidence);
  size_t maxlenof_pos_right_foot_confidence() const;
  float * ori_head() const;
  float ori_head(unsigned int index) const;
  void set_ori_head(unsigned int index, const float new_ori_head);
  void set_ori_head(const float * new_ori_head);
  size_t maxlenof_ori_head() const;
  float ori_head_confidence() const;
  void set_ori_head_confidence(const float new_ori_head_confidence);
  size_t maxlenof_ori_head_confidence() const;
  float * ori_neck() const;
  float ori_neck(unsigned int index) const;
  void set_ori_neck(unsigned int index, const float new_ori_neck);
  void set_ori_neck(const float * new_ori_neck);
  size_t maxlenof_ori_neck() const;
  float ori_neck_confidence() const;
  void set_ori_neck_confidence(const float new_ori_neck_confidence);
  size_t maxlenof_ori_neck_confidence() const;
  float * ori_torso() const;
  float ori_torso(unsigned int index) const;
  void set_ori_torso(unsigned int index, const float new_ori_torso);
  void set_ori_torso(const float * new_ori_torso);
  size_t maxlenof_ori_torso() const;
  float ori_torso_confidence() const;
  void set_ori_torso_confidence(const float new_ori_torso_confidence);
  size_t maxlenof_ori_torso_confidence() const;
  float * ori_waist() const;
  float ori_waist(unsigned int index) const;
  void set_ori_waist(unsigned int index, const float new_ori_waist);
  void set_ori_waist(const float * new_ori_waist);
  size_t maxlenof_ori_waist() const;
  float ori_waist_confidence() const;
  void set_ori_waist_confidence(const float new_ori_waist_confidence);
  size_t maxlenof_ori_waist_confidence() const;
  float * ori_left_collar() const;
  float ori_left_collar(unsigned int index) const;
  void set_ori_left_collar(unsigned int index, const float new_ori_left_collar);
  void set_ori_left_collar(const float * new_ori_left_collar);
  size_t maxlenof_ori_left_collar() const;
  float ori_left_collar_confidence() const;
  void set_ori_left_collar_confidence(const float new_ori_left_collar_confidence);
  size_t maxlenof_ori_left_collar_confidence() const;
  float * ori_left_shoulder() const;
  float ori_left_shoulder(unsigned int index) const;
  void set_ori_left_shoulder(unsigned int index, const float new_ori_left_shoulder);
  void set_ori_left_shoulder(const float * new_ori_left_shoulder);
  size_t maxlenof_ori_left_shoulder() const;
  float ori_left_shoulder_confidence() const;
  void set_ori_left_shoulder_confidence(const float new_ori_left_shoulder_confidence);
  size_t maxlenof_ori_left_shoulder_confidence() const;
  float * ori_left_elbow() const;
  float ori_left_elbow(unsigned int index) const;
  void set_ori_left_elbow(unsigned int index, const float new_ori_left_elbow);
  void set_ori_left_elbow(const float * new_ori_left_elbow);
  size_t maxlenof_ori_left_elbow() const;
  float ori_left_elbow_confidence() const;
  void set_ori_left_elbow_confidence(const float new_ori_left_elbow_confidence);
  size_t maxlenof_ori_left_elbow_confidence() const;
  float * ori_left_wrist() const;
  float ori_left_wrist(unsigned int index) const;
  void set_ori_left_wrist(unsigned int index, const float new_ori_left_wrist);
  void set_ori_left_wrist(const float * new_ori_left_wrist);
  size_t maxlenof_ori_left_wrist() const;
  float ori_left_wrist_confidence() const;
  void set_ori_left_wrist_confidence(const float new_ori_left_wrist_confidence);
  size_t maxlenof_ori_left_wrist_confidence() const;
  float * ori_left_hand() const;
  float ori_left_hand(unsigned int index) const;
  void set_ori_left_hand(unsigned int index, const float new_ori_left_hand);
  void set_ori_left_hand(const float * new_ori_left_hand);
  size_t maxlenof_ori_left_hand() const;
  float ori_left_hand_confidence() const;
  void set_ori_left_hand_confidence(const float new_ori_left_hand_confidence);
  size_t maxlenof_ori_left_hand_confidence() const;
  float * ori_left_fingertip() const;
  float ori_left_fingertip(unsigned int index) const;
  void set_ori_left_fingertip(unsigned int index, const float new_ori_left_fingertip);
  void set_ori_left_fingertip(const float * new_ori_left_fingertip);
  size_t maxlenof_ori_left_fingertip() const;
  float ori_left_fingertip_confidence() const;
  void set_ori_left_fingertip_confidence(const float new_ori_left_fingertip_confidence);
  size_t maxlenof_ori_left_fingertip_confidence() const;
  float * ori_right_collar() const;
  float ori_right_collar(unsigned int index) const;
  void set_ori_right_collar(unsigned int index, const float new_ori_right_collar);
  void set_ori_right_collar(const float * new_ori_right_collar);
  size_t maxlenof_ori_right_collar() const;
  float ori_right_collar_confidence() const;
  void set_ori_right_collar_confidence(const float new_ori_right_collar_confidence);
  size_t maxlenof_ori_right_collar_confidence() const;
  float * ori_right_shoulder() const;
  float ori_right_shoulder(unsigned int index) const;
  void set_ori_right_shoulder(unsigned int index, const float new_ori_right_shoulder);
  void set_ori_right_shoulder(const float * new_ori_right_shoulder);
  size_t maxlenof_ori_right_shoulder() const;
  float ori_right_shoulder_confidence() const;
  void set_ori_right_shoulder_confidence(const float new_ori_right_shoulder_confidence);
  size_t maxlenof_ori_right_shoulder_confidence() const;
  float * ori_right_elbow() const;
  float ori_right_elbow(unsigned int index) const;
  void set_ori_right_elbow(unsigned int index, const float new_ori_right_elbow);
  void set_ori_right_elbow(const float * new_ori_right_elbow);
  size_t maxlenof_ori_right_elbow() const;
  float ori_right_elbow_confidence() const;
  void set_ori_right_elbow_confidence(const float new_ori_right_elbow_confidence);
  size_t maxlenof_ori_right_elbow_confidence() const;
  float * ori_right_wrist() const;
  float ori_right_wrist(unsigned int index) const;
  void set_ori_right_wrist(unsigned int index, const float new_ori_right_wrist);
  void set_ori_right_wrist(const float * new_ori_right_wrist);
  size_t maxlenof_ori_right_wrist() const;
  float ori_right_wrist_confidence() const;
  void set_ori_right_wrist_confidence(const float new_ori_right_wrist_confidence);
  size_t maxlenof_ori_right_wrist_confidence() const;
  float * ori_right_hand() const;
  float ori_right_hand(unsigned int index) const;
  void set_ori_right_hand(unsigned int index, const float new_ori_right_hand);
  void set_ori_right_hand(const float * new_ori_right_hand);
  size_t maxlenof_ori_right_hand() const;
  float ori_right_hand_confidence() const;
  void set_ori_right_hand_confidence(const float new_ori_right_hand_confidence);
  size_t maxlenof_ori_right_hand_confidence() const;
  float * ori_right_fingertip() const;
  float ori_right_fingertip(unsigned int index) const;
  void set_ori_right_fingertip(unsigned int index, const float new_ori_right_fingertip);
  void set_ori_right_fingertip(const float * new_ori_right_fingertip);
  size_t maxlenof_ori_right_fingertip() const;
  float ori_right_fingertip_confidence() const;
  void set_ori_right_fingertip_confidence(const float new_ori_right_fingertip_confidence);
  size_t maxlenof_ori_right_fingertip_confidence() const;
  float * ori_left_hip() const;
  float ori_left_hip(unsigned int index) const;
  void set_ori_left_hip(unsigned int index, const float new_ori_left_hip);
  void set_ori_left_hip(const float * new_ori_left_hip);
  size_t maxlenof_ori_left_hip() const;
  float ori_left_hip_confidence() const;
  void set_ori_left_hip_confidence(const float new_ori_left_hip_confidence);
  size_t maxlenof_ori_left_hip_confidence() const;
  float * ori_left_knee() const;
  float ori_left_knee(unsigned int index) const;
  void set_ori_left_knee(unsigned int index, const float new_ori_left_knee);
  void set_ori_left_knee(const float * new_ori_left_knee);
  size_t maxlenof_ori_left_knee() const;
  float ori_left_knee_confidence() const;
  void set_ori_left_knee_confidence(const float new_ori_left_knee_confidence);
  size_t maxlenof_ori_left_knee_confidence() const;
  float * ori_left_ankle() const;
  float ori_left_ankle(unsigned int index) const;
  void set_ori_left_ankle(unsigned int index, const float new_ori_left_ankle);
  void set_ori_left_ankle(const float * new_ori_left_ankle);
  size_t maxlenof_ori_left_ankle() const;
  float ori_left_ankle_confidence() const;
  void set_ori_left_ankle_confidence(const float new_ori_left_ankle_confidence);
  size_t maxlenof_ori_left_ankle_confidence() const;
  float * ori_left_foot() const;
  float ori_left_foot(unsigned int index) const;
  void set_ori_left_foot(unsigned int index, const float new_ori_left_foot);
  void set_ori_left_foot(const float * new_ori_left_foot);
  size_t maxlenof_ori_left_foot() const;
  float ori_left_foot_confidence() const;
  void set_ori_left_foot_confidence(const float new_ori_left_foot_confidence);
  size_t maxlenof_ori_left_foot_confidence() const;
  float * ori_right_hip() const;
  float ori_right_hip(unsigned int index) const;
  void set_ori_right_hip(unsigned int index, const float new_ori_right_hip);
  void set_ori_right_hip(const float * new_ori_right_hip);
  size_t maxlenof_ori_right_hip() const;
  float ori_right_hip_confidence() const;
  void set_ori_right_hip_confidence(const float new_ori_right_hip_confidence);
  size_t maxlenof_ori_right_hip_confidence() const;
  float * ori_right_knee() const;
  float ori_right_knee(unsigned int index) const;
  void set_ori_right_knee(unsigned int index, const float new_ori_right_knee);
  void set_ori_right_knee(const float * new_ori_right_knee);
  size_t maxlenof_ori_right_knee() const;
  float ori_right_knee_confidence() const;
  void set_ori_right_knee_confidence(const float new_ori_right_knee_confidence);
  size_t maxlenof_ori_right_knee_confidence() const;
  float * ori_right_ankle() const;
  float ori_right_ankle(unsigned int index) const;
  void set_ori_right_ankle(unsigned int index, const float new_ori_right_ankle);
  void set_ori_right_ankle(const float * new_ori_right_ankle);
  size_t maxlenof_ori_right_ankle() const;
  float ori_right_ankle_confidence() const;
  void set_ori_right_ankle_confidence(const float new_ori_right_ankle_confidence);
  size_t maxlenof_ori_right_ankle_confidence() const;
  float * ori_right_foot() const;
  float ori_right_foot(unsigned int index) const;
  void set_ori_right_foot(unsigned int index, const float new_ori_right_foot);
  void set_ori_right_foot(const float * new_ori_right_foot);
  size_t maxlenof_ori_right_foot() const;
  float ori_right_foot_confidence() const;
  void set_ori_right_foot_confidence(const float new_ori_right_foot_confidence);
  size_t maxlenof_ori_right_foot_confidence() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
