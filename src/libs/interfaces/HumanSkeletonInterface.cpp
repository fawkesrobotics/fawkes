
/***************************************************************************
 *  HumanSkeletonInterface.cpp - Fawkes BlackBoard Interface - HumanSkeletonInterface
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

#include <interfaces/HumanSkeletonInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class HumanSkeletonInterface <interfaces/HumanSkeletonInterface.h>
 * HumanSkeletonInterface Fawkes BlackBoard Interface.
 * 
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
HumanSkeletonInterface::HumanSkeletonInterface() : Interface()
{
  data_size = sizeof(HumanSkeletonInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (HumanSkeletonInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_State[(int)STATE_INVALID] = "STATE_INVALID";
  enum_map_State[(int)STATE_DETECTING_POSE] = "STATE_DETECTING_POSE";
  enum_map_State[(int)STATE_CALIBRATING] = "STATE_CALIBRATING";
  enum_map_State[(int)STATE_TRACKING] = "STATE_TRACKING";
  add_fieldinfo(IFT_ENUM, "state", 1, &data->state, "State", &enum_map_State);
  add_fieldinfo(IFT_UINT32, "user_id", 1, &data->user_id);
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_STRING, "pose", 32, data->pose);
  add_fieldinfo(IFT_FLOAT, "com", 3, &data->com);
  add_fieldinfo(IFT_FLOAT, "pos_head", 3, &data->pos_head);
  add_fieldinfo(IFT_FLOAT, "pos_head_confidence", 1, &data->pos_head_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_neck", 3, &data->pos_neck);
  add_fieldinfo(IFT_FLOAT, "pos_neck_confidence", 1, &data->pos_neck_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_torso", 3, &data->pos_torso);
  add_fieldinfo(IFT_FLOAT, "pos_torso_confidence", 1, &data->pos_torso_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_waist", 3, &data->pos_waist);
  add_fieldinfo(IFT_FLOAT, "pos_waist_confidence", 1, &data->pos_waist_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_collar", 3, &data->pos_left_collar);
  add_fieldinfo(IFT_FLOAT, "pos_left_collar_confidence", 1, &data->pos_left_collar_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_shoulder", 3, &data->pos_left_shoulder);
  add_fieldinfo(IFT_FLOAT, "pos_left_shoulder_confidence", 1, &data->pos_left_shoulder_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_elbow", 3, &data->pos_left_elbow);
  add_fieldinfo(IFT_FLOAT, "pos_left_elbow_confidence", 1, &data->pos_left_elbow_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_wrist", 3, &data->pos_left_wrist);
  add_fieldinfo(IFT_FLOAT, "pos_left_wrist_confidence", 1, &data->pos_left_wrist_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_hand", 3, &data->pos_left_hand);
  add_fieldinfo(IFT_FLOAT, "pos_left_hand_confidence", 1, &data->pos_left_hand_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_fingertip", 3, &data->pos_left_fingertip);
  add_fieldinfo(IFT_FLOAT, "pos_left_fingertip_confidence", 1, &data->pos_left_fingertip_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_collar", 3, &data->pos_right_collar);
  add_fieldinfo(IFT_FLOAT, "pos_right_collar_confidence", 1, &data->pos_right_collar_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_shoulder", 3, &data->pos_right_shoulder);
  add_fieldinfo(IFT_FLOAT, "pos_right_shoulder_confidence", 1, &data->pos_right_shoulder_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_elbow", 3, &data->pos_right_elbow);
  add_fieldinfo(IFT_FLOAT, "pos_right_elbow_confidence", 1, &data->pos_right_elbow_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_wrist", 3, &data->pos_right_wrist);
  add_fieldinfo(IFT_FLOAT, "pos_right_wrist_confidence", 1, &data->pos_right_wrist_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_hand", 3, &data->pos_right_hand);
  add_fieldinfo(IFT_FLOAT, "pos_right_hand_confidence", 1, &data->pos_right_hand_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_fingertip", 3, &data->pos_right_fingertip);
  add_fieldinfo(IFT_FLOAT, "pos_right_fingertip_confidence", 1, &data->pos_right_fingertip_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_hip", 3, &data->pos_left_hip);
  add_fieldinfo(IFT_FLOAT, "pos_left_hip_confidence", 1, &data->pos_left_hip_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_knee", 3, &data->pos_left_knee);
  add_fieldinfo(IFT_FLOAT, "pos_left_knee_confidence", 1, &data->pos_left_knee_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_ankle", 3, &data->pos_left_ankle);
  add_fieldinfo(IFT_FLOAT, "pos_left_ankle_confidence", 1, &data->pos_left_ankle_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_left_foot", 3, &data->pos_left_foot);
  add_fieldinfo(IFT_FLOAT, "pos_left_foot_confidence", 1, &data->pos_left_foot_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_hip", 3, &data->pos_right_hip);
  add_fieldinfo(IFT_FLOAT, "pos_right_hip_confidence", 1, &data->pos_right_hip_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_knee", 3, &data->pos_right_knee);
  add_fieldinfo(IFT_FLOAT, "pos_right_knee_confidence", 1, &data->pos_right_knee_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_ankle", 3, &data->pos_right_ankle);
  add_fieldinfo(IFT_FLOAT, "pos_right_ankle_confidence", 1, &data->pos_right_ankle_confidence);
  add_fieldinfo(IFT_FLOAT, "pos_right_foot", 3, &data->pos_right_foot);
  add_fieldinfo(IFT_FLOAT, "pos_right_foot_confidence", 1, &data->pos_right_foot_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_head", 9, &data->ori_head);
  add_fieldinfo(IFT_FLOAT, "ori_head_confidence", 1, &data->ori_head_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_neck", 9, &data->ori_neck);
  add_fieldinfo(IFT_FLOAT, "ori_neck_confidence", 1, &data->ori_neck_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_torso", 9, &data->ori_torso);
  add_fieldinfo(IFT_FLOAT, "ori_torso_confidence", 1, &data->ori_torso_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_waist", 9, &data->ori_waist);
  add_fieldinfo(IFT_FLOAT, "ori_waist_confidence", 1, &data->ori_waist_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_collar", 9, &data->ori_left_collar);
  add_fieldinfo(IFT_FLOAT, "ori_left_collar_confidence", 1, &data->ori_left_collar_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_shoulder", 9, &data->ori_left_shoulder);
  add_fieldinfo(IFT_FLOAT, "ori_left_shoulder_confidence", 1, &data->ori_left_shoulder_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_elbow", 9, &data->ori_left_elbow);
  add_fieldinfo(IFT_FLOAT, "ori_left_elbow_confidence", 1, &data->ori_left_elbow_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_wrist", 9, &data->ori_left_wrist);
  add_fieldinfo(IFT_FLOAT, "ori_left_wrist_confidence", 1, &data->ori_left_wrist_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_hand", 9, &data->ori_left_hand);
  add_fieldinfo(IFT_FLOAT, "ori_left_hand_confidence", 1, &data->ori_left_hand_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_fingertip", 9, &data->ori_left_fingertip);
  add_fieldinfo(IFT_FLOAT, "ori_left_fingertip_confidence", 1, &data->ori_left_fingertip_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_collar", 9, &data->ori_right_collar);
  add_fieldinfo(IFT_FLOAT, "ori_right_collar_confidence", 1, &data->ori_right_collar_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_shoulder", 9, &data->ori_right_shoulder);
  add_fieldinfo(IFT_FLOAT, "ori_right_shoulder_confidence", 1, &data->ori_right_shoulder_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_elbow", 9, &data->ori_right_elbow);
  add_fieldinfo(IFT_FLOAT, "ori_right_elbow_confidence", 1, &data->ori_right_elbow_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_wrist", 9, &data->ori_right_wrist);
  add_fieldinfo(IFT_FLOAT, "ori_right_wrist_confidence", 1, &data->ori_right_wrist_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_hand", 9, &data->ori_right_hand);
  add_fieldinfo(IFT_FLOAT, "ori_right_hand_confidence", 1, &data->ori_right_hand_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_fingertip", 9, &data->ori_right_fingertip);
  add_fieldinfo(IFT_FLOAT, "ori_right_fingertip_confidence", 1, &data->ori_right_fingertip_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_hip", 9, &data->ori_left_hip);
  add_fieldinfo(IFT_FLOAT, "ori_left_hip_confidence", 1, &data->ori_left_hip_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_knee", 9, &data->ori_left_knee);
  add_fieldinfo(IFT_FLOAT, "ori_left_knee_confidence", 1, &data->ori_left_knee_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_ankle", 9, &data->ori_left_ankle);
  add_fieldinfo(IFT_FLOAT, "ori_left_ankle_confidence", 1, &data->ori_left_ankle_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_left_foot", 9, &data->ori_left_foot);
  add_fieldinfo(IFT_FLOAT, "ori_left_foot_confidence", 1, &data->ori_left_foot_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_hip", 9, &data->ori_right_hip);
  add_fieldinfo(IFT_FLOAT, "ori_right_hip_confidence", 1, &data->ori_right_hip_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_knee", 9, &data->ori_right_knee);
  add_fieldinfo(IFT_FLOAT, "ori_right_knee_confidence", 1, &data->ori_right_knee_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_ankle", 9, &data->ori_right_ankle);
  add_fieldinfo(IFT_FLOAT, "ori_right_ankle_confidence", 1, &data->ori_right_ankle_confidence);
  add_fieldinfo(IFT_FLOAT, "ori_right_foot", 9, &data->ori_right_foot);
  add_fieldinfo(IFT_FLOAT, "ori_right_foot_confidence", 1, &data->ori_right_foot_confidence);
  unsigned char tmp_hash[] = {0x5f, 0x47, 0x2f, 0xb3, 0x8b, 0xf1, 0xe1, 0xa, 0xb9, 0x42, 0x34, 0xea, 0x83, 0x43, 0x94, 0x37};
  set_hash(tmp_hash);
}

/** Destructor */
HumanSkeletonInterface::~HumanSkeletonInterface()
{
  free(data_ptr);
}
/** Convert State constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
HumanSkeletonInterface::tostring_State(State value) const
{
  switch (value) {
  case STATE_INVALID: return "STATE_INVALID";
  case STATE_DETECTING_POSE: return "STATE_DETECTING_POSE";
  case STATE_CALIBRATING: return "STATE_CALIBRATING";
  case STATE_TRACKING: return "STATE_TRACKING";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get state value.
 * Current state.
 * @return state value
 */
HumanSkeletonInterface::State
HumanSkeletonInterface::state() const
{
  return (HumanSkeletonInterface::State)data->state;
}

/** Get maximum length of state value.
 * @return length of state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_state() const
{
  return 1;
}

/** Set state value.
 * Current state.
 * @param new_state new state value
 */
void
HumanSkeletonInterface::set_state(const State new_state)
{
  data->state = new_state;
  data_changed = true;
}

/** Get user_id value.
 * Tracking ID of this user.
 * @return user_id value
 */
uint32_t
HumanSkeletonInterface::user_id() const
{
  return data->user_id;
}

/** Get maximum length of user_id value.
 * @return length of user_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_user_id() const
{
  return 1;
}

/** Set user_id value.
 * Tracking ID of this user.
 * @param new_user_id new user_id value
 */
void
HumanSkeletonInterface::set_user_id(const uint32_t new_user_id)
{
  data->user_id = new_user_id;
  data_changed = true;
}

/** Get visibility_history value.
 * 
      The visibility history indicates the persistence of user sightings.
      A positive value indicates the number of successful consecutive sightings
      of the user (center of mass not equal to zero), the absolute of a negative
      value gives the number of consecutive negative (non-) sightings. The value
      is zero only if uninitialized.
    
 * @return visibility_history value
 */
int32_t
HumanSkeletonInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * 
      The visibility history indicates the persistence of user sightings.
      A positive value indicates the number of successful consecutive sightings
      of the user (center of mass not equal to zero), the absolute of a negative
      value gives the number of consecutive negative (non-) sightings. The value
      is zero only if uninitialized.
    
 * @param new_visibility_history new visibility_history value
 */
void
HumanSkeletonInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get pose value.
 * Detected user pose.
 * @return pose value
 */
char *
HumanSkeletonInterface::pose() const
{
  return data->pose;
}

/** Get maximum length of pose value.
 * @return length of pose value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pose() const
{
  return 32;
}

/** Set pose value.
 * Detected user pose.
 * @param new_pose new pose value
 */
void
HumanSkeletonInterface::set_pose(const char * new_pose)
{
  strncpy(data->pose, new_pose, sizeof(data->pose)-1);
  data->pose[sizeof(data->pose)-1] = 0;
  data_changed = true;
}

/** Get com value.
 * Center of mass.
 * @return com value
 */
float *
HumanSkeletonInterface::com() const
{
  return data->com;
}

/** Get com value at given index.
 * Center of mass.
 * @param index index of value
 * @return com value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::com(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->com[index];
}

/** Get maximum length of com value.
 * @return length of com value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_com() const
{
  return 3;
}

/** Set com value.
 * Center of mass.
 * @param new_com new com value
 */
void
HumanSkeletonInterface::set_com(const float * new_com)
{
  memcpy(data->com, new_com, sizeof(float) * 3);
  data_changed = true;
}

/** Set com value at given index.
 * Center of mass.
 * @param new_com new com value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_com(unsigned int index, const float new_com)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->com[index] = new_com;
  data_changed = true;
}
/** Get pos_head value.
 * Head position vector.
 * @return pos_head value
 */
float *
HumanSkeletonInterface::pos_head() const
{
  return data->pos_head;
}

/** Get pos_head value at given index.
 * Head position vector.
 * @param index index of value
 * @return pos_head value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_head(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_head[index];
}

/** Get maximum length of pos_head value.
 * @return length of pos_head value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_head() const
{
  return 3;
}

/** Set pos_head value.
 * Head position vector.
 * @param new_pos_head new pos_head value
 */
void
HumanSkeletonInterface::set_pos_head(const float * new_pos_head)
{
  memcpy(data->pos_head, new_pos_head, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_head value at given index.
 * Head position vector.
 * @param new_pos_head new pos_head value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_head(unsigned int index, const float new_pos_head)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_head[index] = new_pos_head;
  data_changed = true;
}
/** Get pos_head_confidence value.
 * 
      Head position confidence.
 * @return pos_head_confidence value
 */
float
HumanSkeletonInterface::pos_head_confidence() const
{
  return data->pos_head_confidence;
}

/** Get maximum length of pos_head_confidence value.
 * @return length of pos_head_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_head_confidence() const
{
  return 1;
}

/** Set pos_head_confidence value.
 * 
      Head position confidence.
 * @param new_pos_head_confidence new pos_head_confidence value
 */
void
HumanSkeletonInterface::set_pos_head_confidence(const float new_pos_head_confidence)
{
  data->pos_head_confidence = new_pos_head_confidence;
  data_changed = true;
}

/** Get pos_neck value.
 * Neck position vector.
 * @return pos_neck value
 */
float *
HumanSkeletonInterface::pos_neck() const
{
  return data->pos_neck;
}

/** Get pos_neck value at given index.
 * Neck position vector.
 * @param index index of value
 * @return pos_neck value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_neck(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_neck[index];
}

/** Get maximum length of pos_neck value.
 * @return length of pos_neck value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_neck() const
{
  return 3;
}

/** Set pos_neck value.
 * Neck position vector.
 * @param new_pos_neck new pos_neck value
 */
void
HumanSkeletonInterface::set_pos_neck(const float * new_pos_neck)
{
  memcpy(data->pos_neck, new_pos_neck, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_neck value at given index.
 * Neck position vector.
 * @param new_pos_neck new pos_neck value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_neck(unsigned int index, const float new_pos_neck)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_neck[index] = new_pos_neck;
  data_changed = true;
}
/** Get pos_neck_confidence value.
 * 
      Neck position confidence.
 * @return pos_neck_confidence value
 */
float
HumanSkeletonInterface::pos_neck_confidence() const
{
  return data->pos_neck_confidence;
}

/** Get maximum length of pos_neck_confidence value.
 * @return length of pos_neck_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_neck_confidence() const
{
  return 1;
}

/** Set pos_neck_confidence value.
 * 
      Neck position confidence.
 * @param new_pos_neck_confidence new pos_neck_confidence value
 */
void
HumanSkeletonInterface::set_pos_neck_confidence(const float new_pos_neck_confidence)
{
  data->pos_neck_confidence = new_pos_neck_confidence;
  data_changed = true;
}

/** Get pos_torso value.
 * Torso position vector.
 * @return pos_torso value
 */
float *
HumanSkeletonInterface::pos_torso() const
{
  return data->pos_torso;
}

/** Get pos_torso value at given index.
 * Torso position vector.
 * @param index index of value
 * @return pos_torso value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_torso(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_torso[index];
}

/** Get maximum length of pos_torso value.
 * @return length of pos_torso value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_torso() const
{
  return 3;
}

/** Set pos_torso value.
 * Torso position vector.
 * @param new_pos_torso new pos_torso value
 */
void
HumanSkeletonInterface::set_pos_torso(const float * new_pos_torso)
{
  memcpy(data->pos_torso, new_pos_torso, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_torso value at given index.
 * Torso position vector.
 * @param new_pos_torso new pos_torso value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_torso(unsigned int index, const float new_pos_torso)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_torso[index] = new_pos_torso;
  data_changed = true;
}
/** Get pos_torso_confidence value.
 * 
      Torso position confidence.
 * @return pos_torso_confidence value
 */
float
HumanSkeletonInterface::pos_torso_confidence() const
{
  return data->pos_torso_confidence;
}

/** Get maximum length of pos_torso_confidence value.
 * @return length of pos_torso_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_torso_confidence() const
{
  return 1;
}

/** Set pos_torso_confidence value.
 * 
      Torso position confidence.
 * @param new_pos_torso_confidence new pos_torso_confidence value
 */
void
HumanSkeletonInterface::set_pos_torso_confidence(const float new_pos_torso_confidence)
{
  data->pos_torso_confidence = new_pos_torso_confidence;
  data_changed = true;
}

/** Get pos_waist value.
 * Waist position vector.
 * @return pos_waist value
 */
float *
HumanSkeletonInterface::pos_waist() const
{
  return data->pos_waist;
}

/** Get pos_waist value at given index.
 * Waist position vector.
 * @param index index of value
 * @return pos_waist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_waist(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_waist[index];
}

/** Get maximum length of pos_waist value.
 * @return length of pos_waist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_waist() const
{
  return 3;
}

/** Set pos_waist value.
 * Waist position vector.
 * @param new_pos_waist new pos_waist value
 */
void
HumanSkeletonInterface::set_pos_waist(const float * new_pos_waist)
{
  memcpy(data->pos_waist, new_pos_waist, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_waist value at given index.
 * Waist position vector.
 * @param new_pos_waist new pos_waist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_waist(unsigned int index, const float new_pos_waist)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_waist[index] = new_pos_waist;
  data_changed = true;
}
/** Get pos_waist_confidence value.
 * 
      Waist position confidence.
 * @return pos_waist_confidence value
 */
float
HumanSkeletonInterface::pos_waist_confidence() const
{
  return data->pos_waist_confidence;
}

/** Get maximum length of pos_waist_confidence value.
 * @return length of pos_waist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_waist_confidence() const
{
  return 1;
}

/** Set pos_waist_confidence value.
 * 
      Waist position confidence.
 * @param new_pos_waist_confidence new pos_waist_confidence value
 */
void
HumanSkeletonInterface::set_pos_waist_confidence(const float new_pos_waist_confidence)
{
  data->pos_waist_confidence = new_pos_waist_confidence;
  data_changed = true;
}

/** Get pos_left_collar value.
 * 
      Left position vector.
 * @return pos_left_collar value
 */
float *
HumanSkeletonInterface::pos_left_collar() const
{
  return data->pos_left_collar;
}

/** Get pos_left_collar value at given index.
 * 
      Left position vector.
 * @param index index of value
 * @return pos_left_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_collar(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_collar[index];
}

/** Get maximum length of pos_left_collar value.
 * @return length of pos_left_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_collar() const
{
  return 3;
}

/** Set pos_left_collar value.
 * 
      Left position vector.
 * @param new_pos_left_collar new pos_left_collar value
 */
void
HumanSkeletonInterface::set_pos_left_collar(const float * new_pos_left_collar)
{
  memcpy(data->pos_left_collar, new_pos_left_collar, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_collar value at given index.
 * 
      Left position vector.
 * @param new_pos_left_collar new pos_left_collar value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_collar(unsigned int index, const float new_pos_left_collar)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_collar[index] = new_pos_left_collar;
  data_changed = true;
}
/** Get pos_left_collar_confidence value.
 * 
      Left position confidence.
 * @return pos_left_collar_confidence value
 */
float
HumanSkeletonInterface::pos_left_collar_confidence() const
{
  return data->pos_left_collar_confidence;
}

/** Get maximum length of pos_left_collar_confidence value.
 * @return length of pos_left_collar_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_collar_confidence() const
{
  return 1;
}

/** Set pos_left_collar_confidence value.
 * 
      Left position confidence.
 * @param new_pos_left_collar_confidence new pos_left_collar_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_collar_confidence(const float new_pos_left_collar_confidence)
{
  data->pos_left_collar_confidence = new_pos_left_collar_confidence;
  data_changed = true;
}

/** Get pos_left_shoulder value.
 * 
      Left shoulder position vector.
 * @return pos_left_shoulder value
 */
float *
HumanSkeletonInterface::pos_left_shoulder() const
{
  return data->pos_left_shoulder;
}

/** Get pos_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param index index of value
 * @return pos_left_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_shoulder(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_shoulder[index];
}

/** Get maximum length of pos_left_shoulder value.
 * @return length of pos_left_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_shoulder() const
{
  return 3;
}

/** Set pos_left_shoulder value.
 * 
      Left shoulder position vector.
 * @param new_pos_left_shoulder new pos_left_shoulder value
 */
void
HumanSkeletonInterface::set_pos_left_shoulder(const float * new_pos_left_shoulder)
{
  memcpy(data->pos_left_shoulder, new_pos_left_shoulder, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param new_pos_left_shoulder new pos_left_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_shoulder(unsigned int index, const float new_pos_left_shoulder)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_shoulder[index] = new_pos_left_shoulder;
  data_changed = true;
}
/** Get pos_left_shoulder_confidence value.
 * 
      Left shoulder position confidence.
 * @return pos_left_shoulder_confidence value
 */
float
HumanSkeletonInterface::pos_left_shoulder_confidence() const
{
  return data->pos_left_shoulder_confidence;
}

/** Get maximum length of pos_left_shoulder_confidence value.
 * @return length of pos_left_shoulder_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_shoulder_confidence() const
{
  return 1;
}

/** Set pos_left_shoulder_confidence value.
 * 
      Left shoulder position confidence.
 * @param new_pos_left_shoulder_confidence new pos_left_shoulder_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_shoulder_confidence(const float new_pos_left_shoulder_confidence)
{
  data->pos_left_shoulder_confidence = new_pos_left_shoulder_confidence;
  data_changed = true;
}

/** Get pos_left_elbow value.
 * 
      Left elbow position vector.
 * @return pos_left_elbow value
 */
float *
HumanSkeletonInterface::pos_left_elbow() const
{
  return data->pos_left_elbow;
}

/** Get pos_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param index index of value
 * @return pos_left_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_elbow(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_elbow[index];
}

/** Get maximum length of pos_left_elbow value.
 * @return length of pos_left_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_elbow() const
{
  return 3;
}

/** Set pos_left_elbow value.
 * 
      Left elbow position vector.
 * @param new_pos_left_elbow new pos_left_elbow value
 */
void
HumanSkeletonInterface::set_pos_left_elbow(const float * new_pos_left_elbow)
{
  memcpy(data->pos_left_elbow, new_pos_left_elbow, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param new_pos_left_elbow new pos_left_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_elbow(unsigned int index, const float new_pos_left_elbow)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_elbow[index] = new_pos_left_elbow;
  data_changed = true;
}
/** Get pos_left_elbow_confidence value.
 * 
      Left elbow position confidence.
 * @return pos_left_elbow_confidence value
 */
float
HumanSkeletonInterface::pos_left_elbow_confidence() const
{
  return data->pos_left_elbow_confidence;
}

/** Get maximum length of pos_left_elbow_confidence value.
 * @return length of pos_left_elbow_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_elbow_confidence() const
{
  return 1;
}

/** Set pos_left_elbow_confidence value.
 * 
      Left elbow position confidence.
 * @param new_pos_left_elbow_confidence new pos_left_elbow_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_elbow_confidence(const float new_pos_left_elbow_confidence)
{
  data->pos_left_elbow_confidence = new_pos_left_elbow_confidence;
  data_changed = true;
}

/** Get pos_left_wrist value.
 * 
      Left wrist position vector.
 * @return pos_left_wrist value
 */
float *
HumanSkeletonInterface::pos_left_wrist() const
{
  return data->pos_left_wrist;
}

/** Get pos_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param index index of value
 * @return pos_left_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_wrist(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_wrist[index];
}

/** Get maximum length of pos_left_wrist value.
 * @return length of pos_left_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_wrist() const
{
  return 3;
}

/** Set pos_left_wrist value.
 * 
      Left wrist position vector.
 * @param new_pos_left_wrist new pos_left_wrist value
 */
void
HumanSkeletonInterface::set_pos_left_wrist(const float * new_pos_left_wrist)
{
  memcpy(data->pos_left_wrist, new_pos_left_wrist, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param new_pos_left_wrist new pos_left_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_wrist(unsigned int index, const float new_pos_left_wrist)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_wrist[index] = new_pos_left_wrist;
  data_changed = true;
}
/** Get pos_left_wrist_confidence value.
 * 
      Left wrist position confidence.
 * @return pos_left_wrist_confidence value
 */
float
HumanSkeletonInterface::pos_left_wrist_confidence() const
{
  return data->pos_left_wrist_confidence;
}

/** Get maximum length of pos_left_wrist_confidence value.
 * @return length of pos_left_wrist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_wrist_confidence() const
{
  return 1;
}

/** Set pos_left_wrist_confidence value.
 * 
      Left wrist position confidence.
 * @param new_pos_left_wrist_confidence new pos_left_wrist_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_wrist_confidence(const float new_pos_left_wrist_confidence)
{
  data->pos_left_wrist_confidence = new_pos_left_wrist_confidence;
  data_changed = true;
}

/** Get pos_left_hand value.
 * 
      Left hand position vector.
 * @return pos_left_hand value
 */
float *
HumanSkeletonInterface::pos_left_hand() const
{
  return data->pos_left_hand;
}

/** Get pos_left_hand value at given index.
 * 
      Left hand position vector.
 * @param index index of value
 * @return pos_left_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_hand(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_hand[index];
}

/** Get maximum length of pos_left_hand value.
 * @return length of pos_left_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_hand() const
{
  return 3;
}

/** Set pos_left_hand value.
 * 
      Left hand position vector.
 * @param new_pos_left_hand new pos_left_hand value
 */
void
HumanSkeletonInterface::set_pos_left_hand(const float * new_pos_left_hand)
{
  memcpy(data->pos_left_hand, new_pos_left_hand, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_hand value at given index.
 * 
      Left hand position vector.
 * @param new_pos_left_hand new pos_left_hand value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_hand(unsigned int index, const float new_pos_left_hand)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_hand[index] = new_pos_left_hand;
  data_changed = true;
}
/** Get pos_left_hand_confidence value.
 * 
      Left hand position confidence.
 * @return pos_left_hand_confidence value
 */
float
HumanSkeletonInterface::pos_left_hand_confidence() const
{
  return data->pos_left_hand_confidence;
}

/** Get maximum length of pos_left_hand_confidence value.
 * @return length of pos_left_hand_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_hand_confidence() const
{
  return 1;
}

/** Set pos_left_hand_confidence value.
 * 
      Left hand position confidence.
 * @param new_pos_left_hand_confidence new pos_left_hand_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_hand_confidence(const float new_pos_left_hand_confidence)
{
  data->pos_left_hand_confidence = new_pos_left_hand_confidence;
  data_changed = true;
}

/** Get pos_left_fingertip value.
 * 
      Left fingertip position vector.
 * @return pos_left_fingertip value
 */
float *
HumanSkeletonInterface::pos_left_fingertip() const
{
  return data->pos_left_fingertip;
}

/** Get pos_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param index index of value
 * @return pos_left_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_fingertip(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_fingertip[index];
}

/** Get maximum length of pos_left_fingertip value.
 * @return length of pos_left_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_fingertip() const
{
  return 3;
}

/** Set pos_left_fingertip value.
 * 
      Left fingertip position vector.
 * @param new_pos_left_fingertip new pos_left_fingertip value
 */
void
HumanSkeletonInterface::set_pos_left_fingertip(const float * new_pos_left_fingertip)
{
  memcpy(data->pos_left_fingertip, new_pos_left_fingertip, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param new_pos_left_fingertip new pos_left_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_fingertip(unsigned int index, const float new_pos_left_fingertip)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_fingertip[index] = new_pos_left_fingertip;
  data_changed = true;
}
/** Get pos_left_fingertip_confidence value.
 * 
      Left fingertip position confidence.
 * @return pos_left_fingertip_confidence value
 */
float
HumanSkeletonInterface::pos_left_fingertip_confidence() const
{
  return data->pos_left_fingertip_confidence;
}

/** Get maximum length of pos_left_fingertip_confidence value.
 * @return length of pos_left_fingertip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_fingertip_confidence() const
{
  return 1;
}

/** Set pos_left_fingertip_confidence value.
 * 
      Left fingertip position confidence.
 * @param new_pos_left_fingertip_confidence new pos_left_fingertip_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_fingertip_confidence(const float new_pos_left_fingertip_confidence)
{
  data->pos_left_fingertip_confidence = new_pos_left_fingertip_confidence;
  data_changed = true;
}

/** Get pos_right_collar value.
 * 
      Right collar position vector.
 * @return pos_right_collar value
 */
float *
HumanSkeletonInterface::pos_right_collar() const
{
  return data->pos_right_collar;
}

/** Get pos_right_collar value at given index.
 * 
      Right collar position vector.
 * @param index index of value
 * @return pos_right_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_collar(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_collar[index];
}

/** Get maximum length of pos_right_collar value.
 * @return length of pos_right_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_collar() const
{
  return 3;
}

/** Set pos_right_collar value.
 * 
      Right collar position vector.
 * @param new_pos_right_collar new pos_right_collar value
 */
void
HumanSkeletonInterface::set_pos_right_collar(const float * new_pos_right_collar)
{
  memcpy(data->pos_right_collar, new_pos_right_collar, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_collar value at given index.
 * 
      Right collar position vector.
 * @param new_pos_right_collar new pos_right_collar value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_collar(unsigned int index, const float new_pos_right_collar)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_collar[index] = new_pos_right_collar;
  data_changed = true;
}
/** Get pos_right_collar_confidence value.
 * 
      Right collar position confidence.
 * @return pos_right_collar_confidence value
 */
float
HumanSkeletonInterface::pos_right_collar_confidence() const
{
  return data->pos_right_collar_confidence;
}

/** Get maximum length of pos_right_collar_confidence value.
 * @return length of pos_right_collar_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_collar_confidence() const
{
  return 1;
}

/** Set pos_right_collar_confidence value.
 * 
      Right collar position confidence.
 * @param new_pos_right_collar_confidence new pos_right_collar_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_collar_confidence(const float new_pos_right_collar_confidence)
{
  data->pos_right_collar_confidence = new_pos_right_collar_confidence;
  data_changed = true;
}

/** Get pos_right_shoulder value.
 * 
      Right shoulder position vector.
 * @return pos_right_shoulder value
 */
float *
HumanSkeletonInterface::pos_right_shoulder() const
{
  return data->pos_right_shoulder;
}

/** Get pos_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param index index of value
 * @return pos_right_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_shoulder(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_shoulder[index];
}

/** Get maximum length of pos_right_shoulder value.
 * @return length of pos_right_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_shoulder() const
{
  return 3;
}

/** Set pos_right_shoulder value.
 * 
      Right shoulder position vector.
 * @param new_pos_right_shoulder new pos_right_shoulder value
 */
void
HumanSkeletonInterface::set_pos_right_shoulder(const float * new_pos_right_shoulder)
{
  memcpy(data->pos_right_shoulder, new_pos_right_shoulder, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param new_pos_right_shoulder new pos_right_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_shoulder(unsigned int index, const float new_pos_right_shoulder)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_shoulder[index] = new_pos_right_shoulder;
  data_changed = true;
}
/** Get pos_right_shoulder_confidence value.
 * 
      Right shoulder position confidence.
 * @return pos_right_shoulder_confidence value
 */
float
HumanSkeletonInterface::pos_right_shoulder_confidence() const
{
  return data->pos_right_shoulder_confidence;
}

/** Get maximum length of pos_right_shoulder_confidence value.
 * @return length of pos_right_shoulder_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_shoulder_confidence() const
{
  return 1;
}

/** Set pos_right_shoulder_confidence value.
 * 
      Right shoulder position confidence.
 * @param new_pos_right_shoulder_confidence new pos_right_shoulder_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_shoulder_confidence(const float new_pos_right_shoulder_confidence)
{
  data->pos_right_shoulder_confidence = new_pos_right_shoulder_confidence;
  data_changed = true;
}

/** Get pos_right_elbow value.
 * 
      Right elbow position vector.
 * @return pos_right_elbow value
 */
float *
HumanSkeletonInterface::pos_right_elbow() const
{
  return data->pos_right_elbow;
}

/** Get pos_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param index index of value
 * @return pos_right_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_elbow(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_elbow[index];
}

/** Get maximum length of pos_right_elbow value.
 * @return length of pos_right_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_elbow() const
{
  return 3;
}

/** Set pos_right_elbow value.
 * 
      Right elbow position vector.
 * @param new_pos_right_elbow new pos_right_elbow value
 */
void
HumanSkeletonInterface::set_pos_right_elbow(const float * new_pos_right_elbow)
{
  memcpy(data->pos_right_elbow, new_pos_right_elbow, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param new_pos_right_elbow new pos_right_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_elbow(unsigned int index, const float new_pos_right_elbow)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_elbow[index] = new_pos_right_elbow;
  data_changed = true;
}
/** Get pos_right_elbow_confidence value.
 * 
      Right elbow position confidence.
 * @return pos_right_elbow_confidence value
 */
float
HumanSkeletonInterface::pos_right_elbow_confidence() const
{
  return data->pos_right_elbow_confidence;
}

/** Get maximum length of pos_right_elbow_confidence value.
 * @return length of pos_right_elbow_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_elbow_confidence() const
{
  return 1;
}

/** Set pos_right_elbow_confidence value.
 * 
      Right elbow position confidence.
 * @param new_pos_right_elbow_confidence new pos_right_elbow_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_elbow_confidence(const float new_pos_right_elbow_confidence)
{
  data->pos_right_elbow_confidence = new_pos_right_elbow_confidence;
  data_changed = true;
}

/** Get pos_right_wrist value.
 * 
      Right wrist position vector.
 * @return pos_right_wrist value
 */
float *
HumanSkeletonInterface::pos_right_wrist() const
{
  return data->pos_right_wrist;
}

/** Get pos_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param index index of value
 * @return pos_right_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_wrist(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_wrist[index];
}

/** Get maximum length of pos_right_wrist value.
 * @return length of pos_right_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_wrist() const
{
  return 3;
}

/** Set pos_right_wrist value.
 * 
      Right wrist position vector.
 * @param new_pos_right_wrist new pos_right_wrist value
 */
void
HumanSkeletonInterface::set_pos_right_wrist(const float * new_pos_right_wrist)
{
  memcpy(data->pos_right_wrist, new_pos_right_wrist, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param new_pos_right_wrist new pos_right_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_wrist(unsigned int index, const float new_pos_right_wrist)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_wrist[index] = new_pos_right_wrist;
  data_changed = true;
}
/** Get pos_right_wrist_confidence value.
 * 
      Right wrist position confidence.
 * @return pos_right_wrist_confidence value
 */
float
HumanSkeletonInterface::pos_right_wrist_confidence() const
{
  return data->pos_right_wrist_confidence;
}

/** Get maximum length of pos_right_wrist_confidence value.
 * @return length of pos_right_wrist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_wrist_confidence() const
{
  return 1;
}

/** Set pos_right_wrist_confidence value.
 * 
      Right wrist position confidence.
 * @param new_pos_right_wrist_confidence new pos_right_wrist_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_wrist_confidence(const float new_pos_right_wrist_confidence)
{
  data->pos_right_wrist_confidence = new_pos_right_wrist_confidence;
  data_changed = true;
}

/** Get pos_right_hand value.
 * 
      Right hand position vector.
 * @return pos_right_hand value
 */
float *
HumanSkeletonInterface::pos_right_hand() const
{
  return data->pos_right_hand;
}

/** Get pos_right_hand value at given index.
 * 
      Right hand position vector.
 * @param index index of value
 * @return pos_right_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_hand(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_hand[index];
}

/** Get maximum length of pos_right_hand value.
 * @return length of pos_right_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_hand() const
{
  return 3;
}

/** Set pos_right_hand value.
 * 
      Right hand position vector.
 * @param new_pos_right_hand new pos_right_hand value
 */
void
HumanSkeletonInterface::set_pos_right_hand(const float * new_pos_right_hand)
{
  memcpy(data->pos_right_hand, new_pos_right_hand, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_hand value at given index.
 * 
      Right hand position vector.
 * @param new_pos_right_hand new pos_right_hand value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_hand(unsigned int index, const float new_pos_right_hand)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_hand[index] = new_pos_right_hand;
  data_changed = true;
}
/** Get pos_right_hand_confidence value.
 * 
      Right hand position confidence.
 * @return pos_right_hand_confidence value
 */
float
HumanSkeletonInterface::pos_right_hand_confidence() const
{
  return data->pos_right_hand_confidence;
}

/** Get maximum length of pos_right_hand_confidence value.
 * @return length of pos_right_hand_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_hand_confidence() const
{
  return 1;
}

/** Set pos_right_hand_confidence value.
 * 
      Right hand position confidence.
 * @param new_pos_right_hand_confidence new pos_right_hand_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_hand_confidence(const float new_pos_right_hand_confidence)
{
  data->pos_right_hand_confidence = new_pos_right_hand_confidence;
  data_changed = true;
}

/** Get pos_right_fingertip value.
 * 
      Right fingertip position vector.
 * @return pos_right_fingertip value
 */
float *
HumanSkeletonInterface::pos_right_fingertip() const
{
  return data->pos_right_fingertip;
}

/** Get pos_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param index index of value
 * @return pos_right_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_fingertip(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_fingertip[index];
}

/** Get maximum length of pos_right_fingertip value.
 * @return length of pos_right_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_fingertip() const
{
  return 3;
}

/** Set pos_right_fingertip value.
 * 
      Right fingertip position vector.
 * @param new_pos_right_fingertip new pos_right_fingertip value
 */
void
HumanSkeletonInterface::set_pos_right_fingertip(const float * new_pos_right_fingertip)
{
  memcpy(data->pos_right_fingertip, new_pos_right_fingertip, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param new_pos_right_fingertip new pos_right_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_fingertip(unsigned int index, const float new_pos_right_fingertip)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_fingertip[index] = new_pos_right_fingertip;
  data_changed = true;
}
/** Get pos_right_fingertip_confidence value.
 * 
      Right fingertip position confidence.
 * @return pos_right_fingertip_confidence value
 */
float
HumanSkeletonInterface::pos_right_fingertip_confidence() const
{
  return data->pos_right_fingertip_confidence;
}

/** Get maximum length of pos_right_fingertip_confidence value.
 * @return length of pos_right_fingertip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_fingertip_confidence() const
{
  return 1;
}

/** Set pos_right_fingertip_confidence value.
 * 
      Right fingertip position confidence.
 * @param new_pos_right_fingertip_confidence new pos_right_fingertip_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_fingertip_confidence(const float new_pos_right_fingertip_confidence)
{
  data->pos_right_fingertip_confidence = new_pos_right_fingertip_confidence;
  data_changed = true;
}

/** Get pos_left_hip value.
 * 
      Left hip position vector.
 * @return pos_left_hip value
 */
float *
HumanSkeletonInterface::pos_left_hip() const
{
  return data->pos_left_hip;
}

/** Get pos_left_hip value at given index.
 * 
      Left hip position vector.
 * @param index index of value
 * @return pos_left_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_hip(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_hip[index];
}

/** Get maximum length of pos_left_hip value.
 * @return length of pos_left_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_hip() const
{
  return 3;
}

/** Set pos_left_hip value.
 * 
      Left hip position vector.
 * @param new_pos_left_hip new pos_left_hip value
 */
void
HumanSkeletonInterface::set_pos_left_hip(const float * new_pos_left_hip)
{
  memcpy(data->pos_left_hip, new_pos_left_hip, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_hip value at given index.
 * 
      Left hip position vector.
 * @param new_pos_left_hip new pos_left_hip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_hip(unsigned int index, const float new_pos_left_hip)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_hip[index] = new_pos_left_hip;
  data_changed = true;
}
/** Get pos_left_hip_confidence value.
 * 
      Left hip position confidence.
 * @return pos_left_hip_confidence value
 */
float
HumanSkeletonInterface::pos_left_hip_confidence() const
{
  return data->pos_left_hip_confidence;
}

/** Get maximum length of pos_left_hip_confidence value.
 * @return length of pos_left_hip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_hip_confidence() const
{
  return 1;
}

/** Set pos_left_hip_confidence value.
 * 
      Left hip position confidence.
 * @param new_pos_left_hip_confidence new pos_left_hip_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_hip_confidence(const float new_pos_left_hip_confidence)
{
  data->pos_left_hip_confidence = new_pos_left_hip_confidence;
  data_changed = true;
}

/** Get pos_left_knee value.
 * 
      Left knee position vector.
 * @return pos_left_knee value
 */
float *
HumanSkeletonInterface::pos_left_knee() const
{
  return data->pos_left_knee;
}

/** Get pos_left_knee value at given index.
 * 
      Left knee position vector.
 * @param index index of value
 * @return pos_left_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_knee(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_knee[index];
}

/** Get maximum length of pos_left_knee value.
 * @return length of pos_left_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_knee() const
{
  return 3;
}

/** Set pos_left_knee value.
 * 
      Left knee position vector.
 * @param new_pos_left_knee new pos_left_knee value
 */
void
HumanSkeletonInterface::set_pos_left_knee(const float * new_pos_left_knee)
{
  memcpy(data->pos_left_knee, new_pos_left_knee, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_knee value at given index.
 * 
      Left knee position vector.
 * @param new_pos_left_knee new pos_left_knee value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_knee(unsigned int index, const float new_pos_left_knee)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_knee[index] = new_pos_left_knee;
  data_changed = true;
}
/** Get pos_left_knee_confidence value.
 * 
      Left knee position confidence.
 * @return pos_left_knee_confidence value
 */
float
HumanSkeletonInterface::pos_left_knee_confidence() const
{
  return data->pos_left_knee_confidence;
}

/** Get maximum length of pos_left_knee_confidence value.
 * @return length of pos_left_knee_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_knee_confidence() const
{
  return 1;
}

/** Set pos_left_knee_confidence value.
 * 
      Left knee position confidence.
 * @param new_pos_left_knee_confidence new pos_left_knee_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_knee_confidence(const float new_pos_left_knee_confidence)
{
  data->pos_left_knee_confidence = new_pos_left_knee_confidence;
  data_changed = true;
}

/** Get pos_left_ankle value.
 * 
      Left ankle position vector.
 * @return pos_left_ankle value
 */
float *
HumanSkeletonInterface::pos_left_ankle() const
{
  return data->pos_left_ankle;
}

/** Get pos_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param index index of value
 * @return pos_left_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_ankle(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_ankle[index];
}

/** Get maximum length of pos_left_ankle value.
 * @return length of pos_left_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_ankle() const
{
  return 3;
}

/** Set pos_left_ankle value.
 * 
      Left ankle position vector.
 * @param new_pos_left_ankle new pos_left_ankle value
 */
void
HumanSkeletonInterface::set_pos_left_ankle(const float * new_pos_left_ankle)
{
  memcpy(data->pos_left_ankle, new_pos_left_ankle, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param new_pos_left_ankle new pos_left_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_ankle(unsigned int index, const float new_pos_left_ankle)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_ankle[index] = new_pos_left_ankle;
  data_changed = true;
}
/** Get pos_left_ankle_confidence value.
 * 
      Left ankle position confidence.
 * @return pos_left_ankle_confidence value
 */
float
HumanSkeletonInterface::pos_left_ankle_confidence() const
{
  return data->pos_left_ankle_confidence;
}

/** Get maximum length of pos_left_ankle_confidence value.
 * @return length of pos_left_ankle_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_ankle_confidence() const
{
  return 1;
}

/** Set pos_left_ankle_confidence value.
 * 
      Left ankle position confidence.
 * @param new_pos_left_ankle_confidence new pos_left_ankle_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_ankle_confidence(const float new_pos_left_ankle_confidence)
{
  data->pos_left_ankle_confidence = new_pos_left_ankle_confidence;
  data_changed = true;
}

/** Get pos_left_foot value.
 * 
      Left foot position vector.
 * @return pos_left_foot value
 */
float *
HumanSkeletonInterface::pos_left_foot() const
{
  return data->pos_left_foot;
}

/** Get pos_left_foot value at given index.
 * 
      Left foot position vector.
 * @param index index of value
 * @return pos_left_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_left_foot(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_left_foot[index];
}

/** Get maximum length of pos_left_foot value.
 * @return length of pos_left_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_foot() const
{
  return 3;
}

/** Set pos_left_foot value.
 * 
      Left foot position vector.
 * @param new_pos_left_foot new pos_left_foot value
 */
void
HumanSkeletonInterface::set_pos_left_foot(const float * new_pos_left_foot)
{
  memcpy(data->pos_left_foot, new_pos_left_foot, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_left_foot value at given index.
 * 
      Left foot position vector.
 * @param new_pos_left_foot new pos_left_foot value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_left_foot(unsigned int index, const float new_pos_left_foot)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_left_foot[index] = new_pos_left_foot;
  data_changed = true;
}
/** Get pos_left_foot_confidence value.
 * 
      Left foot position confidence.
 * @return pos_left_foot_confidence value
 */
float
HumanSkeletonInterface::pos_left_foot_confidence() const
{
  return data->pos_left_foot_confidence;
}

/** Get maximum length of pos_left_foot_confidence value.
 * @return length of pos_left_foot_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_left_foot_confidence() const
{
  return 1;
}

/** Set pos_left_foot_confidence value.
 * 
      Left foot position confidence.
 * @param new_pos_left_foot_confidence new pos_left_foot_confidence value
 */
void
HumanSkeletonInterface::set_pos_left_foot_confidence(const float new_pos_left_foot_confidence)
{
  data->pos_left_foot_confidence = new_pos_left_foot_confidence;
  data_changed = true;
}

/** Get pos_right_hip value.
 * 
      Right hip position vector.
 * @return pos_right_hip value
 */
float *
HumanSkeletonInterface::pos_right_hip() const
{
  return data->pos_right_hip;
}

/** Get pos_right_hip value at given index.
 * 
      Right hip position vector.
 * @param index index of value
 * @return pos_right_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_hip(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_hip[index];
}

/** Get maximum length of pos_right_hip value.
 * @return length of pos_right_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_hip() const
{
  return 3;
}

/** Set pos_right_hip value.
 * 
      Right hip position vector.
 * @param new_pos_right_hip new pos_right_hip value
 */
void
HumanSkeletonInterface::set_pos_right_hip(const float * new_pos_right_hip)
{
  memcpy(data->pos_right_hip, new_pos_right_hip, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_hip value at given index.
 * 
      Right hip position vector.
 * @param new_pos_right_hip new pos_right_hip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_hip(unsigned int index, const float new_pos_right_hip)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_hip[index] = new_pos_right_hip;
  data_changed = true;
}
/** Get pos_right_hip_confidence value.
 * 
      Right hip position confidence.
 * @return pos_right_hip_confidence value
 */
float
HumanSkeletonInterface::pos_right_hip_confidence() const
{
  return data->pos_right_hip_confidence;
}

/** Get maximum length of pos_right_hip_confidence value.
 * @return length of pos_right_hip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_hip_confidence() const
{
  return 1;
}

/** Set pos_right_hip_confidence value.
 * 
      Right hip position confidence.
 * @param new_pos_right_hip_confidence new pos_right_hip_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_hip_confidence(const float new_pos_right_hip_confidence)
{
  data->pos_right_hip_confidence = new_pos_right_hip_confidence;
  data_changed = true;
}

/** Get pos_right_knee value.
 * 
      Right knee position vector.
 * @return pos_right_knee value
 */
float *
HumanSkeletonInterface::pos_right_knee() const
{
  return data->pos_right_knee;
}

/** Get pos_right_knee value at given index.
 * 
      Right knee position vector.
 * @param index index of value
 * @return pos_right_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_knee(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_knee[index];
}

/** Get maximum length of pos_right_knee value.
 * @return length of pos_right_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_knee() const
{
  return 3;
}

/** Set pos_right_knee value.
 * 
      Right knee position vector.
 * @param new_pos_right_knee new pos_right_knee value
 */
void
HumanSkeletonInterface::set_pos_right_knee(const float * new_pos_right_knee)
{
  memcpy(data->pos_right_knee, new_pos_right_knee, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_knee value at given index.
 * 
      Right knee position vector.
 * @param new_pos_right_knee new pos_right_knee value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_knee(unsigned int index, const float new_pos_right_knee)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_knee[index] = new_pos_right_knee;
  data_changed = true;
}
/** Get pos_right_knee_confidence value.
 * 
      Right knee position confidence.
 * @return pos_right_knee_confidence value
 */
float
HumanSkeletonInterface::pos_right_knee_confidence() const
{
  return data->pos_right_knee_confidence;
}

/** Get maximum length of pos_right_knee_confidence value.
 * @return length of pos_right_knee_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_knee_confidence() const
{
  return 1;
}

/** Set pos_right_knee_confidence value.
 * 
      Right knee position confidence.
 * @param new_pos_right_knee_confidence new pos_right_knee_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_knee_confidence(const float new_pos_right_knee_confidence)
{
  data->pos_right_knee_confidence = new_pos_right_knee_confidence;
  data_changed = true;
}

/** Get pos_right_ankle value.
 * 
      Right ankle position vector.
 * @return pos_right_ankle value
 */
float *
HumanSkeletonInterface::pos_right_ankle() const
{
  return data->pos_right_ankle;
}

/** Get pos_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param index index of value
 * @return pos_right_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_ankle(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_ankle[index];
}

/** Get maximum length of pos_right_ankle value.
 * @return length of pos_right_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_ankle() const
{
  return 3;
}

/** Set pos_right_ankle value.
 * 
      Right ankle position vector.
 * @param new_pos_right_ankle new pos_right_ankle value
 */
void
HumanSkeletonInterface::set_pos_right_ankle(const float * new_pos_right_ankle)
{
  memcpy(data->pos_right_ankle, new_pos_right_ankle, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param new_pos_right_ankle new pos_right_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_ankle(unsigned int index, const float new_pos_right_ankle)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_ankle[index] = new_pos_right_ankle;
  data_changed = true;
}
/** Get pos_right_ankle_confidence value.
 * 
      Right ankle position confidence.
 * @return pos_right_ankle_confidence value
 */
float
HumanSkeletonInterface::pos_right_ankle_confidence() const
{
  return data->pos_right_ankle_confidence;
}

/** Get maximum length of pos_right_ankle_confidence value.
 * @return length of pos_right_ankle_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_ankle_confidence() const
{
  return 1;
}

/** Set pos_right_ankle_confidence value.
 * 
      Right ankle position confidence.
 * @param new_pos_right_ankle_confidence new pos_right_ankle_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_ankle_confidence(const float new_pos_right_ankle_confidence)
{
  data->pos_right_ankle_confidence = new_pos_right_ankle_confidence;
  data_changed = true;
}

/** Get pos_right_foot value.
 * 
      Right foot position vector.
 * @return pos_right_foot value
 */
float *
HumanSkeletonInterface::pos_right_foot() const
{
  return data->pos_right_foot;
}

/** Get pos_right_foot value at given index.
 * 
      Right foot position vector.
 * @param index index of value
 * @return pos_right_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::pos_right_foot(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->pos_right_foot[index];
}

/** Get maximum length of pos_right_foot value.
 * @return length of pos_right_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_foot() const
{
  return 3;
}

/** Set pos_right_foot value.
 * 
      Right foot position vector.
 * @param new_pos_right_foot new pos_right_foot value
 */
void
HumanSkeletonInterface::set_pos_right_foot(const float * new_pos_right_foot)
{
  memcpy(data->pos_right_foot, new_pos_right_foot, sizeof(float) * 3);
  data_changed = true;
}

/** Set pos_right_foot value at given index.
 * 
      Right foot position vector.
 * @param new_pos_right_foot new pos_right_foot value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_pos_right_foot(unsigned int index, const float new_pos_right_foot)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->pos_right_foot[index] = new_pos_right_foot;
  data_changed = true;
}
/** Get pos_right_foot_confidence value.
 * 
      Right foot position confidence.
 * @return pos_right_foot_confidence value
 */
float
HumanSkeletonInterface::pos_right_foot_confidence() const
{
  return data->pos_right_foot_confidence;
}

/** Get maximum length of pos_right_foot_confidence value.
 * @return length of pos_right_foot_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_pos_right_foot_confidence() const
{
  return 1;
}

/** Set pos_right_foot_confidence value.
 * 
      Right foot position confidence.
 * @param new_pos_right_foot_confidence new pos_right_foot_confidence value
 */
void
HumanSkeletonInterface::set_pos_right_foot_confidence(const float new_pos_right_foot_confidence)
{
  data->pos_right_foot_confidence = new_pos_right_foot_confidence;
  data_changed = true;
}

/** Get ori_head value.
 * Head position vector.
 * @return ori_head value
 */
float *
HumanSkeletonInterface::ori_head() const
{
  return data->ori_head;
}

/** Get ori_head value at given index.
 * Head position vector.
 * @param index index of value
 * @return ori_head value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_head(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_head[index];
}

/** Get maximum length of ori_head value.
 * @return length of ori_head value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_head() const
{
  return 9;
}

/** Set ori_head value.
 * Head position vector.
 * @param new_ori_head new ori_head value
 */
void
HumanSkeletonInterface::set_ori_head(const float * new_ori_head)
{
  memcpy(data->ori_head, new_ori_head, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_head value at given index.
 * Head position vector.
 * @param new_ori_head new ori_head value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_head(unsigned int index, const float new_ori_head)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_head[index] = new_ori_head;
  data_changed = true;
}
/** Get ori_head_confidence value.
 * 
      Head position confidence.
 * @return ori_head_confidence value
 */
float
HumanSkeletonInterface::ori_head_confidence() const
{
  return data->ori_head_confidence;
}

/** Get maximum length of ori_head_confidence value.
 * @return length of ori_head_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_head_confidence() const
{
  return 1;
}

/** Set ori_head_confidence value.
 * 
      Head position confidence.
 * @param new_ori_head_confidence new ori_head_confidence value
 */
void
HumanSkeletonInterface::set_ori_head_confidence(const float new_ori_head_confidence)
{
  data->ori_head_confidence = new_ori_head_confidence;
  data_changed = true;
}

/** Get ori_neck value.
 * Neck position vector.
 * @return ori_neck value
 */
float *
HumanSkeletonInterface::ori_neck() const
{
  return data->ori_neck;
}

/** Get ori_neck value at given index.
 * Neck position vector.
 * @param index index of value
 * @return ori_neck value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_neck(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_neck[index];
}

/** Get maximum length of ori_neck value.
 * @return length of ori_neck value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_neck() const
{
  return 9;
}

/** Set ori_neck value.
 * Neck position vector.
 * @param new_ori_neck new ori_neck value
 */
void
HumanSkeletonInterface::set_ori_neck(const float * new_ori_neck)
{
  memcpy(data->ori_neck, new_ori_neck, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_neck value at given index.
 * Neck position vector.
 * @param new_ori_neck new ori_neck value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_neck(unsigned int index, const float new_ori_neck)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_neck[index] = new_ori_neck;
  data_changed = true;
}
/** Get ori_neck_confidence value.
 * 
      Neck position confidence.
 * @return ori_neck_confidence value
 */
float
HumanSkeletonInterface::ori_neck_confidence() const
{
  return data->ori_neck_confidence;
}

/** Get maximum length of ori_neck_confidence value.
 * @return length of ori_neck_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_neck_confidence() const
{
  return 1;
}

/** Set ori_neck_confidence value.
 * 
      Neck position confidence.
 * @param new_ori_neck_confidence new ori_neck_confidence value
 */
void
HumanSkeletonInterface::set_ori_neck_confidence(const float new_ori_neck_confidence)
{
  data->ori_neck_confidence = new_ori_neck_confidence;
  data_changed = true;
}

/** Get ori_torso value.
 * Torso position vector.
 * @return ori_torso value
 */
float *
HumanSkeletonInterface::ori_torso() const
{
  return data->ori_torso;
}

/** Get ori_torso value at given index.
 * Torso position vector.
 * @param index index of value
 * @return ori_torso value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_torso(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_torso[index];
}

/** Get maximum length of ori_torso value.
 * @return length of ori_torso value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_torso() const
{
  return 9;
}

/** Set ori_torso value.
 * Torso position vector.
 * @param new_ori_torso new ori_torso value
 */
void
HumanSkeletonInterface::set_ori_torso(const float * new_ori_torso)
{
  memcpy(data->ori_torso, new_ori_torso, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_torso value at given index.
 * Torso position vector.
 * @param new_ori_torso new ori_torso value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_torso(unsigned int index, const float new_ori_torso)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_torso[index] = new_ori_torso;
  data_changed = true;
}
/** Get ori_torso_confidence value.
 * 
      Torso position confidence.
 * @return ori_torso_confidence value
 */
float
HumanSkeletonInterface::ori_torso_confidence() const
{
  return data->ori_torso_confidence;
}

/** Get maximum length of ori_torso_confidence value.
 * @return length of ori_torso_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_torso_confidence() const
{
  return 1;
}

/** Set ori_torso_confidence value.
 * 
      Torso position confidence.
 * @param new_ori_torso_confidence new ori_torso_confidence value
 */
void
HumanSkeletonInterface::set_ori_torso_confidence(const float new_ori_torso_confidence)
{
  data->ori_torso_confidence = new_ori_torso_confidence;
  data_changed = true;
}

/** Get ori_waist value.
 * Waist position vector.
 * @return ori_waist value
 */
float *
HumanSkeletonInterface::ori_waist() const
{
  return data->ori_waist;
}

/** Get ori_waist value at given index.
 * Waist position vector.
 * @param index index of value
 * @return ori_waist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_waist(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_waist[index];
}

/** Get maximum length of ori_waist value.
 * @return length of ori_waist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_waist() const
{
  return 9;
}

/** Set ori_waist value.
 * Waist position vector.
 * @param new_ori_waist new ori_waist value
 */
void
HumanSkeletonInterface::set_ori_waist(const float * new_ori_waist)
{
  memcpy(data->ori_waist, new_ori_waist, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_waist value at given index.
 * Waist position vector.
 * @param new_ori_waist new ori_waist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_waist(unsigned int index, const float new_ori_waist)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_waist[index] = new_ori_waist;
  data_changed = true;
}
/** Get ori_waist_confidence value.
 * 
      Waist position confidence.
 * @return ori_waist_confidence value
 */
float
HumanSkeletonInterface::ori_waist_confidence() const
{
  return data->ori_waist_confidence;
}

/** Get maximum length of ori_waist_confidence value.
 * @return length of ori_waist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_waist_confidence() const
{
  return 1;
}

/** Set ori_waist_confidence value.
 * 
      Waist position confidence.
 * @param new_ori_waist_confidence new ori_waist_confidence value
 */
void
HumanSkeletonInterface::set_ori_waist_confidence(const float new_ori_waist_confidence)
{
  data->ori_waist_confidence = new_ori_waist_confidence;
  data_changed = true;
}

/** Get ori_left_collar value.
 * 
      Left position vector.
 * @return ori_left_collar value
 */
float *
HumanSkeletonInterface::ori_left_collar() const
{
  return data->ori_left_collar;
}

/** Get ori_left_collar value at given index.
 * 
      Left position vector.
 * @param index index of value
 * @return ori_left_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_collar(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_collar[index];
}

/** Get maximum length of ori_left_collar value.
 * @return length of ori_left_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_collar() const
{
  return 9;
}

/** Set ori_left_collar value.
 * 
      Left position vector.
 * @param new_ori_left_collar new ori_left_collar value
 */
void
HumanSkeletonInterface::set_ori_left_collar(const float * new_ori_left_collar)
{
  memcpy(data->ori_left_collar, new_ori_left_collar, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_collar value at given index.
 * 
      Left position vector.
 * @param new_ori_left_collar new ori_left_collar value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_collar(unsigned int index, const float new_ori_left_collar)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_collar[index] = new_ori_left_collar;
  data_changed = true;
}
/** Get ori_left_collar_confidence value.
 * 
      Left position confidence.
 * @return ori_left_collar_confidence value
 */
float
HumanSkeletonInterface::ori_left_collar_confidence() const
{
  return data->ori_left_collar_confidence;
}

/** Get maximum length of ori_left_collar_confidence value.
 * @return length of ori_left_collar_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_collar_confidence() const
{
  return 1;
}

/** Set ori_left_collar_confidence value.
 * 
      Left position confidence.
 * @param new_ori_left_collar_confidence new ori_left_collar_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_collar_confidence(const float new_ori_left_collar_confidence)
{
  data->ori_left_collar_confidence = new_ori_left_collar_confidence;
  data_changed = true;
}

/** Get ori_left_shoulder value.
 * 
      Left shoulder position vector.
 * @return ori_left_shoulder value
 */
float *
HumanSkeletonInterface::ori_left_shoulder() const
{
  return data->ori_left_shoulder;
}

/** Get ori_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param index index of value
 * @return ori_left_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_shoulder(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_shoulder[index];
}

/** Get maximum length of ori_left_shoulder value.
 * @return length of ori_left_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_shoulder() const
{
  return 9;
}

/** Set ori_left_shoulder value.
 * 
      Left shoulder position vector.
 * @param new_ori_left_shoulder new ori_left_shoulder value
 */
void
HumanSkeletonInterface::set_ori_left_shoulder(const float * new_ori_left_shoulder)
{
  memcpy(data->ori_left_shoulder, new_ori_left_shoulder, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_shoulder value at given index.
 * 
      Left shoulder position vector.
 * @param new_ori_left_shoulder new ori_left_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_shoulder(unsigned int index, const float new_ori_left_shoulder)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_shoulder[index] = new_ori_left_shoulder;
  data_changed = true;
}
/** Get ori_left_shoulder_confidence value.
 * 
      Left shoulder position confidence.
 * @return ori_left_shoulder_confidence value
 */
float
HumanSkeletonInterface::ori_left_shoulder_confidence() const
{
  return data->ori_left_shoulder_confidence;
}

/** Get maximum length of ori_left_shoulder_confidence value.
 * @return length of ori_left_shoulder_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_shoulder_confidence() const
{
  return 1;
}

/** Set ori_left_shoulder_confidence value.
 * 
      Left shoulder position confidence.
 * @param new_ori_left_shoulder_confidence new ori_left_shoulder_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_shoulder_confidence(const float new_ori_left_shoulder_confidence)
{
  data->ori_left_shoulder_confidence = new_ori_left_shoulder_confidence;
  data_changed = true;
}

/** Get ori_left_elbow value.
 * 
      Left elbow position vector.
 * @return ori_left_elbow value
 */
float *
HumanSkeletonInterface::ori_left_elbow() const
{
  return data->ori_left_elbow;
}

/** Get ori_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param index index of value
 * @return ori_left_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_elbow(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_elbow[index];
}

/** Get maximum length of ori_left_elbow value.
 * @return length of ori_left_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_elbow() const
{
  return 9;
}

/** Set ori_left_elbow value.
 * 
      Left elbow position vector.
 * @param new_ori_left_elbow new ori_left_elbow value
 */
void
HumanSkeletonInterface::set_ori_left_elbow(const float * new_ori_left_elbow)
{
  memcpy(data->ori_left_elbow, new_ori_left_elbow, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_elbow value at given index.
 * 
      Left elbow position vector.
 * @param new_ori_left_elbow new ori_left_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_elbow(unsigned int index, const float new_ori_left_elbow)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_elbow[index] = new_ori_left_elbow;
  data_changed = true;
}
/** Get ori_left_elbow_confidence value.
 * 
      Left elbow position confidence.
 * @return ori_left_elbow_confidence value
 */
float
HumanSkeletonInterface::ori_left_elbow_confidence() const
{
  return data->ori_left_elbow_confidence;
}

/** Get maximum length of ori_left_elbow_confidence value.
 * @return length of ori_left_elbow_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_elbow_confidence() const
{
  return 1;
}

/** Set ori_left_elbow_confidence value.
 * 
      Left elbow position confidence.
 * @param new_ori_left_elbow_confidence new ori_left_elbow_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_elbow_confidence(const float new_ori_left_elbow_confidence)
{
  data->ori_left_elbow_confidence = new_ori_left_elbow_confidence;
  data_changed = true;
}

/** Get ori_left_wrist value.
 * 
      Left wrist position vector.
 * @return ori_left_wrist value
 */
float *
HumanSkeletonInterface::ori_left_wrist() const
{
  return data->ori_left_wrist;
}

/** Get ori_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param index index of value
 * @return ori_left_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_wrist(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_wrist[index];
}

/** Get maximum length of ori_left_wrist value.
 * @return length of ori_left_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_wrist() const
{
  return 9;
}

/** Set ori_left_wrist value.
 * 
      Left wrist position vector.
 * @param new_ori_left_wrist new ori_left_wrist value
 */
void
HumanSkeletonInterface::set_ori_left_wrist(const float * new_ori_left_wrist)
{
  memcpy(data->ori_left_wrist, new_ori_left_wrist, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_wrist value at given index.
 * 
      Left wrist position vector.
 * @param new_ori_left_wrist new ori_left_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_wrist(unsigned int index, const float new_ori_left_wrist)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_wrist[index] = new_ori_left_wrist;
  data_changed = true;
}
/** Get ori_left_wrist_confidence value.
 * 
      Left wrist position confidence.
 * @return ori_left_wrist_confidence value
 */
float
HumanSkeletonInterface::ori_left_wrist_confidence() const
{
  return data->ori_left_wrist_confidence;
}

/** Get maximum length of ori_left_wrist_confidence value.
 * @return length of ori_left_wrist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_wrist_confidence() const
{
  return 1;
}

/** Set ori_left_wrist_confidence value.
 * 
      Left wrist position confidence.
 * @param new_ori_left_wrist_confidence new ori_left_wrist_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_wrist_confidence(const float new_ori_left_wrist_confidence)
{
  data->ori_left_wrist_confidence = new_ori_left_wrist_confidence;
  data_changed = true;
}

/** Get ori_left_hand value.
 * 
      Left hand position vector.
 * @return ori_left_hand value
 */
float *
HumanSkeletonInterface::ori_left_hand() const
{
  return data->ori_left_hand;
}

/** Get ori_left_hand value at given index.
 * 
      Left hand position vector.
 * @param index index of value
 * @return ori_left_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_hand(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_hand[index];
}

/** Get maximum length of ori_left_hand value.
 * @return length of ori_left_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_hand() const
{
  return 9;
}

/** Set ori_left_hand value.
 * 
      Left hand position vector.
 * @param new_ori_left_hand new ori_left_hand value
 */
void
HumanSkeletonInterface::set_ori_left_hand(const float * new_ori_left_hand)
{
  memcpy(data->ori_left_hand, new_ori_left_hand, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_hand value at given index.
 * 
      Left hand position vector.
 * @param new_ori_left_hand new ori_left_hand value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_hand(unsigned int index, const float new_ori_left_hand)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_hand[index] = new_ori_left_hand;
  data_changed = true;
}
/** Get ori_left_hand_confidence value.
 * 
      Left hand position confidence.
 * @return ori_left_hand_confidence value
 */
float
HumanSkeletonInterface::ori_left_hand_confidence() const
{
  return data->ori_left_hand_confidence;
}

/** Get maximum length of ori_left_hand_confidence value.
 * @return length of ori_left_hand_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_hand_confidence() const
{
  return 1;
}

/** Set ori_left_hand_confidence value.
 * 
      Left hand position confidence.
 * @param new_ori_left_hand_confidence new ori_left_hand_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_hand_confidence(const float new_ori_left_hand_confidence)
{
  data->ori_left_hand_confidence = new_ori_left_hand_confidence;
  data_changed = true;
}

/** Get ori_left_fingertip value.
 * 
      Left fingertip position vector.
 * @return ori_left_fingertip value
 */
float *
HumanSkeletonInterface::ori_left_fingertip() const
{
  return data->ori_left_fingertip;
}

/** Get ori_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param index index of value
 * @return ori_left_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_fingertip(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_fingertip[index];
}

/** Get maximum length of ori_left_fingertip value.
 * @return length of ori_left_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_fingertip() const
{
  return 9;
}

/** Set ori_left_fingertip value.
 * 
      Left fingertip position vector.
 * @param new_ori_left_fingertip new ori_left_fingertip value
 */
void
HumanSkeletonInterface::set_ori_left_fingertip(const float * new_ori_left_fingertip)
{
  memcpy(data->ori_left_fingertip, new_ori_left_fingertip, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_fingertip value at given index.
 * 
      Left fingertip position vector.
 * @param new_ori_left_fingertip new ori_left_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_fingertip(unsigned int index, const float new_ori_left_fingertip)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_fingertip[index] = new_ori_left_fingertip;
  data_changed = true;
}
/** Get ori_left_fingertip_confidence value.
 * 
      Left fingertip position confidence.
 * @return ori_left_fingertip_confidence value
 */
float
HumanSkeletonInterface::ori_left_fingertip_confidence() const
{
  return data->ori_left_fingertip_confidence;
}

/** Get maximum length of ori_left_fingertip_confidence value.
 * @return length of ori_left_fingertip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_fingertip_confidence() const
{
  return 1;
}

/** Set ori_left_fingertip_confidence value.
 * 
      Left fingertip position confidence.
 * @param new_ori_left_fingertip_confidence new ori_left_fingertip_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_fingertip_confidence(const float new_ori_left_fingertip_confidence)
{
  data->ori_left_fingertip_confidence = new_ori_left_fingertip_confidence;
  data_changed = true;
}

/** Get ori_right_collar value.
 * 
      Right collar position vector.
 * @return ori_right_collar value
 */
float *
HumanSkeletonInterface::ori_right_collar() const
{
  return data->ori_right_collar;
}

/** Get ori_right_collar value at given index.
 * 
      Right collar position vector.
 * @param index index of value
 * @return ori_right_collar value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_collar(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_collar[index];
}

/** Get maximum length of ori_right_collar value.
 * @return length of ori_right_collar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_collar() const
{
  return 9;
}

/** Set ori_right_collar value.
 * 
      Right collar position vector.
 * @param new_ori_right_collar new ori_right_collar value
 */
void
HumanSkeletonInterface::set_ori_right_collar(const float * new_ori_right_collar)
{
  memcpy(data->ori_right_collar, new_ori_right_collar, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_collar value at given index.
 * 
      Right collar position vector.
 * @param new_ori_right_collar new ori_right_collar value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_collar(unsigned int index, const float new_ori_right_collar)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_collar[index] = new_ori_right_collar;
  data_changed = true;
}
/** Get ori_right_collar_confidence value.
 * 
      Right collar position confidence.
 * @return ori_right_collar_confidence value
 */
float
HumanSkeletonInterface::ori_right_collar_confidence() const
{
  return data->ori_right_collar_confidence;
}

/** Get maximum length of ori_right_collar_confidence value.
 * @return length of ori_right_collar_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_collar_confidence() const
{
  return 1;
}

/** Set ori_right_collar_confidence value.
 * 
      Right collar position confidence.
 * @param new_ori_right_collar_confidence new ori_right_collar_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_collar_confidence(const float new_ori_right_collar_confidence)
{
  data->ori_right_collar_confidence = new_ori_right_collar_confidence;
  data_changed = true;
}

/** Get ori_right_shoulder value.
 * 
      Right shoulder position vector.
 * @return ori_right_shoulder value
 */
float *
HumanSkeletonInterface::ori_right_shoulder() const
{
  return data->ori_right_shoulder;
}

/** Get ori_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param index index of value
 * @return ori_right_shoulder value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_shoulder(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_shoulder[index];
}

/** Get maximum length of ori_right_shoulder value.
 * @return length of ori_right_shoulder value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_shoulder() const
{
  return 9;
}

/** Set ori_right_shoulder value.
 * 
      Right shoulder position vector.
 * @param new_ori_right_shoulder new ori_right_shoulder value
 */
void
HumanSkeletonInterface::set_ori_right_shoulder(const float * new_ori_right_shoulder)
{
  memcpy(data->ori_right_shoulder, new_ori_right_shoulder, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_shoulder value at given index.
 * 
      Right shoulder position vector.
 * @param new_ori_right_shoulder new ori_right_shoulder value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_shoulder(unsigned int index, const float new_ori_right_shoulder)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_shoulder[index] = new_ori_right_shoulder;
  data_changed = true;
}
/** Get ori_right_shoulder_confidence value.
 * 
      Right shoulder position confidence.
 * @return ori_right_shoulder_confidence value
 */
float
HumanSkeletonInterface::ori_right_shoulder_confidence() const
{
  return data->ori_right_shoulder_confidence;
}

/** Get maximum length of ori_right_shoulder_confidence value.
 * @return length of ori_right_shoulder_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_shoulder_confidence() const
{
  return 1;
}

/** Set ori_right_shoulder_confidence value.
 * 
      Right shoulder position confidence.
 * @param new_ori_right_shoulder_confidence new ori_right_shoulder_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_shoulder_confidence(const float new_ori_right_shoulder_confidence)
{
  data->ori_right_shoulder_confidence = new_ori_right_shoulder_confidence;
  data_changed = true;
}

/** Get ori_right_elbow value.
 * 
      Right elbow position vector.
 * @return ori_right_elbow value
 */
float *
HumanSkeletonInterface::ori_right_elbow() const
{
  return data->ori_right_elbow;
}

/** Get ori_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param index index of value
 * @return ori_right_elbow value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_elbow(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_elbow[index];
}

/** Get maximum length of ori_right_elbow value.
 * @return length of ori_right_elbow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_elbow() const
{
  return 9;
}

/** Set ori_right_elbow value.
 * 
      Right elbow position vector.
 * @param new_ori_right_elbow new ori_right_elbow value
 */
void
HumanSkeletonInterface::set_ori_right_elbow(const float * new_ori_right_elbow)
{
  memcpy(data->ori_right_elbow, new_ori_right_elbow, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_elbow value at given index.
 * 
      Right elbow position vector.
 * @param new_ori_right_elbow new ori_right_elbow value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_elbow(unsigned int index, const float new_ori_right_elbow)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_elbow[index] = new_ori_right_elbow;
  data_changed = true;
}
/** Get ori_right_elbow_confidence value.
 * 
      Right elbow position confidence.
 * @return ori_right_elbow_confidence value
 */
float
HumanSkeletonInterface::ori_right_elbow_confidence() const
{
  return data->ori_right_elbow_confidence;
}

/** Get maximum length of ori_right_elbow_confidence value.
 * @return length of ori_right_elbow_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_elbow_confidence() const
{
  return 1;
}

/** Set ori_right_elbow_confidence value.
 * 
      Right elbow position confidence.
 * @param new_ori_right_elbow_confidence new ori_right_elbow_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_elbow_confidence(const float new_ori_right_elbow_confidence)
{
  data->ori_right_elbow_confidence = new_ori_right_elbow_confidence;
  data_changed = true;
}

/** Get ori_right_wrist value.
 * 
      Right wrist position vector.
 * @return ori_right_wrist value
 */
float *
HumanSkeletonInterface::ori_right_wrist() const
{
  return data->ori_right_wrist;
}

/** Get ori_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param index index of value
 * @return ori_right_wrist value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_wrist(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_wrist[index];
}

/** Get maximum length of ori_right_wrist value.
 * @return length of ori_right_wrist value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_wrist() const
{
  return 9;
}

/** Set ori_right_wrist value.
 * 
      Right wrist position vector.
 * @param new_ori_right_wrist new ori_right_wrist value
 */
void
HumanSkeletonInterface::set_ori_right_wrist(const float * new_ori_right_wrist)
{
  memcpy(data->ori_right_wrist, new_ori_right_wrist, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_wrist value at given index.
 * 
      Right wrist position vector.
 * @param new_ori_right_wrist new ori_right_wrist value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_wrist(unsigned int index, const float new_ori_right_wrist)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_wrist[index] = new_ori_right_wrist;
  data_changed = true;
}
/** Get ori_right_wrist_confidence value.
 * 
      Right wrist position confidence.
 * @return ori_right_wrist_confidence value
 */
float
HumanSkeletonInterface::ori_right_wrist_confidence() const
{
  return data->ori_right_wrist_confidence;
}

/** Get maximum length of ori_right_wrist_confidence value.
 * @return length of ori_right_wrist_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_wrist_confidence() const
{
  return 1;
}

/** Set ori_right_wrist_confidence value.
 * 
      Right wrist position confidence.
 * @param new_ori_right_wrist_confidence new ori_right_wrist_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_wrist_confidence(const float new_ori_right_wrist_confidence)
{
  data->ori_right_wrist_confidence = new_ori_right_wrist_confidence;
  data_changed = true;
}

/** Get ori_right_hand value.
 * 
      Right hand position vector.
 * @return ori_right_hand value
 */
float *
HumanSkeletonInterface::ori_right_hand() const
{
  return data->ori_right_hand;
}

/** Get ori_right_hand value at given index.
 * 
      Right hand position vector.
 * @param index index of value
 * @return ori_right_hand value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_hand(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_hand[index];
}

/** Get maximum length of ori_right_hand value.
 * @return length of ori_right_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_hand() const
{
  return 9;
}

/** Set ori_right_hand value.
 * 
      Right hand position vector.
 * @param new_ori_right_hand new ori_right_hand value
 */
void
HumanSkeletonInterface::set_ori_right_hand(const float * new_ori_right_hand)
{
  memcpy(data->ori_right_hand, new_ori_right_hand, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_hand value at given index.
 * 
      Right hand position vector.
 * @param new_ori_right_hand new ori_right_hand value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_hand(unsigned int index, const float new_ori_right_hand)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_hand[index] = new_ori_right_hand;
  data_changed = true;
}
/** Get ori_right_hand_confidence value.
 * 
      Right hand position confidence.
 * @return ori_right_hand_confidence value
 */
float
HumanSkeletonInterface::ori_right_hand_confidence() const
{
  return data->ori_right_hand_confidence;
}

/** Get maximum length of ori_right_hand_confidence value.
 * @return length of ori_right_hand_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_hand_confidence() const
{
  return 1;
}

/** Set ori_right_hand_confidence value.
 * 
      Right hand position confidence.
 * @param new_ori_right_hand_confidence new ori_right_hand_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_hand_confidence(const float new_ori_right_hand_confidence)
{
  data->ori_right_hand_confidence = new_ori_right_hand_confidence;
  data_changed = true;
}

/** Get ori_right_fingertip value.
 * 
      Right fingertip position vector.
 * @return ori_right_fingertip value
 */
float *
HumanSkeletonInterface::ori_right_fingertip() const
{
  return data->ori_right_fingertip;
}

/** Get ori_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param index index of value
 * @return ori_right_fingertip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_fingertip(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_fingertip[index];
}

/** Get maximum length of ori_right_fingertip value.
 * @return length of ori_right_fingertip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_fingertip() const
{
  return 9;
}

/** Set ori_right_fingertip value.
 * 
      Right fingertip position vector.
 * @param new_ori_right_fingertip new ori_right_fingertip value
 */
void
HumanSkeletonInterface::set_ori_right_fingertip(const float * new_ori_right_fingertip)
{
  memcpy(data->ori_right_fingertip, new_ori_right_fingertip, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_fingertip value at given index.
 * 
      Right fingertip position vector.
 * @param new_ori_right_fingertip new ori_right_fingertip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_fingertip(unsigned int index, const float new_ori_right_fingertip)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_fingertip[index] = new_ori_right_fingertip;
  data_changed = true;
}
/** Get ori_right_fingertip_confidence value.
 * 
      Right fingertip position confidence.
 * @return ori_right_fingertip_confidence value
 */
float
HumanSkeletonInterface::ori_right_fingertip_confidence() const
{
  return data->ori_right_fingertip_confidence;
}

/** Get maximum length of ori_right_fingertip_confidence value.
 * @return length of ori_right_fingertip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_fingertip_confidence() const
{
  return 1;
}

/** Set ori_right_fingertip_confidence value.
 * 
      Right fingertip position confidence.
 * @param new_ori_right_fingertip_confidence new ori_right_fingertip_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_fingertip_confidence(const float new_ori_right_fingertip_confidence)
{
  data->ori_right_fingertip_confidence = new_ori_right_fingertip_confidence;
  data_changed = true;
}

/** Get ori_left_hip value.
 * 
      Left hip position vector.
 * @return ori_left_hip value
 */
float *
HumanSkeletonInterface::ori_left_hip() const
{
  return data->ori_left_hip;
}

/** Get ori_left_hip value at given index.
 * 
      Left hip position vector.
 * @param index index of value
 * @return ori_left_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_hip(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_hip[index];
}

/** Get maximum length of ori_left_hip value.
 * @return length of ori_left_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_hip() const
{
  return 9;
}

/** Set ori_left_hip value.
 * 
      Left hip position vector.
 * @param new_ori_left_hip new ori_left_hip value
 */
void
HumanSkeletonInterface::set_ori_left_hip(const float * new_ori_left_hip)
{
  memcpy(data->ori_left_hip, new_ori_left_hip, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_hip value at given index.
 * 
      Left hip position vector.
 * @param new_ori_left_hip new ori_left_hip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_hip(unsigned int index, const float new_ori_left_hip)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_hip[index] = new_ori_left_hip;
  data_changed = true;
}
/** Get ori_left_hip_confidence value.
 * 
      Left hip position confidence.
 * @return ori_left_hip_confidence value
 */
float
HumanSkeletonInterface::ori_left_hip_confidence() const
{
  return data->ori_left_hip_confidence;
}

/** Get maximum length of ori_left_hip_confidence value.
 * @return length of ori_left_hip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_hip_confidence() const
{
  return 1;
}

/** Set ori_left_hip_confidence value.
 * 
      Left hip position confidence.
 * @param new_ori_left_hip_confidence new ori_left_hip_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_hip_confidence(const float new_ori_left_hip_confidence)
{
  data->ori_left_hip_confidence = new_ori_left_hip_confidence;
  data_changed = true;
}

/** Get ori_left_knee value.
 * 
      Left knee position vector.
 * @return ori_left_knee value
 */
float *
HumanSkeletonInterface::ori_left_knee() const
{
  return data->ori_left_knee;
}

/** Get ori_left_knee value at given index.
 * 
      Left knee position vector.
 * @param index index of value
 * @return ori_left_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_knee(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_knee[index];
}

/** Get maximum length of ori_left_knee value.
 * @return length of ori_left_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_knee() const
{
  return 9;
}

/** Set ori_left_knee value.
 * 
      Left knee position vector.
 * @param new_ori_left_knee new ori_left_knee value
 */
void
HumanSkeletonInterface::set_ori_left_knee(const float * new_ori_left_knee)
{
  memcpy(data->ori_left_knee, new_ori_left_knee, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_knee value at given index.
 * 
      Left knee position vector.
 * @param new_ori_left_knee new ori_left_knee value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_knee(unsigned int index, const float new_ori_left_knee)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_knee[index] = new_ori_left_knee;
  data_changed = true;
}
/** Get ori_left_knee_confidence value.
 * 
      Left knee position confidence.
 * @return ori_left_knee_confidence value
 */
float
HumanSkeletonInterface::ori_left_knee_confidence() const
{
  return data->ori_left_knee_confidence;
}

/** Get maximum length of ori_left_knee_confidence value.
 * @return length of ori_left_knee_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_knee_confidence() const
{
  return 1;
}

/** Set ori_left_knee_confidence value.
 * 
      Left knee position confidence.
 * @param new_ori_left_knee_confidence new ori_left_knee_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_knee_confidence(const float new_ori_left_knee_confidence)
{
  data->ori_left_knee_confidence = new_ori_left_knee_confidence;
  data_changed = true;
}

/** Get ori_left_ankle value.
 * 
      Left ankle position vector.
 * @return ori_left_ankle value
 */
float *
HumanSkeletonInterface::ori_left_ankle() const
{
  return data->ori_left_ankle;
}

/** Get ori_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param index index of value
 * @return ori_left_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_ankle(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_ankle[index];
}

/** Get maximum length of ori_left_ankle value.
 * @return length of ori_left_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_ankle() const
{
  return 9;
}

/** Set ori_left_ankle value.
 * 
      Left ankle position vector.
 * @param new_ori_left_ankle new ori_left_ankle value
 */
void
HumanSkeletonInterface::set_ori_left_ankle(const float * new_ori_left_ankle)
{
  memcpy(data->ori_left_ankle, new_ori_left_ankle, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_ankle value at given index.
 * 
      Left ankle position vector.
 * @param new_ori_left_ankle new ori_left_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_ankle(unsigned int index, const float new_ori_left_ankle)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_ankle[index] = new_ori_left_ankle;
  data_changed = true;
}
/** Get ori_left_ankle_confidence value.
 * 
      Left ankle position confidence.
 * @return ori_left_ankle_confidence value
 */
float
HumanSkeletonInterface::ori_left_ankle_confidence() const
{
  return data->ori_left_ankle_confidence;
}

/** Get maximum length of ori_left_ankle_confidence value.
 * @return length of ori_left_ankle_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_ankle_confidence() const
{
  return 1;
}

/** Set ori_left_ankle_confidence value.
 * 
      Left ankle position confidence.
 * @param new_ori_left_ankle_confidence new ori_left_ankle_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_ankle_confidence(const float new_ori_left_ankle_confidence)
{
  data->ori_left_ankle_confidence = new_ori_left_ankle_confidence;
  data_changed = true;
}

/** Get ori_left_foot value.
 * 
      Left foot position vector.
 * @return ori_left_foot value
 */
float *
HumanSkeletonInterface::ori_left_foot() const
{
  return data->ori_left_foot;
}

/** Get ori_left_foot value at given index.
 * 
      Left foot position vector.
 * @param index index of value
 * @return ori_left_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_left_foot(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_left_foot[index];
}

/** Get maximum length of ori_left_foot value.
 * @return length of ori_left_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_foot() const
{
  return 9;
}

/** Set ori_left_foot value.
 * 
      Left foot position vector.
 * @param new_ori_left_foot new ori_left_foot value
 */
void
HumanSkeletonInterface::set_ori_left_foot(const float * new_ori_left_foot)
{
  memcpy(data->ori_left_foot, new_ori_left_foot, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_left_foot value at given index.
 * 
      Left foot position vector.
 * @param new_ori_left_foot new ori_left_foot value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_left_foot(unsigned int index, const float new_ori_left_foot)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_left_foot[index] = new_ori_left_foot;
  data_changed = true;
}
/** Get ori_left_foot_confidence value.
 * 
      Left foot position confidence.
 * @return ori_left_foot_confidence value
 */
float
HumanSkeletonInterface::ori_left_foot_confidence() const
{
  return data->ori_left_foot_confidence;
}

/** Get maximum length of ori_left_foot_confidence value.
 * @return length of ori_left_foot_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_left_foot_confidence() const
{
  return 1;
}

/** Set ori_left_foot_confidence value.
 * 
      Left foot position confidence.
 * @param new_ori_left_foot_confidence new ori_left_foot_confidence value
 */
void
HumanSkeletonInterface::set_ori_left_foot_confidence(const float new_ori_left_foot_confidence)
{
  data->ori_left_foot_confidence = new_ori_left_foot_confidence;
  data_changed = true;
}

/** Get ori_right_hip value.
 * 
      Right hip position vector.
 * @return ori_right_hip value
 */
float *
HumanSkeletonInterface::ori_right_hip() const
{
  return data->ori_right_hip;
}

/** Get ori_right_hip value at given index.
 * 
      Right hip position vector.
 * @param index index of value
 * @return ori_right_hip value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_hip(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_hip[index];
}

/** Get maximum length of ori_right_hip value.
 * @return length of ori_right_hip value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_hip() const
{
  return 9;
}

/** Set ori_right_hip value.
 * 
      Right hip position vector.
 * @param new_ori_right_hip new ori_right_hip value
 */
void
HumanSkeletonInterface::set_ori_right_hip(const float * new_ori_right_hip)
{
  memcpy(data->ori_right_hip, new_ori_right_hip, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_hip value at given index.
 * 
      Right hip position vector.
 * @param new_ori_right_hip new ori_right_hip value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_hip(unsigned int index, const float new_ori_right_hip)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_hip[index] = new_ori_right_hip;
  data_changed = true;
}
/** Get ori_right_hip_confidence value.
 * 
      Right hip position confidence.
 * @return ori_right_hip_confidence value
 */
float
HumanSkeletonInterface::ori_right_hip_confidence() const
{
  return data->ori_right_hip_confidence;
}

/** Get maximum length of ori_right_hip_confidence value.
 * @return length of ori_right_hip_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_hip_confidence() const
{
  return 1;
}

/** Set ori_right_hip_confidence value.
 * 
      Right hip position confidence.
 * @param new_ori_right_hip_confidence new ori_right_hip_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_hip_confidence(const float new_ori_right_hip_confidence)
{
  data->ori_right_hip_confidence = new_ori_right_hip_confidence;
  data_changed = true;
}

/** Get ori_right_knee value.
 * 
      Right knee position vector.
 * @return ori_right_knee value
 */
float *
HumanSkeletonInterface::ori_right_knee() const
{
  return data->ori_right_knee;
}

/** Get ori_right_knee value at given index.
 * 
      Right knee position vector.
 * @param index index of value
 * @return ori_right_knee value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_knee(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_knee[index];
}

/** Get maximum length of ori_right_knee value.
 * @return length of ori_right_knee value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_knee() const
{
  return 9;
}

/** Set ori_right_knee value.
 * 
      Right knee position vector.
 * @param new_ori_right_knee new ori_right_knee value
 */
void
HumanSkeletonInterface::set_ori_right_knee(const float * new_ori_right_knee)
{
  memcpy(data->ori_right_knee, new_ori_right_knee, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_knee value at given index.
 * 
      Right knee position vector.
 * @param new_ori_right_knee new ori_right_knee value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_knee(unsigned int index, const float new_ori_right_knee)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_knee[index] = new_ori_right_knee;
  data_changed = true;
}
/** Get ori_right_knee_confidence value.
 * 
      Right knee position confidence.
 * @return ori_right_knee_confidence value
 */
float
HumanSkeletonInterface::ori_right_knee_confidence() const
{
  return data->ori_right_knee_confidence;
}

/** Get maximum length of ori_right_knee_confidence value.
 * @return length of ori_right_knee_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_knee_confidence() const
{
  return 1;
}

/** Set ori_right_knee_confidence value.
 * 
      Right knee position confidence.
 * @param new_ori_right_knee_confidence new ori_right_knee_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_knee_confidence(const float new_ori_right_knee_confidence)
{
  data->ori_right_knee_confidence = new_ori_right_knee_confidence;
  data_changed = true;
}

/** Get ori_right_ankle value.
 * 
      Right ankle position vector.
 * @return ori_right_ankle value
 */
float *
HumanSkeletonInterface::ori_right_ankle() const
{
  return data->ori_right_ankle;
}

/** Get ori_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param index index of value
 * @return ori_right_ankle value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_ankle(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_ankle[index];
}

/** Get maximum length of ori_right_ankle value.
 * @return length of ori_right_ankle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_ankle() const
{
  return 9;
}

/** Set ori_right_ankle value.
 * 
      Right ankle position vector.
 * @param new_ori_right_ankle new ori_right_ankle value
 */
void
HumanSkeletonInterface::set_ori_right_ankle(const float * new_ori_right_ankle)
{
  memcpy(data->ori_right_ankle, new_ori_right_ankle, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_ankle value at given index.
 * 
      Right ankle position vector.
 * @param new_ori_right_ankle new ori_right_ankle value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_ankle(unsigned int index, const float new_ori_right_ankle)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_ankle[index] = new_ori_right_ankle;
  data_changed = true;
}
/** Get ori_right_ankle_confidence value.
 * 
      Right ankle position confidence.
 * @return ori_right_ankle_confidence value
 */
float
HumanSkeletonInterface::ori_right_ankle_confidence() const
{
  return data->ori_right_ankle_confidence;
}

/** Get maximum length of ori_right_ankle_confidence value.
 * @return length of ori_right_ankle_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_ankle_confidence() const
{
  return 1;
}

/** Set ori_right_ankle_confidence value.
 * 
      Right ankle position confidence.
 * @param new_ori_right_ankle_confidence new ori_right_ankle_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_ankle_confidence(const float new_ori_right_ankle_confidence)
{
  data->ori_right_ankle_confidence = new_ori_right_ankle_confidence;
  data_changed = true;
}

/** Get ori_right_foot value.
 * 
      Right foot position vector.
 * @return ori_right_foot value
 */
float *
HumanSkeletonInterface::ori_right_foot() const
{
  return data->ori_right_foot;
}

/** Get ori_right_foot value at given index.
 * 
      Right foot position vector.
 * @param index index of value
 * @return ori_right_foot value
 * @exception Exception thrown if index is out of bounds
 */
float
HumanSkeletonInterface::ori_right_foot(unsigned int index) const
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  return data->ori_right_foot[index];
}

/** Get maximum length of ori_right_foot value.
 * @return length of ori_right_foot value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_foot() const
{
  return 9;
}

/** Set ori_right_foot value.
 * 
      Right foot position vector.
 * @param new_ori_right_foot new ori_right_foot value
 */
void
HumanSkeletonInterface::set_ori_right_foot(const float * new_ori_right_foot)
{
  memcpy(data->ori_right_foot, new_ori_right_foot, sizeof(float) * 9);
  data_changed = true;
}

/** Set ori_right_foot value at given index.
 * 
      Right foot position vector.
 * @param new_ori_right_foot new ori_right_foot value
 * @param index index for of the value
 */
void
HumanSkeletonInterface::set_ori_right_foot(unsigned int index, const float new_ori_right_foot)
{
  if (index > 9) {
    throw Exception("Index value %u out of bounds (0..9)", index);
  }
  data->ori_right_foot[index] = new_ori_right_foot;
  data_changed = true;
}
/** Get ori_right_foot_confidence value.
 * 
      Right foot position confidence.
 * @return ori_right_foot_confidence value
 */
float
HumanSkeletonInterface::ori_right_foot_confidence() const
{
  return data->ori_right_foot_confidence;
}

/** Get maximum length of ori_right_foot_confidence value.
 * @return length of ori_right_foot_confidence value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanSkeletonInterface::maxlenof_ori_right_foot_confidence() const
{
  return 1;
}

/** Set ori_right_foot_confidence value.
 * 
      Right foot position confidence.
 * @param new_ori_right_foot_confidence new ori_right_foot_confidence value
 */
void
HumanSkeletonInterface::set_ori_right_foot_confidence(const float new_ori_right_foot_confidence)
{
  data->ori_right_foot_confidence = new_ori_right_foot_confidence;
  data_changed = true;
}

/* =========== message create =========== */
Message *
HumanSkeletonInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
HumanSkeletonInterface::copy_values(const Interface *other)
{
  const HumanSkeletonInterface *oi = dynamic_cast<const HumanSkeletonInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(HumanSkeletonInterface_data_t));
}

const char *
HumanSkeletonInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "State") == 0) {
    return tostring_State((State)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
HumanSkeletonInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(HumanSkeletonInterface)
/// @endcond


} // end namespace fawkes
