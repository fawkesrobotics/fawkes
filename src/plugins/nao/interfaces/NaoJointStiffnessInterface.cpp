
/***************************************************************************
 *  NaoJointStiffnessInterface.cpp - Fawkes BlackBoard Interface - NaoJointStiffnessInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2011  Tim Niemueller
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

#include <interfaces/NaoJointStiffnessInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NaoJointStiffnessInterface <interfaces/NaoJointStiffnessInterface.h>
 * NaoJointStiffnessInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to Nao joint stiffness.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NaoJointStiffnessInterface::NaoJointStiffnessInterface() : Interface()
{
  data_size = sizeof(NaoJointStiffnessInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NaoJointStiffnessInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "minimum", 1, &data->minimum);
  add_messageinfo("SetStiffnessMessage");
  add_messageinfo("SetBodyStiffnessMessage");
  add_messageinfo("SetStiffnessesMessage");
  unsigned char tmp_hash[] = {0x29, 0x35, 0x74, 0x2f, 0x4e, 0x93, 0x53, 0xc4, 0x28, 0x56, 0xc8, 0x4a, 0x66, 0x81, 0xd6, 0x6d};
  set_hash(tmp_hash);
}

/** Destructor */
NaoJointStiffnessInterface::~NaoJointStiffnessInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoJointStiffnessInterface::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoJointStiffnessInterface::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
  data_changed = true;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoJointStiffnessInterface::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoJointStiffnessInterface::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
  data_changed = true;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoJointStiffnessInterface::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoJointStiffnessInterface::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
  data_changed = true;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoJointStiffnessInterface::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoJointStiffnessInterface::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
  data_changed = true;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoJointStiffnessInterface::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoJointStiffnessInterface::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
  data_changed = true;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoJointStiffnessInterface::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoJointStiffnessInterface::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
  data_changed = true;
}

/** Get l_wrist_yaw value.
 * Left wrist yaw
 * @return l_wrist_yaw value
 */
float
NaoJointStiffnessInterface::l_wrist_yaw() const
{
  return data->l_wrist_yaw;
}

/** Get maximum length of l_wrist_yaw value.
 * @return length of l_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_wrist_yaw() const
{
  return 1;
}

/** Set l_wrist_yaw value.
 * Left wrist yaw
 * @param new_l_wrist_yaw new l_wrist_yaw value
 */
void
NaoJointStiffnessInterface::set_l_wrist_yaw(const float new_l_wrist_yaw)
{
  data->l_wrist_yaw = new_l_wrist_yaw;
  data_changed = true;
}

/** Get l_hand value.
 * Left hand
 * @return l_hand value
 */
float
NaoJointStiffnessInterface::l_hand() const
{
  return data->l_hand;
}

/** Get maximum length of l_hand value.
 * @return length of l_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_hand() const
{
  return 1;
}

/** Set l_hand value.
 * Left hand
 * @param new_l_hand new l_hand value
 */
void
NaoJointStiffnessInterface::set_l_hand(const float new_l_hand)
{
  data->l_hand = new_l_hand;
  data_changed = true;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoJointStiffnessInterface::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoJointStiffnessInterface::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
  data_changed = true;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoJointStiffnessInterface::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoJointStiffnessInterface::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
  data_changed = true;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoJointStiffnessInterface::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoJointStiffnessInterface::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
  data_changed = true;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoJointStiffnessInterface::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoJointStiffnessInterface::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
  data_changed = true;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoJointStiffnessInterface::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoJointStiffnessInterface::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
  data_changed = true;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoJointStiffnessInterface::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoJointStiffnessInterface::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
  data_changed = true;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoJointStiffnessInterface::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoJointStiffnessInterface::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
  data_changed = true;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoJointStiffnessInterface::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoJointStiffnessInterface::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
  data_changed = true;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoJointStiffnessInterface::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoJointStiffnessInterface::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
  data_changed = true;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoJointStiffnessInterface::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoJointStiffnessInterface::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
  data_changed = true;
}

/** Get r_wrist_yaw value.
 * Right wrist yaw
 * @return r_wrist_yaw value
 */
float
NaoJointStiffnessInterface::r_wrist_yaw() const
{
  return data->r_wrist_yaw;
}

/** Get maximum length of r_wrist_yaw value.
 * @return length of r_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_wrist_yaw() const
{
  return 1;
}

/** Set r_wrist_yaw value.
 * Right wrist yaw
 * @param new_r_wrist_yaw new r_wrist_yaw value
 */
void
NaoJointStiffnessInterface::set_r_wrist_yaw(const float new_r_wrist_yaw)
{
  data->r_wrist_yaw = new_r_wrist_yaw;
  data_changed = true;
}

/** Get r_hand value.
 * Right hand
 * @return r_hand value
 */
float
NaoJointStiffnessInterface::r_hand() const
{
  return data->r_hand;
}

/** Get maximum length of r_hand value.
 * @return length of r_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_hand() const
{
  return 1;
}

/** Set r_hand value.
 * Right hand
 * @param new_r_hand new r_hand value
 */
void
NaoJointStiffnessInterface::set_r_hand(const float new_r_hand)
{
  data->r_hand = new_r_hand;
  data_changed = true;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoJointStiffnessInterface::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoJointStiffnessInterface::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
  data_changed = true;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoJointStiffnessInterface::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoJointStiffnessInterface::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
  data_changed = true;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoJointStiffnessInterface::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoJointStiffnessInterface::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
  data_changed = true;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoJointStiffnessInterface::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoJointStiffnessInterface::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
  data_changed = true;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoJointStiffnessInterface::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoJointStiffnessInterface::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
  data_changed = true;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoJointStiffnessInterface::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoJointStiffnessInterface::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
  data_changed = true;
}

/** Get minimum value.
 * 
      Minimum stiffness of all joints. On the RoboCup version of the Nao this
      ignores the hand and wrist values.
    
 * @return minimum value
 */
float
NaoJointStiffnessInterface::minimum() const
{
  return data->minimum;
}

/** Get maximum length of minimum value.
 * @return length of minimum value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::maxlenof_minimum() const
{
  return 1;
}

/** Set minimum value.
 * 
      Minimum stiffness of all joints. On the RoboCup version of the Nao this
      ignores the hand and wrist values.
    
 * @param new_minimum new minimum value
 */
void
NaoJointStiffnessInterface::set_minimum(const float new_minimum)
{
  data->minimum = new_minimum;
  data_changed = true;
}

/* =========== message create =========== */
Message *
NaoJointStiffnessInterface::create_message(const char *type) const
{
  if ( strncmp("SetStiffnessMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetStiffnessMessage();
  } else if ( strncmp("SetBodyStiffnessMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetBodyStiffnessMessage();
  } else if ( strncmp("SetStiffnessesMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetStiffnessesMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NaoJointStiffnessInterface::copy_values(const Interface *other)
{
  const NaoJointStiffnessInterface *oi = dynamic_cast<const NaoJointStiffnessInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NaoJointStiffnessInterface_data_t));
}

const char *
NaoJointStiffnessInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NaoJointStiffnessInterface::SetStiffnessMessage <interfaces/NaoJointStiffnessInterface.h>
 * SetStiffnessMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servo initial value for servo
 * @param ini_value initial value for value
 * @param ini_time_sec initial value for time_sec
 */
NaoJointStiffnessInterface::SetStiffnessMessage::SetStiffnessMessage(const uint32_t ini_servo, const float ini_value, const float ini_time_sec) : Message("SetStiffnessMessage")
{
  data_size = sizeof(SetStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servo = ini_servo;
  data->value = ini_value;
  data->time_sec = ini_time_sec;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}
/** Constructor */
NaoJointStiffnessInterface::SetStiffnessMessage::SetStiffnessMessage() : Message("SetStiffnessMessage")
{
  data_size = sizeof(SetStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servo", 1, &data->servo);
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}

/** Destructor */
NaoJointStiffnessInterface::SetStiffnessMessage::~SetStiffnessMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointStiffnessInterface::SetStiffnessMessage::SetStiffnessMessage(const SetStiffnessMessage *m) : Message("SetStiffnessMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servo value.
 * 
      A concatenated list of SERVO_* constants from the
      NaoJointPositionInterface to define the servos that should
      execute the movement. The list shall consist of binary or'ed
      SERVO_* constants.
    
 * @return servo value
 */
uint32_t
NaoJointStiffnessInterface::SetStiffnessMessage::servo() const
{
  return data->servo;
}

/** Get maximum length of servo value.
 * @return length of servo value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessMessage::maxlenof_servo() const
{
  return 1;
}

/** Set servo value.
 * 
      A concatenated list of SERVO_* constants from the
      NaoJointPositionInterface to define the servos that should
      execute the movement. The list shall consist of binary or'ed
      SERVO_* constants.
    
 * @param new_servo new servo value
 */
void
NaoJointStiffnessInterface::SetStiffnessMessage::set_servo(const uint32_t new_servo)
{
  data->servo = new_servo;
}

/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoJointStiffnessInterface::SetStiffnessMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoJointStiffnessInterface::SetStiffnessMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get time_sec value.
 * Time when to reach the stiffness.
 * @return time_sec value
 */
float
NaoJointStiffnessInterface::SetStiffnessMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time when to reach the stiffness.
 * @param new_time_sec new time_sec value
 */
void
NaoJointStiffnessInterface::SetStiffnessMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointStiffnessInterface::SetStiffnessMessage::clone() const
{
  return new NaoJointStiffnessInterface::SetStiffnessMessage(this);
}
/** @class NaoJointStiffnessInterface::SetBodyStiffnessMessage <interfaces/NaoJointStiffnessInterface.h>
 * SetBodyStiffnessMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_value initial value for value
 * @param ini_time_sec initial value for time_sec
 */
NaoJointStiffnessInterface::SetBodyStiffnessMessage::SetBodyStiffnessMessage(const float ini_value, const float ini_time_sec) : Message("SetBodyStiffnessMessage")
{
  data_size = sizeof(SetBodyStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBodyStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->value = ini_value;
  data->time_sec = ini_time_sec;
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}
/** Constructor */
NaoJointStiffnessInterface::SetBodyStiffnessMessage::SetBodyStiffnessMessage() : Message("SetBodyStiffnessMessage")
{
  data_size = sizeof(SetBodyStiffnessMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetBodyStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "value", 1, &data->value);
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
}

/** Destructor */
NaoJointStiffnessInterface::SetBodyStiffnessMessage::~SetBodyStiffnessMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointStiffnessInterface::SetBodyStiffnessMessage::SetBodyStiffnessMessage(const SetBodyStiffnessMessage *m) : Message("SetBodyStiffnessMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetBodyStiffnessMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get value value.
 * Servo value to set for servos.
 * @return value value
 */
float
NaoJointStiffnessInterface::SetBodyStiffnessMessage::value() const
{
  return data->value;
}

/** Get maximum length of value value.
 * @return length of value value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetBodyStiffnessMessage::maxlenof_value() const
{
  return 1;
}

/** Set value value.
 * Servo value to set for servos.
 * @param new_value new value value
 */
void
NaoJointStiffnessInterface::SetBodyStiffnessMessage::set_value(const float new_value)
{
  data->value = new_value;
}

/** Get time_sec value.
 * Time when to reach the stiffness.
 * @return time_sec value
 */
float
NaoJointStiffnessInterface::SetBodyStiffnessMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetBodyStiffnessMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time when to reach the stiffness.
 * @param new_time_sec new time_sec value
 */
void
NaoJointStiffnessInterface::SetBodyStiffnessMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointStiffnessInterface::SetBodyStiffnessMessage::clone() const
{
  return new NaoJointStiffnessInterface::SetBodyStiffnessMessage(this);
}
/** @class NaoJointStiffnessInterface::SetStiffnessesMessage <interfaces/NaoJointStiffnessInterface.h>
 * SetStiffnessesMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_head_yaw initial value for head_yaw
 * @param ini_head_pitch initial value for head_pitch
 * @param ini_l_shoulder_pitch initial value for l_shoulder_pitch
 * @param ini_l_shoulder_roll initial value for l_shoulder_roll
 * @param ini_l_elbow_yaw initial value for l_elbow_yaw
 * @param ini_l_elbow_roll initial value for l_elbow_roll
 * @param ini_l_wrist_yaw initial value for l_wrist_yaw
 * @param ini_l_hand initial value for l_hand
 * @param ini_l_hip_yaw_pitch initial value for l_hip_yaw_pitch
 * @param ini_l_hip_roll initial value for l_hip_roll
 * @param ini_l_hip_pitch initial value for l_hip_pitch
 * @param ini_l_knee_pitch initial value for l_knee_pitch
 * @param ini_l_ankle_pitch initial value for l_ankle_pitch
 * @param ini_l_ankle_roll initial value for l_ankle_roll
 * @param ini_r_shoulder_pitch initial value for r_shoulder_pitch
 * @param ini_r_shoulder_roll initial value for r_shoulder_roll
 * @param ini_r_elbow_yaw initial value for r_elbow_yaw
 * @param ini_r_wrist_yaw initial value for r_wrist_yaw
 * @param ini_r_hand initial value for r_hand
 * @param ini_r_hip_yaw_pitch initial value for r_hip_yaw_pitch
 * @param ini_r_hip_roll initial value for r_hip_roll
 * @param ini_r_hip_pitch initial value for r_hip_pitch
 * @param ini_r_knee_pitch initial value for r_knee_pitch
 * @param ini_r_ankle_pitch initial value for r_ankle_pitch
 * @param ini_r_ankle_roll initial value for r_ankle_roll
 * @param ini_r_elbow_roll initial value for r_elbow_roll
 */
NaoJointStiffnessInterface::SetStiffnessesMessage::SetStiffnessesMessage(const float ini_time_sec, const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_wrist_yaw, const float ini_l_hand, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_wrist_yaw, const float ini_r_hand, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll, const float ini_r_elbow_roll) : Message("SetStiffnessesMessage")
{
  data_size = sizeof(SetStiffnessesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStiffnessesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->head_yaw = ini_head_yaw;
  data->head_pitch = ini_head_pitch;
  data->l_shoulder_pitch = ini_l_shoulder_pitch;
  data->l_shoulder_roll = ini_l_shoulder_roll;
  data->l_elbow_yaw = ini_l_elbow_yaw;
  data->l_elbow_roll = ini_l_elbow_roll;
  data->l_wrist_yaw = ini_l_wrist_yaw;
  data->l_hand = ini_l_hand;
  data->l_hip_yaw_pitch = ini_l_hip_yaw_pitch;
  data->l_hip_roll = ini_l_hip_roll;
  data->l_hip_pitch = ini_l_hip_pitch;
  data->l_knee_pitch = ini_l_knee_pitch;
  data->l_ankle_pitch = ini_l_ankle_pitch;
  data->l_ankle_roll = ini_l_ankle_roll;
  data->r_shoulder_pitch = ini_r_shoulder_pitch;
  data->r_shoulder_roll = ini_r_shoulder_roll;
  data->r_elbow_yaw = ini_r_elbow_yaw;
  data->r_wrist_yaw = ini_r_wrist_yaw;
  data->r_hand = ini_r_hand;
  data->r_hip_yaw_pitch = ini_r_hip_yaw_pitch;
  data->r_hip_roll = ini_r_hip_roll;
  data->r_hip_pitch = ini_r_hip_pitch;
  data->r_knee_pitch = ini_r_knee_pitch;
  data->r_ankle_pitch = ini_r_ankle_pitch;
  data->r_ankle_roll = ini_r_ankle_roll;
  data->r_elbow_roll = ini_r_elbow_roll;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
}
/** Constructor */
NaoJointStiffnessInterface::SetStiffnessesMessage::SetStiffnessesMessage() : Message("SetStiffnessesMessage")
{
  data_size = sizeof(SetStiffnessesMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStiffnessesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "head_yaw", 1, &data->head_yaw);
  add_fieldinfo(IFT_FLOAT, "head_pitch", 1, &data->head_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_pitch", 1, &data->l_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "l_shoulder_roll", 1, &data->l_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "l_elbow_yaw", 1, &data->l_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "l_elbow_roll", 1, &data->l_elbow_roll);
  add_fieldinfo(IFT_FLOAT, "l_wrist_yaw", 1, &data->l_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "l_hand", 1, &data->l_hand);
  add_fieldinfo(IFT_FLOAT, "l_hip_yaw_pitch", 1, &data->l_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "l_hip_roll", 1, &data->l_hip_roll);
  add_fieldinfo(IFT_FLOAT, "l_hip_pitch", 1, &data->l_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "l_knee_pitch", 1, &data->l_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_pitch", 1, &data->l_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "l_ankle_roll", 1, &data->l_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_pitch", 1, &data->r_shoulder_pitch);
  add_fieldinfo(IFT_FLOAT, "r_shoulder_roll", 1, &data->r_shoulder_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_yaw", 1, &data->r_elbow_yaw);
  add_fieldinfo(IFT_FLOAT, "r_wrist_yaw", 1, &data->r_wrist_yaw);
  add_fieldinfo(IFT_FLOAT, "r_hand", 1, &data->r_hand);
  add_fieldinfo(IFT_FLOAT, "r_hip_yaw_pitch", 1, &data->r_hip_yaw_pitch);
  add_fieldinfo(IFT_FLOAT, "r_hip_roll", 1, &data->r_hip_roll);
  add_fieldinfo(IFT_FLOAT, "r_hip_pitch", 1, &data->r_hip_pitch);
  add_fieldinfo(IFT_FLOAT, "r_knee_pitch", 1, &data->r_knee_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_pitch", 1, &data->r_ankle_pitch);
  add_fieldinfo(IFT_FLOAT, "r_ankle_roll", 1, &data->r_ankle_roll);
  add_fieldinfo(IFT_FLOAT, "r_elbow_roll", 1, &data->r_elbow_roll);
}

/** Destructor */
NaoJointStiffnessInterface::SetStiffnessesMessage::~SetStiffnessesMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NaoJointStiffnessInterface::SetStiffnessesMessage::SetStiffnessesMessage(const SetStiffnessesMessage *m) : Message("SetStiffnessesMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetStiffnessesMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time when to reach the stiffness.
 * @return time_sec value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time when to reach the stiffness.
 * @param new_time_sec new time_sec value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get head_yaw value.
 * Head yaw
 * @return head_yaw value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::head_yaw() const
{
  return data->head_yaw;
}

/** Get maximum length of head_yaw value.
 * @return length of head_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_head_yaw() const
{
  return 1;
}

/** Set head_yaw value.
 * Head yaw
 * @param new_head_yaw new head_yaw value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_head_yaw(const float new_head_yaw)
{
  data->head_yaw = new_head_yaw;
}

/** Get head_pitch value.
 * Head pitch
 * @return head_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::head_pitch() const
{
  return data->head_pitch;
}

/** Get maximum length of head_pitch value.
 * @return length of head_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_head_pitch() const
{
  return 1;
}

/** Set head_pitch value.
 * Head pitch
 * @param new_head_pitch new head_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_head_pitch(const float new_head_pitch)
{
  data->head_pitch = new_head_pitch;
}

/** Get l_shoulder_pitch value.
 * Left shoulder pitch
 * @return l_shoulder_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_shoulder_pitch() const
{
  return data->l_shoulder_pitch;
}

/** Get maximum length of l_shoulder_pitch value.
 * @return length of l_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_shoulder_pitch() const
{
  return 1;
}

/** Set l_shoulder_pitch value.
 * Left shoulder pitch
 * @param new_l_shoulder_pitch new l_shoulder_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_shoulder_pitch(const float new_l_shoulder_pitch)
{
  data->l_shoulder_pitch = new_l_shoulder_pitch;
}

/** Get l_shoulder_roll value.
 * Left shoulder roll
 * @return l_shoulder_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_shoulder_roll() const
{
  return data->l_shoulder_roll;
}

/** Get maximum length of l_shoulder_roll value.
 * @return length of l_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_shoulder_roll() const
{
  return 1;
}

/** Set l_shoulder_roll value.
 * Left shoulder roll
 * @param new_l_shoulder_roll new l_shoulder_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_shoulder_roll(const float new_l_shoulder_roll)
{
  data->l_shoulder_roll = new_l_shoulder_roll;
}

/** Get l_elbow_yaw value.
 * Left elbow yaw
 * @return l_elbow_yaw value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_elbow_yaw() const
{
  return data->l_elbow_yaw;
}

/** Get maximum length of l_elbow_yaw value.
 * @return length of l_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_elbow_yaw() const
{
  return 1;
}

/** Set l_elbow_yaw value.
 * Left elbow yaw
 * @param new_l_elbow_yaw new l_elbow_yaw value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_elbow_yaw(const float new_l_elbow_yaw)
{
  data->l_elbow_yaw = new_l_elbow_yaw;
}

/** Get l_elbow_roll value.
 * Left elbow roll
 * @return l_elbow_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_elbow_roll() const
{
  return data->l_elbow_roll;
}

/** Get maximum length of l_elbow_roll value.
 * @return length of l_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_elbow_roll() const
{
  return 1;
}

/** Set l_elbow_roll value.
 * Left elbow roll
 * @param new_l_elbow_roll new l_elbow_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_elbow_roll(const float new_l_elbow_roll)
{
  data->l_elbow_roll = new_l_elbow_roll;
}

/** Get l_wrist_yaw value.
 * Left wrist yaw
 * @return l_wrist_yaw value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_wrist_yaw() const
{
  return data->l_wrist_yaw;
}

/** Get maximum length of l_wrist_yaw value.
 * @return length of l_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_wrist_yaw() const
{
  return 1;
}

/** Set l_wrist_yaw value.
 * Left wrist yaw
 * @param new_l_wrist_yaw new l_wrist_yaw value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_wrist_yaw(const float new_l_wrist_yaw)
{
  data->l_wrist_yaw = new_l_wrist_yaw;
}

/** Get l_hand value.
 * Left hand
 * @return l_hand value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_hand() const
{
  return data->l_hand;
}

/** Get maximum length of l_hand value.
 * @return length of l_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_hand() const
{
  return 1;
}

/** Set l_hand value.
 * Left hand
 * @param new_l_hand new l_hand value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_hand(const float new_l_hand)
{
  data->l_hand = new_l_hand;
}

/** Get l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @return l_hip_yaw_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_hip_yaw_pitch() const
{
  return data->l_hip_yaw_pitch;
}

/** Get maximum length of l_hip_yaw_pitch value.
 * @return length of l_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_hip_yaw_pitch() const
{
  return 1;
}

/** Set l_hip_yaw_pitch value.
 * Left hip yaw pitch
 * @param new_l_hip_yaw_pitch new l_hip_yaw_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch)
{
  data->l_hip_yaw_pitch = new_l_hip_yaw_pitch;
}

/** Get l_hip_roll value.
 * Left hip roll
 * @return l_hip_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_hip_roll() const
{
  return data->l_hip_roll;
}

/** Get maximum length of l_hip_roll value.
 * @return length of l_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_hip_roll() const
{
  return 1;
}

/** Set l_hip_roll value.
 * Left hip roll
 * @param new_l_hip_roll new l_hip_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_hip_roll(const float new_l_hip_roll)
{
  data->l_hip_roll = new_l_hip_roll;
}

/** Get l_hip_pitch value.
 * Left hip pitch
 * @return l_hip_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_hip_pitch() const
{
  return data->l_hip_pitch;
}

/** Get maximum length of l_hip_pitch value.
 * @return length of l_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_hip_pitch() const
{
  return 1;
}

/** Set l_hip_pitch value.
 * Left hip pitch
 * @param new_l_hip_pitch new l_hip_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_hip_pitch(const float new_l_hip_pitch)
{
  data->l_hip_pitch = new_l_hip_pitch;
}

/** Get l_knee_pitch value.
 * Left knee pitch
 * @return l_knee_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_knee_pitch() const
{
  return data->l_knee_pitch;
}

/** Get maximum length of l_knee_pitch value.
 * @return length of l_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_knee_pitch() const
{
  return 1;
}

/** Set l_knee_pitch value.
 * Left knee pitch
 * @param new_l_knee_pitch new l_knee_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_knee_pitch(const float new_l_knee_pitch)
{
  data->l_knee_pitch = new_l_knee_pitch;
}

/** Get l_ankle_pitch value.
 * Left ankle pitch
 * @return l_ankle_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_ankle_pitch() const
{
  return data->l_ankle_pitch;
}

/** Get maximum length of l_ankle_pitch value.
 * @return length of l_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_ankle_pitch() const
{
  return 1;
}

/** Set l_ankle_pitch value.
 * Left ankle pitch
 * @param new_l_ankle_pitch new l_ankle_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_ankle_pitch(const float new_l_ankle_pitch)
{
  data->l_ankle_pitch = new_l_ankle_pitch;
}

/** Get l_ankle_roll value.
 * Left ankle roll
 * @return l_ankle_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::l_ankle_roll() const
{
  return data->l_ankle_roll;
}

/** Get maximum length of l_ankle_roll value.
 * @return length of l_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_l_ankle_roll() const
{
  return 1;
}

/** Set l_ankle_roll value.
 * Left ankle roll
 * @param new_l_ankle_roll new l_ankle_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_l_ankle_roll(const float new_l_ankle_roll)
{
  data->l_ankle_roll = new_l_ankle_roll;
}

/** Get r_shoulder_pitch value.
 * Right shoulder pitch
 * @return r_shoulder_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_shoulder_pitch() const
{
  return data->r_shoulder_pitch;
}

/** Get maximum length of r_shoulder_pitch value.
 * @return length of r_shoulder_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_shoulder_pitch() const
{
  return 1;
}

/** Set r_shoulder_pitch value.
 * Right shoulder pitch
 * @param new_r_shoulder_pitch new r_shoulder_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_shoulder_pitch(const float new_r_shoulder_pitch)
{
  data->r_shoulder_pitch = new_r_shoulder_pitch;
}

/** Get r_shoulder_roll value.
 * Right shoulder roll
 * @return r_shoulder_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_shoulder_roll() const
{
  return data->r_shoulder_roll;
}

/** Get maximum length of r_shoulder_roll value.
 * @return length of r_shoulder_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_shoulder_roll() const
{
  return 1;
}

/** Set r_shoulder_roll value.
 * Right shoulder roll
 * @param new_r_shoulder_roll new r_shoulder_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_shoulder_roll(const float new_r_shoulder_roll)
{
  data->r_shoulder_roll = new_r_shoulder_roll;
}

/** Get r_elbow_yaw value.
 * Right elbow yaw
 * @return r_elbow_yaw value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_elbow_yaw() const
{
  return data->r_elbow_yaw;
}

/** Get maximum length of r_elbow_yaw value.
 * @return length of r_elbow_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_elbow_yaw() const
{
  return 1;
}

/** Set r_elbow_yaw value.
 * Right elbow yaw
 * @param new_r_elbow_yaw new r_elbow_yaw value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_elbow_yaw(const float new_r_elbow_yaw)
{
  data->r_elbow_yaw = new_r_elbow_yaw;
}

/** Get r_wrist_yaw value.
 * Right wrist yaw
 * @return r_wrist_yaw value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_wrist_yaw() const
{
  return data->r_wrist_yaw;
}

/** Get maximum length of r_wrist_yaw value.
 * @return length of r_wrist_yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_wrist_yaw() const
{
  return 1;
}

/** Set r_wrist_yaw value.
 * Right wrist yaw
 * @param new_r_wrist_yaw new r_wrist_yaw value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_wrist_yaw(const float new_r_wrist_yaw)
{
  data->r_wrist_yaw = new_r_wrist_yaw;
}

/** Get r_hand value.
 * Right hand
 * @return r_hand value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_hand() const
{
  return data->r_hand;
}

/** Get maximum length of r_hand value.
 * @return length of r_hand value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_hand() const
{
  return 1;
}

/** Set r_hand value.
 * Right hand
 * @param new_r_hand new r_hand value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_hand(const float new_r_hand)
{
  data->r_hand = new_r_hand;
}

/** Get r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @return r_hip_yaw_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_hip_yaw_pitch() const
{
  return data->r_hip_yaw_pitch;
}

/** Get maximum length of r_hip_yaw_pitch value.
 * @return length of r_hip_yaw_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_hip_yaw_pitch() const
{
  return 1;
}

/** Set r_hip_yaw_pitch value.
 * Right hip yaw pitch
 * @param new_r_hip_yaw_pitch new r_hip_yaw_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch)
{
  data->r_hip_yaw_pitch = new_r_hip_yaw_pitch;
}

/** Get r_hip_roll value.
 * Right hip roll
 * @return r_hip_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_hip_roll() const
{
  return data->r_hip_roll;
}

/** Get maximum length of r_hip_roll value.
 * @return length of r_hip_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_hip_roll() const
{
  return 1;
}

/** Set r_hip_roll value.
 * Right hip roll
 * @param new_r_hip_roll new r_hip_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_hip_roll(const float new_r_hip_roll)
{
  data->r_hip_roll = new_r_hip_roll;
}

/** Get r_hip_pitch value.
 * Right hip pitch
 * @return r_hip_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_hip_pitch() const
{
  return data->r_hip_pitch;
}

/** Get maximum length of r_hip_pitch value.
 * @return length of r_hip_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_hip_pitch() const
{
  return 1;
}

/** Set r_hip_pitch value.
 * Right hip pitch
 * @param new_r_hip_pitch new r_hip_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_hip_pitch(const float new_r_hip_pitch)
{
  data->r_hip_pitch = new_r_hip_pitch;
}

/** Get r_knee_pitch value.
 * Right knee pitch
 * @return r_knee_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_knee_pitch() const
{
  return data->r_knee_pitch;
}

/** Get maximum length of r_knee_pitch value.
 * @return length of r_knee_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_knee_pitch() const
{
  return 1;
}

/** Set r_knee_pitch value.
 * Right knee pitch
 * @param new_r_knee_pitch new r_knee_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_knee_pitch(const float new_r_knee_pitch)
{
  data->r_knee_pitch = new_r_knee_pitch;
}

/** Get r_ankle_pitch value.
 * Right ankle pitch
 * @return r_ankle_pitch value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_ankle_pitch() const
{
  return data->r_ankle_pitch;
}

/** Get maximum length of r_ankle_pitch value.
 * @return length of r_ankle_pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_ankle_pitch() const
{
  return 1;
}

/** Set r_ankle_pitch value.
 * Right ankle pitch
 * @param new_r_ankle_pitch new r_ankle_pitch value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_ankle_pitch(const float new_r_ankle_pitch)
{
  data->r_ankle_pitch = new_r_ankle_pitch;
}

/** Get r_ankle_roll value.
 * Right ankle roll
 * @return r_ankle_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_ankle_roll() const
{
  return data->r_ankle_roll;
}

/** Get maximum length of r_ankle_roll value.
 * @return length of r_ankle_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_ankle_roll() const
{
  return 1;
}

/** Set r_ankle_roll value.
 * Right ankle roll
 * @param new_r_ankle_roll new r_ankle_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_ankle_roll(const float new_r_ankle_roll)
{
  data->r_ankle_roll = new_r_ankle_roll;
}

/** Get r_elbow_roll value.
 * Right elbow roll
 * @return r_elbow_roll value
 */
float
NaoJointStiffnessInterface::SetStiffnessesMessage::r_elbow_roll() const
{
  return data->r_elbow_roll;
}

/** Get maximum length of r_elbow_roll value.
 * @return length of r_elbow_roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NaoJointStiffnessInterface::SetStiffnessesMessage::maxlenof_r_elbow_roll() const
{
  return 1;
}

/** Set r_elbow_roll value.
 * Right elbow roll
 * @param new_r_elbow_roll new r_elbow_roll value
 */
void
NaoJointStiffnessInterface::SetStiffnessesMessage::set_r_elbow_roll(const float new_r_elbow_roll)
{
  data->r_elbow_roll = new_r_elbow_roll;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NaoJointStiffnessInterface::SetStiffnessesMessage::clone() const
{
  return new NaoJointStiffnessInterface::SetStiffnessesMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NaoJointStiffnessInterface::message_valid(const Message *message) const
{
  const SetStiffnessMessage *m0 = dynamic_cast<const SetStiffnessMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetBodyStiffnessMessage *m1 = dynamic_cast<const SetBodyStiffnessMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetStiffnessesMessage *m2 = dynamic_cast<const SetStiffnessesMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NaoJointStiffnessInterface)
/// @endcond


} // end namespace fawkes
