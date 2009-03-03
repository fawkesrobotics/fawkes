
/***************************************************************************
 *  HumanoidMotionInterface.cpp - Fawkes BlackBoard Interface - HumanoidMotionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
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

#include <interfaces/HumanoidMotionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class HumanoidMotionInterface <interfaces/HumanoidMotionInterface.h>
 * HumanoidMotionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides acces to basic humanoid motion patterns.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
HumanoidMotionInterface::HumanoidMotionInterface() : Interface()
{
  data_size = sizeof(HumanoidMotionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (HumanoidMotionInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_BOOL, "moving", 1, &data->moving);
  add_fieldinfo(Interface::IFT_FLOAT, "max_step_length", 1, &data->max_step_length);
  add_fieldinfo(Interface::IFT_FLOAT, "max_step_height", 1, &data->max_step_height);
  add_fieldinfo(Interface::IFT_FLOAT, "max_step_side", 1, &data->max_step_side);
  add_fieldinfo(Interface::IFT_FLOAT, "max_step_turn", 1, &data->max_step_turn);
  add_fieldinfo(Interface::IFT_FLOAT, "zmp_offset_forward", 1, &data->zmp_offset_forward);
  add_fieldinfo(Interface::IFT_FLOAT, "zmp_offset_sideward", 1, &data->zmp_offset_sideward);
  add_fieldinfo(Interface::IFT_FLOAT, "l_hip_roll_compensation", 1, &data->l_hip_roll_compensation);
  add_fieldinfo(Interface::IFT_FLOAT, "r_hip_roll_compensation", 1, &data->r_hip_roll_compensation);
  add_fieldinfo(Interface::IFT_FLOAT, "hip_height", 1, &data->hip_height);
  add_fieldinfo(Interface::IFT_FLOAT, "torso_sideward_orientation", 1, &data->torso_sideward_orientation);
  add_fieldinfo(Interface::IFT_BOOL, "arms_enabled", 1, &data->arms_enabled);
  add_fieldinfo(Interface::IFT_FLOAT, "shoulder_pitch_median", 1, &data->shoulder_pitch_median);
  add_fieldinfo(Interface::IFT_FLOAT, "shoulder_pitch_amplitude", 1, &data->shoulder_pitch_amplitude);
  add_fieldinfo(Interface::IFT_FLOAT, "elbow_roll_median", 1, &data->elbow_roll_median);
  add_fieldinfo(Interface::IFT_FLOAT, "elbow_roll_amplitude", 1, &data->elbow_roll_amplitude);
  add_fieldinfo(Interface::IFT_UINT, "msgid", 1, &data->msgid);
  unsigned char tmp_hash[] = {0x4, 0x3f, 0x29, 0xef, 0x3d, 0x76, 0x7a, 0x9a, 0x2f, 0x9, 0xcb, 0x3c, 0xb, 0x8a, 0x5f, 0x82};
  set_hash(tmp_hash);
}

/** Destructor */
HumanoidMotionInterface::~HumanoidMotionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get moving value.
 * True if the robot is currently moving.
 * @return moving value
 */
bool
HumanoidMotionInterface::is_moving() const
{
  return data->moving;
}

/** Get maximum length of moving value.
 * @return length of moving value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_moving() const
{
  return 1;
}

/** Set moving value.
 * True if the robot is currently moving.
 * @param new_moving new moving value
 */
void
HumanoidMotionInterface::set_moving(const bool new_moving)
{
  data->moving = new_moving;
}

/** Get supporting_leg value.
 * Marks the supporting leg
 * @return supporting_leg value
 */
HumanoidMotionInterface::LegEnum
HumanoidMotionInterface::supporting_leg() const
{
  return data->supporting_leg;
}

/** Get maximum length of supporting_leg value.
 * @return length of supporting_leg value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_supporting_leg() const
{
  return 1;
}

/** Set supporting_leg value.
 * Marks the supporting leg
 * @param new_supporting_leg new supporting_leg value
 */
void
HumanoidMotionInterface::set_supporting_leg(const LegEnum new_supporting_leg)
{
  data->supporting_leg = new_supporting_leg;
}

/** Get max_step_length value.
 * 
      Maximum length of a footstep in m.
    
 * @return max_step_length value
 */
float
HumanoidMotionInterface::max_step_length() const
{
  return data->max_step_length;
}

/** Get maximum length of max_step_length value.
 * @return length of max_step_length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_max_step_length() const
{
  return 1;
}

/** Set max_step_length value.
 * 
      Maximum length of a footstep in m.
    
 * @param new_max_step_length new max_step_length value
 */
void
HumanoidMotionInterface::set_max_step_length(const float new_max_step_length)
{
  data->max_step_length = new_max_step_length;
}

/** Get max_step_height value.
 * 
      Maxium height of a footstep cycloid in m.
    
 * @return max_step_height value
 */
float
HumanoidMotionInterface::max_step_height() const
{
  return data->max_step_height;
}

/** Get maximum length of max_step_height value.
 * @return length of max_step_height value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_max_step_height() const
{
  return 1;
}

/** Set max_step_height value.
 * 
      Maxium height of a footstep cycloid in m.
    
 * @param new_max_step_height new max_step_height value
 */
void
HumanoidMotionInterface::set_max_step_height(const float new_max_step_height)
{
  data->max_step_height = new_max_step_height;
}

/** Get max_step_side value.
 * 
      Maximum length of side step in m.
    
 * @return max_step_side value
 */
float
HumanoidMotionInterface::max_step_side() const
{
  return data->max_step_side;
}

/** Get maximum length of max_step_side value.
 * @return length of max_step_side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_max_step_side() const
{
  return 1;
}

/** Set max_step_side value.
 * 
      Maximum length of side step in m.
    
 * @param new_max_step_side new max_step_side value
 */
void
HumanoidMotionInterface::set_max_step_side(const float new_max_step_side)
{
  data->max_step_side = new_max_step_side;
}

/** Get max_step_turn value.
 * 
      Maximum change around vertical axis on radians per footstep.
    
 * @return max_step_turn value
 */
float
HumanoidMotionInterface::max_step_turn() const
{
  return data->max_step_turn;
}

/** Get maximum length of max_step_turn value.
 * @return length of max_step_turn value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_max_step_turn() const
{
  return 1;
}

/** Set max_step_turn value.
 * 
      Maximum change around vertical axis on radians per footstep.
    
 * @param new_max_step_turn new max_step_turn value
 */
void
HumanoidMotionInterface::set_max_step_turn(const float new_max_step_turn)
{
  data->max_step_turn = new_max_step_turn;
}

/** Get zmp_offset_forward value.
 * 
      Zero moment point offset in forward direction in m.
    
 * @return zmp_offset_forward value
 */
float
HumanoidMotionInterface::zmp_offset_forward() const
{
  return data->zmp_offset_forward;
}

/** Get maximum length of zmp_offset_forward value.
 * @return length of zmp_offset_forward value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_zmp_offset_forward() const
{
  return 1;
}

/** Set zmp_offset_forward value.
 * 
      Zero moment point offset in forward direction in m.
    
 * @param new_zmp_offset_forward new zmp_offset_forward value
 */
void
HumanoidMotionInterface::set_zmp_offset_forward(const float new_zmp_offset_forward)
{
  data->zmp_offset_forward = new_zmp_offset_forward;
}

/** Get zmp_offset_sideward value.
 * 
      Zero moment point offset in sideward direction in m.
    
 * @return zmp_offset_sideward value
 */
float
HumanoidMotionInterface::zmp_offset_sideward() const
{
  return data->zmp_offset_sideward;
}

/** Get maximum length of zmp_offset_sideward value.
 * @return length of zmp_offset_sideward value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_zmp_offset_sideward() const
{
  return 1;
}

/** Set zmp_offset_sideward value.
 * 
      Zero moment point offset in sideward direction in m.
    
 * @param new_zmp_offset_sideward new zmp_offset_sideward value
 */
void
HumanoidMotionInterface::set_zmp_offset_sideward(const float new_zmp_offset_sideward)
{
  data->zmp_offset_sideward = new_zmp_offset_sideward;
}

/** Get l_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return l_hip_roll_compensation value
 */
float
HumanoidMotionInterface::l_hip_roll_compensation() const
{
  return data->l_hip_roll_compensation;
}

/** Get maximum length of l_hip_roll_compensation value.
 * @return length of l_hip_roll_compensation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_l_hip_roll_compensation() const
{
  return 1;
}

/** Set l_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_l_hip_roll_compensation new l_hip_roll_compensation value
 */
void
HumanoidMotionInterface::set_l_hip_roll_compensation(const float new_l_hip_roll_compensation)
{
  data->l_hip_roll_compensation = new_l_hip_roll_compensation;
}

/** Get r_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return r_hip_roll_compensation value
 */
float
HumanoidMotionInterface::r_hip_roll_compensation() const
{
  return data->r_hip_roll_compensation;
}

/** Get maximum length of r_hip_roll_compensation value.
 * @return length of r_hip_roll_compensation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_r_hip_roll_compensation() const
{
  return 1;
}

/** Set r_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_r_hip_roll_compensation new r_hip_roll_compensation value
 */
void
HumanoidMotionInterface::set_r_hip_roll_compensation(const float new_r_hip_roll_compensation)
{
  data->r_hip_roll_compensation = new_r_hip_roll_compensation;
}

/** Get hip_height value.
 * 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return hip_height value
 */
float
HumanoidMotionInterface::hip_height() const
{
  return data->hip_height;
}

/** Get maximum length of hip_height value.
 * @return length of hip_height value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_hip_height() const
{
  return 1;
}

/** Set hip_height value.
 * 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_hip_height new hip_height value
 */
void
HumanoidMotionInterface::set_hip_height(const float new_hip_height)
{
  data->hip_height = new_hip_height;
}

/** Get torso_sideward_orientation value.
 * 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return torso_sideward_orientation value
 */
float
HumanoidMotionInterface::torso_sideward_orientation() const
{
  return data->torso_sideward_orientation;
}

/** Get maximum length of torso_sideward_orientation value.
 * @return length of torso_sideward_orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_torso_sideward_orientation() const
{
  return 1;
}

/** Set torso_sideward_orientation value.
 * 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_torso_sideward_orientation new torso_sideward_orientation value
 */
void
HumanoidMotionInterface::set_torso_sideward_orientation(const float new_torso_sideward_orientation)
{
  data->torso_sideward_orientation = new_torso_sideward_orientation;
}

/** Get arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @return arms_enabled value
 */
bool
HumanoidMotionInterface::is_arms_enabled() const
{
  return data->arms_enabled;
}

/** Get maximum length of arms_enabled value.
 * @return length of arms_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_arms_enabled() const
{
  return 1;
}

/** Set arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @param new_arms_enabled new arms_enabled value
 */
void
HumanoidMotionInterface::set_arms_enabled(const bool new_arms_enabled)
{
  data->arms_enabled = new_arms_enabled;
}

/** Get shoulder_pitch_median value.
 * 
      Median in radians of the shoulder pitch during walking.
    
 * @return shoulder_pitch_median value
 */
float
HumanoidMotionInterface::shoulder_pitch_median() const
{
  return data->shoulder_pitch_median;
}

/** Get maximum length of shoulder_pitch_median value.
 * @return length of shoulder_pitch_median value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_shoulder_pitch_median() const
{
  return 1;
}

/** Set shoulder_pitch_median value.
 * 
      Median in radians of the shoulder pitch during walking.
    
 * @param new_shoulder_pitch_median new shoulder_pitch_median value
 */
void
HumanoidMotionInterface::set_shoulder_pitch_median(const float new_shoulder_pitch_median)
{
  data->shoulder_pitch_median = new_shoulder_pitch_median;
}

/** Get shoulder_pitch_amplitude value.
 * 
      Amplitude of the shoulder pitch movement during walking.
    
 * @return shoulder_pitch_amplitude value
 */
float
HumanoidMotionInterface::shoulder_pitch_amplitude() const
{
  return data->shoulder_pitch_amplitude;
}

/** Get maximum length of shoulder_pitch_amplitude value.
 * @return length of shoulder_pitch_amplitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_shoulder_pitch_amplitude() const
{
  return 1;
}

/** Set shoulder_pitch_amplitude value.
 * 
      Amplitude of the shoulder pitch movement during walking.
    
 * @param new_shoulder_pitch_amplitude new shoulder_pitch_amplitude value
 */
void
HumanoidMotionInterface::set_shoulder_pitch_amplitude(const float new_shoulder_pitch_amplitude)
{
  data->shoulder_pitch_amplitude = new_shoulder_pitch_amplitude;
}

/** Get elbow_roll_median value.
 * 
      Median in radians of the elbow roll during walking.
    
 * @return elbow_roll_median value
 */
float
HumanoidMotionInterface::elbow_roll_median() const
{
  return data->elbow_roll_median;
}

/** Get maximum length of elbow_roll_median value.
 * @return length of elbow_roll_median value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_elbow_roll_median() const
{
  return 1;
}

/** Set elbow_roll_median value.
 * 
      Median in radians of the elbow roll during walking.
    
 * @param new_elbow_roll_median new elbow_roll_median value
 */
void
HumanoidMotionInterface::set_elbow_roll_median(const float new_elbow_roll_median)
{
  data->elbow_roll_median = new_elbow_roll_median;
}

/** Get elbow_roll_amplitude value.
 * 
      Amplitude of the elbow roll movement during walking.
    
 * @return elbow_roll_amplitude value
 */
float
HumanoidMotionInterface::elbow_roll_amplitude() const
{
  return data->elbow_roll_amplitude;
}

/** Get maximum length of elbow_roll_amplitude value.
 * @return length of elbow_roll_amplitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_elbow_roll_amplitude() const
{
  return 1;
}

/** Set elbow_roll_amplitude value.
 * 
      Amplitude of the elbow roll movement during walking.
    
 * @param new_elbow_roll_amplitude new elbow_roll_amplitude value
 */
void
HumanoidMotionInterface::set_elbow_roll_amplitude(const float new_elbow_roll_amplitude)
{
  data->elbow_roll_amplitude = new_elbow_roll_amplitude;
}

/** Get msgid value.
 * 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
    
 * @return msgid value
 */
unsigned int
HumanoidMotionInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
    
 * @param new_msgid new msgid value
 */
void
HumanoidMotionInterface::set_msgid(const unsigned int new_msgid)
{
  data->msgid = new_msgid;
}

/* =========== message create =========== */
Message *
HumanoidMotionInterface::create_message(const char *type) const
{
  if ( strncmp("SetWalkParamsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetWalkParamsMessage();
  } else if ( strncmp("SetWalkArmsParamsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetWalkArmsParamsMessage();
  } else if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("WalkStraightMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkStraightMessage();
  } else if ( strncmp("WalkSidewaysMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkSidewaysMessage();
  } else if ( strncmp("WalkArcMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new WalkArcMessage();
  } else if ( strncmp("TurnMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TurnMessage();
  } else if ( strncmp("KickMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new KickMessage();
  } else if ( strncmp("ParkMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ParkMessage();
  } else if ( strncmp("GetUpMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GetUpMessage();
  } else if ( strncmp("StandupMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StandupMessage();
  } else if ( strncmp("YawPitchHeadMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new YawPitchHeadMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
HumanoidMotionInterface::copy_values(const Interface *other)
{
  const HumanoidMotionInterface *oi = dynamic_cast<const HumanoidMotionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(HumanoidMotionInterface_data_t));
}

/* =========== messages =========== */
/** @class HumanoidMotionInterface::SetWalkParamsMessage <interfaces/HumanoidMotionInterface.h>
 * SetWalkParamsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_max_step_length initial value for max_step_length
 * @param ini_max_step_height initial value for max_step_height
 * @param ini_max_step_side initial value for max_step_side
 * @param ini_max_step_turn initial value for max_step_turn
 * @param ini_zmp_offset_forward initial value for zmp_offset_forward
 * @param ini_zmp_offset_sideward initial value for zmp_offset_sideward
 * @param ini_l_hip_roll_compensation initial value for l_hip_roll_compensation
 * @param ini_r_hip_roll_compensation initial value for r_hip_roll_compensation
 * @param ini_hip_height initial value for hip_height
 * @param ini_torso_sideward_orientation initial value for torso_sideward_orientation
 */
HumanoidMotionInterface::SetWalkParamsMessage::SetWalkParamsMessage(const float ini_max_step_length, const float ini_max_step_height, const float ini_max_step_side, const float ini_max_step_turn, const float ini_zmp_offset_forward, const float ini_zmp_offset_sideward, const float ini_l_hip_roll_compensation, const float ini_r_hip_roll_compensation, const float ini_hip_height, const float ini_torso_sideward_orientation) : Message("SetWalkParamsMessage")
{
  data_size = sizeof(SetWalkParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetWalkParamsMessage_data_t *)data_ptr;
  data->max_step_length = ini_max_step_length;
  data->max_step_height = ini_max_step_height;
  data->max_step_side = ini_max_step_side;
  data->max_step_turn = ini_max_step_turn;
  data->zmp_offset_forward = ini_zmp_offset_forward;
  data->zmp_offset_sideward = ini_zmp_offset_sideward;
  data->l_hip_roll_compensation = ini_l_hip_roll_compensation;
  data->r_hip_roll_compensation = ini_r_hip_roll_compensation;
  data->hip_height = ini_hip_height;
  data->torso_sideward_orientation = ini_torso_sideward_orientation;
}
/** Constructor */
HumanoidMotionInterface::SetWalkParamsMessage::SetWalkParamsMessage() : Message("SetWalkParamsMessage")
{
  data_size = sizeof(SetWalkParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetWalkParamsMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::SetWalkParamsMessage::~SetWalkParamsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::SetWalkParamsMessage::SetWalkParamsMessage(const SetWalkParamsMessage *m) : Message("SetWalkParamsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetWalkParamsMessage_data_t *)data_ptr;
}

/* Methods */
/** Get max_step_length value.
 * 
      Maximum length of a footstep in m.
    
 * @return max_step_length value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::max_step_length() const
{
  return data->max_step_length;
}

/** Get maximum length of max_step_length value.
 * @return length of max_step_length value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_max_step_length() const
{
  return 1;
}

/** Set max_step_length value.
 * 
      Maximum length of a footstep in m.
    
 * @param new_max_step_length new max_step_length value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_max_step_length(const float new_max_step_length)
{
  data->max_step_length = new_max_step_length;
}

/** Get max_step_height value.
 * 
      Maxium height of a footstep cycloid in m.
    
 * @return max_step_height value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::max_step_height() const
{
  return data->max_step_height;
}

/** Get maximum length of max_step_height value.
 * @return length of max_step_height value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_max_step_height() const
{
  return 1;
}

/** Set max_step_height value.
 * 
      Maxium height of a footstep cycloid in m.
    
 * @param new_max_step_height new max_step_height value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_max_step_height(const float new_max_step_height)
{
  data->max_step_height = new_max_step_height;
}

/** Get max_step_side value.
 * 
      Maximum length of side step in m.
    
 * @return max_step_side value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::max_step_side() const
{
  return data->max_step_side;
}

/** Get maximum length of max_step_side value.
 * @return length of max_step_side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_max_step_side() const
{
  return 1;
}

/** Set max_step_side value.
 * 
      Maximum length of side step in m.
    
 * @param new_max_step_side new max_step_side value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_max_step_side(const float new_max_step_side)
{
  data->max_step_side = new_max_step_side;
}

/** Get max_step_turn value.
 * 
      Maximum change around vertical axis on radians per footstep.
    
 * @return max_step_turn value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::max_step_turn() const
{
  return data->max_step_turn;
}

/** Get maximum length of max_step_turn value.
 * @return length of max_step_turn value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_max_step_turn() const
{
  return 1;
}

/** Set max_step_turn value.
 * 
      Maximum change around vertical axis on radians per footstep.
    
 * @param new_max_step_turn new max_step_turn value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_max_step_turn(const float new_max_step_turn)
{
  data->max_step_turn = new_max_step_turn;
}

/** Get zmp_offset_forward value.
 * 
      Zero moment point offset in forward direction in m.
    
 * @return zmp_offset_forward value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::zmp_offset_forward() const
{
  return data->zmp_offset_forward;
}

/** Get maximum length of zmp_offset_forward value.
 * @return length of zmp_offset_forward value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_zmp_offset_forward() const
{
  return 1;
}

/** Set zmp_offset_forward value.
 * 
      Zero moment point offset in forward direction in m.
    
 * @param new_zmp_offset_forward new zmp_offset_forward value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_zmp_offset_forward(const float new_zmp_offset_forward)
{
  data->zmp_offset_forward = new_zmp_offset_forward;
}

/** Get zmp_offset_sideward value.
 * 
      Zero moment point offset in sideward direction in m.
    
 * @return zmp_offset_sideward value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::zmp_offset_sideward() const
{
  return data->zmp_offset_sideward;
}

/** Get maximum length of zmp_offset_sideward value.
 * @return length of zmp_offset_sideward value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_zmp_offset_sideward() const
{
  return 1;
}

/** Set zmp_offset_sideward value.
 * 
      Zero moment point offset in sideward direction in m.
    
 * @param new_zmp_offset_sideward new zmp_offset_sideward value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_zmp_offset_sideward(const float new_zmp_offset_sideward)
{
  data->zmp_offset_sideward = new_zmp_offset_sideward;
}

/** Get l_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return l_hip_roll_compensation value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::l_hip_roll_compensation() const
{
  return data->l_hip_roll_compensation;
}

/** Get maximum length of l_hip_roll_compensation value.
 * @return length of l_hip_roll_compensation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_l_hip_roll_compensation() const
{
  return 1;
}

/** Set l_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_l_hip_roll_compensation new l_hip_roll_compensation value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_l_hip_roll_compensation(const float new_l_hip_roll_compensation)
{
  data->l_hip_roll_compensation = new_l_hip_roll_compensation;
}

/** Get r_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return r_hip_roll_compensation value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::r_hip_roll_compensation() const
{
  return data->r_hip_roll_compensation;
}

/** Get maximum length of r_hip_roll_compensation value.
 * @return length of r_hip_roll_compensation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_r_hip_roll_compensation() const
{
  return 1;
}

/** Set r_hip_roll_compensation value.
 * 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_r_hip_roll_compensation new r_hip_roll_compensation value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_r_hip_roll_compensation(const float new_r_hip_roll_compensation)
{
  data->r_hip_roll_compensation = new_r_hip_roll_compensation;
}

/** Get hip_height value.
 * 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return hip_height value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::hip_height() const
{
  return data->hip_height;
}

/** Get maximum length of hip_height value.
 * @return length of hip_height value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_hip_height() const
{
  return 1;
}

/** Set hip_height value.
 * 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_hip_height new hip_height value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_hip_height(const float new_hip_height)
{
  data->hip_height = new_hip_height;
}

/** Get torso_sideward_orientation value.
 * 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @return torso_sideward_orientation value
 */
float
HumanoidMotionInterface::SetWalkParamsMessage::torso_sideward_orientation() const
{
  return data->torso_sideward_orientation;
}

/** Get maximum length of torso_sideward_orientation value.
 * @return length of torso_sideward_orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkParamsMessage::maxlenof_torso_sideward_orientation() const
{
  return 1;
}

/** Set torso_sideward_orientation value.
 * 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
    
 * @param new_torso_sideward_orientation new torso_sideward_orientation value
 */
void
HumanoidMotionInterface::SetWalkParamsMessage::set_torso_sideward_orientation(const float new_torso_sideward_orientation)
{
  data->torso_sideward_orientation = new_torso_sideward_orientation;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::SetWalkParamsMessage::clone() const
{
  return new HumanoidMotionInterface::SetWalkParamsMessage(this);
}
/** @class HumanoidMotionInterface::SetWalkArmsParamsMessage <interfaces/HumanoidMotionInterface.h>
 * SetWalkArmsParamsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_arms_enabled initial value for arms_enabled
 * @param ini_shoulder_pitch_median initial value for shoulder_pitch_median
 * @param ini_shoulder_pitch_amplitude initial value for shoulder_pitch_amplitude
 * @param ini_elbow_roll_median initial value for elbow_roll_median
 * @param ini_elbow_roll_amplitude initial value for elbow_roll_amplitude
 */
HumanoidMotionInterface::SetWalkArmsParamsMessage::SetWalkArmsParamsMessage(const bool ini_arms_enabled, const float ini_shoulder_pitch_median, const float ini_shoulder_pitch_amplitude, const float ini_elbow_roll_median, const float ini_elbow_roll_amplitude) : Message("SetWalkArmsParamsMessage")
{
  data_size = sizeof(SetWalkArmsParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetWalkArmsParamsMessage_data_t *)data_ptr;
  data->arms_enabled = ini_arms_enabled;
  data->shoulder_pitch_median = ini_shoulder_pitch_median;
  data->shoulder_pitch_amplitude = ini_shoulder_pitch_amplitude;
  data->elbow_roll_median = ini_elbow_roll_median;
  data->elbow_roll_amplitude = ini_elbow_roll_amplitude;
}
/** Constructor */
HumanoidMotionInterface::SetWalkArmsParamsMessage::SetWalkArmsParamsMessage() : Message("SetWalkArmsParamsMessage")
{
  data_size = sizeof(SetWalkArmsParamsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetWalkArmsParamsMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::SetWalkArmsParamsMessage::~SetWalkArmsParamsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::SetWalkArmsParamsMessage::SetWalkArmsParamsMessage(const SetWalkArmsParamsMessage *m) : Message("SetWalkArmsParamsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetWalkArmsParamsMessage_data_t *)data_ptr;
}

/* Methods */
/** Get arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @return arms_enabled value
 */
bool
HumanoidMotionInterface::SetWalkArmsParamsMessage::is_arms_enabled() const
{
  return data->arms_enabled;
}

/** Get maximum length of arms_enabled value.
 * @return length of arms_enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkArmsParamsMessage::maxlenof_arms_enabled() const
{
  return 1;
}

/** Set arms_enabled value.
 * 
      If true the arms are controlled during walking for balancing.
    
 * @param new_arms_enabled new arms_enabled value
 */
void
HumanoidMotionInterface::SetWalkArmsParamsMessage::set_arms_enabled(const bool new_arms_enabled)
{
  data->arms_enabled = new_arms_enabled;
}

/** Get shoulder_pitch_median value.
 * 
      Median in radians of the shoulder pitch during walking.
    
 * @return shoulder_pitch_median value
 */
float
HumanoidMotionInterface::SetWalkArmsParamsMessage::shoulder_pitch_median() const
{
  return data->shoulder_pitch_median;
}

/** Get maximum length of shoulder_pitch_median value.
 * @return length of shoulder_pitch_median value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkArmsParamsMessage::maxlenof_shoulder_pitch_median() const
{
  return 1;
}

/** Set shoulder_pitch_median value.
 * 
      Median in radians of the shoulder pitch during walking.
    
 * @param new_shoulder_pitch_median new shoulder_pitch_median value
 */
void
HumanoidMotionInterface::SetWalkArmsParamsMessage::set_shoulder_pitch_median(const float new_shoulder_pitch_median)
{
  data->shoulder_pitch_median = new_shoulder_pitch_median;
}

/** Get shoulder_pitch_amplitude value.
 * 
      Amplitude of the shoulder pitch movement during walking.
    
 * @return shoulder_pitch_amplitude value
 */
float
HumanoidMotionInterface::SetWalkArmsParamsMessage::shoulder_pitch_amplitude() const
{
  return data->shoulder_pitch_amplitude;
}

/** Get maximum length of shoulder_pitch_amplitude value.
 * @return length of shoulder_pitch_amplitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkArmsParamsMessage::maxlenof_shoulder_pitch_amplitude() const
{
  return 1;
}

/** Set shoulder_pitch_amplitude value.
 * 
      Amplitude of the shoulder pitch movement during walking.
    
 * @param new_shoulder_pitch_amplitude new shoulder_pitch_amplitude value
 */
void
HumanoidMotionInterface::SetWalkArmsParamsMessage::set_shoulder_pitch_amplitude(const float new_shoulder_pitch_amplitude)
{
  data->shoulder_pitch_amplitude = new_shoulder_pitch_amplitude;
}

/** Get elbow_roll_median value.
 * 
      Median in radians of the elbow roll during walking.
    
 * @return elbow_roll_median value
 */
float
HumanoidMotionInterface::SetWalkArmsParamsMessage::elbow_roll_median() const
{
  return data->elbow_roll_median;
}

/** Get maximum length of elbow_roll_median value.
 * @return length of elbow_roll_median value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkArmsParamsMessage::maxlenof_elbow_roll_median() const
{
  return 1;
}

/** Set elbow_roll_median value.
 * 
      Median in radians of the elbow roll during walking.
    
 * @param new_elbow_roll_median new elbow_roll_median value
 */
void
HumanoidMotionInterface::SetWalkArmsParamsMessage::set_elbow_roll_median(const float new_elbow_roll_median)
{
  data->elbow_roll_median = new_elbow_roll_median;
}

/** Get elbow_roll_amplitude value.
 * 
      Amplitude of the elbow roll movement during walking.
    
 * @return elbow_roll_amplitude value
 */
float
HumanoidMotionInterface::SetWalkArmsParamsMessage::elbow_roll_amplitude() const
{
  return data->elbow_roll_amplitude;
}

/** Get maximum length of elbow_roll_amplitude value.
 * @return length of elbow_roll_amplitude value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::SetWalkArmsParamsMessage::maxlenof_elbow_roll_amplitude() const
{
  return 1;
}

/** Set elbow_roll_amplitude value.
 * 
      Amplitude of the elbow roll movement during walking.
    
 * @param new_elbow_roll_amplitude new elbow_roll_amplitude value
 */
void
HumanoidMotionInterface::SetWalkArmsParamsMessage::set_elbow_roll_amplitude(const float new_elbow_roll_amplitude)
{
  data->elbow_roll_amplitude = new_elbow_roll_amplitude;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::SetWalkArmsParamsMessage::clone() const
{
  return new HumanoidMotionInterface::SetWalkArmsParamsMessage(this);
}
/** @class HumanoidMotionInterface::StopMessage <interfaces/HumanoidMotionInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
HumanoidMotionInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/** Destructor */
HumanoidMotionInterface::StopMessage::~StopMessage()
{
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = 0;
  data_ptr  = NULL;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::StopMessage::clone() const
{
  return new HumanoidMotionInterface::StopMessage(this);
}
/** @class HumanoidMotionInterface::WalkStraightMessage <interfaces/HumanoidMotionInterface.h>
 * WalkStraightMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_distance initial value for distance
 * @param ini_num_samples initial value for num_samples
 */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage(const float ini_distance, const unsigned int ini_num_samples) : Message("WalkStraightMessage")
{
  data_size = sizeof(WalkStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
  data->distance = ini_distance;
  data->num_samples = ini_num_samples;
}
/** Constructor */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage() : Message("WalkStraightMessage")
{
  data_size = sizeof(WalkStraightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::WalkStraightMessage::~WalkStraightMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkStraightMessage::WalkStraightMessage(const WalkStraightMessage *m) : Message("WalkStraightMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkStraightMessage_data_t *)data_ptr;
}

/* Methods */
/** Get distance value.
 * Distance in m to walk.
 * @return distance value
 */
float
HumanoidMotionInterface::WalkStraightMessage::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkStraightMessage::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * Distance in m to walk.
 * @param new_distance new distance value
 */
void
HumanoidMotionInterface::WalkStraightMessage::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Get num_samples value.
 * 
      Number of intermediate samples to use for walking.
    
 * @return num_samples value
 */
unsigned int
HumanoidMotionInterface::WalkStraightMessage::num_samples() const
{
  return data->num_samples;
}

/** Get maximum length of num_samples value.
 * @return length of num_samples value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkStraightMessage::maxlenof_num_samples() const
{
  return 1;
}

/** Set num_samples value.
 * 
      Number of intermediate samples to use for walking.
    
 * @param new_num_samples new num_samples value
 */
void
HumanoidMotionInterface::WalkStraightMessage::set_num_samples(const unsigned int new_num_samples)
{
  data->num_samples = new_num_samples;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkStraightMessage::clone() const
{
  return new HumanoidMotionInterface::WalkStraightMessage(this);
}
/** @class HumanoidMotionInterface::WalkSidewaysMessage <interfaces/HumanoidMotionInterface.h>
 * WalkSidewaysMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_distance initial value for distance
 * @param ini_num_samples initial value for num_samples
 */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage(const float ini_distance, const unsigned int ini_num_samples) : Message("WalkSidewaysMessage")
{
  data_size = sizeof(WalkSidewaysMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
  data->distance = ini_distance;
  data->num_samples = ini_num_samples;
}
/** Constructor */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage() : Message("WalkSidewaysMessage")
{
  data_size = sizeof(WalkSidewaysMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::WalkSidewaysMessage::~WalkSidewaysMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkSidewaysMessage::WalkSidewaysMessage(const WalkSidewaysMessage *m) : Message("WalkSidewaysMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkSidewaysMessage_data_t *)data_ptr;
}

/* Methods */
/** Get distance value.
 * Distance in m to walk.
 * @return distance value
 */
float
HumanoidMotionInterface::WalkSidewaysMessage::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkSidewaysMessage::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * Distance in m to walk.
 * @param new_distance new distance value
 */
void
HumanoidMotionInterface::WalkSidewaysMessage::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Get num_samples value.
 * 
      Number of intermediate samples to use for strafing.
    
 * @return num_samples value
 */
unsigned int
HumanoidMotionInterface::WalkSidewaysMessage::num_samples() const
{
  return data->num_samples;
}

/** Get maximum length of num_samples value.
 * @return length of num_samples value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkSidewaysMessage::maxlenof_num_samples() const
{
  return 1;
}

/** Set num_samples value.
 * 
      Number of intermediate samples to use for strafing.
    
 * @param new_num_samples new num_samples value
 */
void
HumanoidMotionInterface::WalkSidewaysMessage::set_num_samples(const unsigned int new_num_samples)
{
  data->num_samples = new_num_samples;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkSidewaysMessage::clone() const
{
  return new HumanoidMotionInterface::WalkSidewaysMessage(this);
}
/** @class HumanoidMotionInterface::WalkArcMessage <interfaces/HumanoidMotionInterface.h>
 * WalkArcMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 * @param ini_radius initial value for radius
 * @param ini_num_samples initial value for num_samples
 */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage(const float ini_angle, const float ini_radius, const unsigned int ini_num_samples) : Message("WalkArcMessage")
{
  data_size = sizeof(WalkArcMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
  data->angle = ini_angle;
  data->radius = ini_radius;
  data->num_samples = ini_num_samples;
}
/** Constructor */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage() : Message("WalkArcMessage")
{
  data_size = sizeof(WalkArcMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::WalkArcMessage::~WalkArcMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::WalkArcMessage::WalkArcMessage(const WalkArcMessage *m) : Message("WalkArcMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (WalkArcMessage_data_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle in radians to turn over the way.
 * @return angle value
 */
float
HumanoidMotionInterface::WalkArcMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkArcMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle in radians to turn over the way.
 * @param new_angle new angle value
 */
void
HumanoidMotionInterface::WalkArcMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get radius value.
 * Radius in m of the circle in m.
 * @return radius value
 */
float
HumanoidMotionInterface::WalkArcMessage::radius() const
{
  return data->radius;
}

/** Get maximum length of radius value.
 * @return length of radius value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkArcMessage::maxlenof_radius() const
{
  return 1;
}

/** Set radius value.
 * Radius in m of the circle in m.
 * @param new_radius new radius value
 */
void
HumanoidMotionInterface::WalkArcMessage::set_radius(const float new_radius)
{
  data->radius = new_radius;
}

/** Get num_samples value.
 * 
      Number of intermediate samples to use for walking.
    
 * @return num_samples value
 */
unsigned int
HumanoidMotionInterface::WalkArcMessage::num_samples() const
{
  return data->num_samples;
}

/** Get maximum length of num_samples value.
 * @return length of num_samples value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::WalkArcMessage::maxlenof_num_samples() const
{
  return 1;
}

/** Set num_samples value.
 * 
      Number of intermediate samples to use for walking.
    
 * @param new_num_samples new num_samples value
 */
void
HumanoidMotionInterface::WalkArcMessage::set_num_samples(const unsigned int new_num_samples)
{
  data->num_samples = new_num_samples;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::WalkArcMessage::clone() const
{
  return new HumanoidMotionInterface::WalkArcMessage(this);
}
/** @class HumanoidMotionInterface::TurnMessage <interfaces/HumanoidMotionInterface.h>
 * TurnMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 * @param ini_num_samples initial value for num_samples
 */
HumanoidMotionInterface::TurnMessage::TurnMessage(const float ini_angle, const unsigned int ini_num_samples) : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
  data->angle = ini_angle;
  data->num_samples = ini_num_samples;
}
/** Constructor */
HumanoidMotionInterface::TurnMessage::TurnMessage() : Message("TurnMessage")
{
  data_size = sizeof(TurnMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::TurnMessage::~TurnMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::TurnMessage::TurnMessage(const TurnMessage *m) : Message("TurnMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TurnMessage_data_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Angle in radians to turn.
 * @return angle value
 */
float
HumanoidMotionInterface::TurnMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::TurnMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Angle in radians to turn.
 * @param new_angle new angle value
 */
void
HumanoidMotionInterface::TurnMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Get num_samples value.
 * 
      Number of intermediate samples to use for turning.
    
 * @return num_samples value
 */
unsigned int
HumanoidMotionInterface::TurnMessage::num_samples() const
{
  return data->num_samples;
}

/** Get maximum length of num_samples value.
 * @return length of num_samples value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::TurnMessage::maxlenof_num_samples() const
{
  return 1;
}

/** Set num_samples value.
 * 
      Number of intermediate samples to use for turning.
    
 * @param new_num_samples new num_samples value
 */
void
HumanoidMotionInterface::TurnMessage::set_num_samples(const unsigned int new_num_samples)
{
  data->num_samples = new_num_samples;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::TurnMessage::clone() const
{
  return new HumanoidMotionInterface::TurnMessage(this);
}
/** @class HumanoidMotionInterface::KickMessage <interfaces/HumanoidMotionInterface.h>
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_leg initial value for leg
 * @param ini_strength initial value for strength
 */
HumanoidMotionInterface::KickMessage::KickMessage(const LegEnum ini_leg, const float ini_strength) : Message("KickMessage")
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data->leg = ini_leg;
  data->strength = ini_strength;
}
/** Constructor */
HumanoidMotionInterface::KickMessage::KickMessage() : Message("KickMessage")
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (KickMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::KickMessage::~KickMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::KickMessage::KickMessage(const KickMessage *m) : Message("KickMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (KickMessage_data_t *)data_ptr;
}

/* Methods */
/** Get leg value.
 * Leg to kick with
 * @return leg value
 */
HumanoidMotionInterface::LegEnum
HumanoidMotionInterface::KickMessage::leg() const
{
  return data->leg;
}

/** Get maximum length of leg value.
 * @return length of leg value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::KickMessage::maxlenof_leg() const
{
  return 1;
}

/** Set leg value.
 * Leg to kick with
 * @param new_leg new leg value
 */
void
HumanoidMotionInterface::KickMessage::set_leg(const LegEnum new_leg)
{
  data->leg = new_leg;
}

/** Get strength value.
 * Kick strength
 * @return strength value
 */
float
HumanoidMotionInterface::KickMessage::strength() const
{
  return data->strength;
}

/** Get maximum length of strength value.
 * @return length of strength value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::KickMessage::maxlenof_strength() const
{
  return 1;
}

/** Set strength value.
 * Kick strength
 * @param new_strength new strength value
 */
void
HumanoidMotionInterface::KickMessage::set_strength(const float new_strength)
{
  data->strength = new_strength;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::KickMessage::clone() const
{
  return new HumanoidMotionInterface::KickMessage(this);
}
/** @class HumanoidMotionInterface::ParkMessage <interfaces/HumanoidMotionInterface.h>
 * ParkMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 */
HumanoidMotionInterface::ParkMessage::ParkMessage(const float ini_time_sec) : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data->time_sec = ini_time_sec;
}
/** Constructor */
HumanoidMotionInterface::ParkMessage::ParkMessage() : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::ParkMessage::~ParkMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::ParkMessage::ParkMessage(const ParkMessage *m) : Message("ParkMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach the position.
 * @return time_sec value
 */
float
HumanoidMotionInterface::ParkMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::ParkMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the position.
 * @param new_time_sec new time_sec value
 */
void
HumanoidMotionInterface::ParkMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::ParkMessage::clone() const
{
  return new HumanoidMotionInterface::ParkMessage(this);
}
/** @class HumanoidMotionInterface::GetUpMessage <interfaces/HumanoidMotionInterface.h>
 * GetUpMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 */
HumanoidMotionInterface::GetUpMessage::GetUpMessage(const float ini_time_sec) : Message("GetUpMessage")
{
  data_size = sizeof(GetUpMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GetUpMessage_data_t *)data_ptr;
  data->time_sec = ini_time_sec;
}
/** Constructor */
HumanoidMotionInterface::GetUpMessage::GetUpMessage() : Message("GetUpMessage")
{
  data_size = sizeof(GetUpMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GetUpMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::GetUpMessage::~GetUpMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::GetUpMessage::GetUpMessage(const GetUpMessage *m) : Message("GetUpMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GetUpMessage_data_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach the position.
 * @return time_sec value
 */
float
HumanoidMotionInterface::GetUpMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::GetUpMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the position.
 * @param new_time_sec new time_sec value
 */
void
HumanoidMotionInterface::GetUpMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::GetUpMessage::clone() const
{
  return new HumanoidMotionInterface::GetUpMessage(this);
}
/** @class HumanoidMotionInterface::StandupMessage <interfaces/HumanoidMotionInterface.h>
 * StandupMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_from_pos initial value for from_pos
 */
HumanoidMotionInterface::StandupMessage::StandupMessage(const StandupEnum ini_from_pos) : Message("StandupMessage")
{
  data_size = sizeof(StandupMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
  data->from_pos = ini_from_pos;
}
/** Constructor */
HumanoidMotionInterface::StandupMessage::StandupMessage() : Message("StandupMessage")
{
  data_size = sizeof(StandupMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::StandupMessage::~StandupMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::StandupMessage::StandupMessage(const StandupMessage *m) : Message("StandupMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StandupMessage_data_t *)data_ptr;
}

/* Methods */
/** Get from_pos value.
 * Position from where to standup.
 * @return from_pos value
 */
HumanoidMotionInterface::StandupEnum
HumanoidMotionInterface::StandupMessage::from_pos() const
{
  return data->from_pos;
}

/** Get maximum length of from_pos value.
 * @return length of from_pos value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::StandupMessage::maxlenof_from_pos() const
{
  return 1;
}

/** Set from_pos value.
 * Position from where to standup.
 * @param new_from_pos new from_pos value
 */
void
HumanoidMotionInterface::StandupMessage::set_from_pos(const StandupEnum new_from_pos)
{
  data->from_pos = new_from_pos;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::StandupMessage::clone() const
{
  return new HumanoidMotionInterface::StandupMessage(this);
}
/** @class HumanoidMotionInterface::YawPitchHeadMessage <interfaces/HumanoidMotionInterface.h>
 * YawPitchHeadMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_yaw initial value for yaw
 * @param ini_pitch initial value for pitch
 * @param ini_time_sec initial value for time_sec
 */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage(const float ini_yaw, const float ini_pitch, const float ini_time_sec) : Message("YawPitchHeadMessage")
{
  data_size = sizeof(YawPitchHeadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
  data->yaw = ini_yaw;
  data->pitch = ini_pitch;
  data->time_sec = ini_time_sec;
}
/** Constructor */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage() : Message("YawPitchHeadMessage")
{
  data_size = sizeof(YawPitchHeadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
}

/** Destructor */
HumanoidMotionInterface::YawPitchHeadMessage::~YawPitchHeadMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
HumanoidMotionInterface::YawPitchHeadMessage::YawPitchHeadMessage(const YawPitchHeadMessage *m) : Message("YawPitchHeadMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (YawPitchHeadMessage_data_t *)data_ptr;
}

/* Methods */
/** Get yaw value.
 * Desired yaw (horizontal orientation).
 * @return yaw value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::yaw() const
{
  return data->yaw;
}

/** Get maximum length of yaw value.
 * @return length of yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_yaw() const
{
  return 1;
}

/** Set yaw value.
 * Desired yaw (horizontal orientation).
 * @param new_yaw new yaw value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_yaw(const float new_yaw)
{
  data->yaw = new_yaw;
}

/** Get pitch value.
 * Desired pitch (vertical orientation).
 * @return pitch value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::pitch() const
{
  return data->pitch;
}

/** Get maximum length of pitch value.
 * @return length of pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_pitch() const
{
  return 1;
}

/** Set pitch value.
 * Desired pitch (vertical orientation).
 * @param new_pitch new pitch value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_pitch(const float new_pitch)
{
  data->pitch = new_pitch;
}

/** Get time_sec value.
 * Time in seconds when to reach the target.
 * @return time_sec value
 */
float
HumanoidMotionInterface::YawPitchHeadMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
HumanoidMotionInterface::YawPitchHeadMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach the target.
 * @param new_time_sec new time_sec value
 */
void
HumanoidMotionInterface::YawPitchHeadMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
HumanoidMotionInterface::YawPitchHeadMessage::clone() const
{
  return new HumanoidMotionInterface::YawPitchHeadMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
HumanoidMotionInterface::message_valid(const Message *message) const
{
  const SetWalkParamsMessage *m0 = dynamic_cast<const SetWalkParamsMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetWalkArmsParamsMessage *m1 = dynamic_cast<const SetWalkArmsParamsMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const StopMessage *m2 = dynamic_cast<const StopMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const WalkStraightMessage *m3 = dynamic_cast<const WalkStraightMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const WalkSidewaysMessage *m4 = dynamic_cast<const WalkSidewaysMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const WalkArcMessage *m5 = dynamic_cast<const WalkArcMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const TurnMessage *m6 = dynamic_cast<const TurnMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const KickMessage *m7 = dynamic_cast<const KickMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const ParkMessage *m8 = dynamic_cast<const ParkMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const GetUpMessage *m9 = dynamic_cast<const GetUpMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const StandupMessage *m10 = dynamic_cast<const StandupMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  const YawPitchHeadMessage *m11 = dynamic_cast<const YawPitchHeadMessage *>(message);
  if ( m11 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(HumanoidMotionInterface)
/// @endcond


} // end namespace fawkes
