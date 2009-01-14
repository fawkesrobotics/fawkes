
/***************************************************************************
 *  HumanoidMotionInterface.h - Fawkes BlackBoard Interface - HumanoidMotionInterface
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

#ifndef __INTERFACES_HUMANOIDMOTIONINTERFACE_H_
#define __INTERFACES_HUMANOIDMOTIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

class HumanoidMotionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(HumanoidMotionInterface)
 /// @endcond
 public:
  /* constants */

  /** Type to determinate leg side. */
  typedef enum {
    LEG_LEFT /**< Left leg. */,
    LEG_RIGHT /**< Right leg. */
  } LegEnum;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int msgid; /**< 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
     */
    float max_step_length; /**< 
      Maximum length of a footstep in m.
     */
    float max_step_height; /**< 
      Maxium height of a footstep cycloid in m.
     */
    float max_step_side; /**< 
      Maximum length of side step in m.
     */
    float max_step_turn; /**< 
      Maximum change around vertical axis on radians per footstep.
     */
    float zmp_offset_forward; /**< 
      Zero moment point offset in forward direction in m.
     */
    float zmp_offset_sideward; /**< 
      Zero moment point offset in sideward direction in m.
     */
    float l_hip_roll_compensation; /**< 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
    float r_hip_roll_compensation; /**< 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
    float hip_height; /**< 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
    float torso_sideward_orientation; /**< 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
    float shoulder_pitch_median; /**< 
      Median in radians of the shoulder pitch during walking.
     */
    float shoulder_pitch_amplitude; /**< 
      Amplitude of the shoulder pitch movement during walking.
     */
    float elbow_roll_median; /**< 
      Median in radians of the elbow roll during walking.
     */
    float elbow_roll_amplitude; /**< 
      Amplitude of the elbow roll movement during walking.
     */
    bool moving; /**< True if the robot is currently moving. */
    bool arms_enabled; /**< 
      If true the arms are controlled during walking for balancing.
     */
  } HumanoidMotionInterface_data_t;

  HumanoidMotionInterface_data_t *data;

 public:
  /* messages */
  class SetWalkParamsMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float max_step_length; /**< 
      Maximum length of a footstep in m.
     */
      float max_step_height; /**< 
      Maxium height of a footstep cycloid in m.
     */
      float max_step_side; /**< 
      Maximum length of side step in m.
     */
      float max_step_turn; /**< 
      Maximum change around vertical axis on radians per footstep.
     */
      float zmp_offset_forward; /**< 
      Zero moment point offset in forward direction in m.
     */
      float zmp_offset_sideward; /**< 
      Zero moment point offset in sideward direction in m.
     */
      float l_hip_roll_compensation; /**< 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
      float r_hip_roll_compensation; /**< 
      Amplitude in degrees of backlash compensation for left hip roll.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
      float hip_height; /**< 
      Height of hip during walk process.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
      float torso_sideward_orientation; /**< 
      Torso orientation in degrees in sideward direction during walking.
      This is fitted to the Nao and is possibly not applicable to other robots.
     */
    } SetWalkParamsMessage_data_t;

    SetWalkParamsMessage_data_t *data;

   public:
    SetWalkParamsMessage(const float ini_max_step_length, const float ini_max_step_height, const float ini_max_step_side, const float ini_max_step_turn, const float ini_zmp_offset_forward, const float ini_zmp_offset_sideward, const float ini_l_hip_roll_compensation, const float ini_r_hip_roll_compensation, const float ini_hip_height, const float ini_torso_sideward_orientation);
    SetWalkParamsMessage();
    ~SetWalkParamsMessage();

    SetWalkParamsMessage(const SetWalkParamsMessage *m);
    /* Methods */
    float max_step_length() const;
    void set_max_step_length(const float new_max_step_length);
    size_t maxlenof_max_step_length() const;
    float max_step_height() const;
    void set_max_step_height(const float new_max_step_height);
    size_t maxlenof_max_step_height() const;
    float max_step_side() const;
    void set_max_step_side(const float new_max_step_side);
    size_t maxlenof_max_step_side() const;
    float max_step_turn() const;
    void set_max_step_turn(const float new_max_step_turn);
    size_t maxlenof_max_step_turn() const;
    float zmp_offset_forward() const;
    void set_zmp_offset_forward(const float new_zmp_offset_forward);
    size_t maxlenof_zmp_offset_forward() const;
    float zmp_offset_sideward() const;
    void set_zmp_offset_sideward(const float new_zmp_offset_sideward);
    size_t maxlenof_zmp_offset_sideward() const;
    float l_hip_roll_compensation() const;
    void set_l_hip_roll_compensation(const float new_l_hip_roll_compensation);
    size_t maxlenof_l_hip_roll_compensation() const;
    float r_hip_roll_compensation() const;
    void set_r_hip_roll_compensation(const float new_r_hip_roll_compensation);
    size_t maxlenof_r_hip_roll_compensation() const;
    float hip_height() const;
    void set_hip_height(const float new_hip_height);
    size_t maxlenof_hip_height() const;
    float torso_sideward_orientation() const;
    void set_torso_sideward_orientation(const float new_torso_sideward_orientation);
    size_t maxlenof_torso_sideward_orientation() const;
    virtual Message * clone() const;
  };

  class SetWalkArmsParamsMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float shoulder_pitch_median; /**< 
      Median in radians of the shoulder pitch during walking.
     */
      float shoulder_pitch_amplitude; /**< 
      Amplitude of the shoulder pitch movement during walking.
     */
      float elbow_roll_median; /**< 
      Median in radians of the elbow roll during walking.
     */
      float elbow_roll_amplitude; /**< 
      Amplitude of the elbow roll movement during walking.
     */
      bool arms_enabled; /**< 
      If true the arms are controlled during walking for balancing.
     */
    } SetWalkArmsParamsMessage_data_t;

    SetWalkArmsParamsMessage_data_t *data;

   public:
    SetWalkArmsParamsMessage(const bool ini_arms_enabled, const float ini_shoulder_pitch_median, const float ini_shoulder_pitch_amplitude, const float ini_elbow_roll_median, const float ini_elbow_roll_amplitude);
    SetWalkArmsParamsMessage();
    ~SetWalkArmsParamsMessage();

    SetWalkArmsParamsMessage(const SetWalkArmsParamsMessage *m);
    /* Methods */
    bool is_arms_enabled() const;
    void set_arms_enabled(const bool new_arms_enabled);
    size_t maxlenof_arms_enabled() const;
    float shoulder_pitch_median() const;
    void set_shoulder_pitch_median(const float new_shoulder_pitch_median);
    size_t maxlenof_shoulder_pitch_median() const;
    float shoulder_pitch_amplitude() const;
    void set_shoulder_pitch_amplitude(const float new_shoulder_pitch_amplitude);
    size_t maxlenof_shoulder_pitch_amplitude() const;
    float elbow_roll_median() const;
    void set_elbow_roll_median(const float new_elbow_roll_median);
    size_t maxlenof_elbow_roll_median() const;
    float elbow_roll_amplitude() const;
    void set_elbow_roll_amplitude(const float new_elbow_roll_amplitude);
    size_t maxlenof_elbow_roll_amplitude() const;
    virtual Message * clone() const;
  };

  class StopMessage : public Message
  {
   public:
    StopMessage();
    ~StopMessage();

    StopMessage(const StopMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class WalkStraightMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int num_samples; /**< 
      Number of intermediate samples to use for walking.
     */
      float distance; /**< Distance in m to walk. */
    } WalkStraightMessage_data_t;

    WalkStraightMessage_data_t *data;

   public:
    WalkStraightMessage(const float ini_distance, const unsigned int ini_num_samples);
    WalkStraightMessage();
    ~WalkStraightMessage();

    WalkStraightMessage(const WalkStraightMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    unsigned int num_samples() const;
    void set_num_samples(const unsigned int new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class WalkSidewaysMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int num_samples; /**< 
      Number of intermediate samples to use for strafing.
     */
      float distance; /**< Distance in m to walk. */
    } WalkSidewaysMessage_data_t;

    WalkSidewaysMessage_data_t *data;

   public:
    WalkSidewaysMessage(const float ini_distance, const unsigned int ini_num_samples);
    WalkSidewaysMessage();
    ~WalkSidewaysMessage();

    WalkSidewaysMessage(const WalkSidewaysMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    unsigned int num_samples() const;
    void set_num_samples(const unsigned int new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class WalkArcMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int num_samples; /**< 
      Number of intermediate samples to use for walking.
     */
      float angle; /**< Angle in radians to turn over the way. */
      float radius; /**< Radius in m of the circle in m. */
    } WalkArcMessage_data_t;

    WalkArcMessage_data_t *data;

   public:
    WalkArcMessage(const float ini_angle, const float ini_radius, const unsigned int ini_num_samples);
    WalkArcMessage();
    ~WalkArcMessage();

    WalkArcMessage(const WalkArcMessage *m);
    /* Methods */
    float angle() const;
    void set_angle(const float new_angle);
    size_t maxlenof_angle() const;
    float radius() const;
    void set_radius(const float new_radius);
    size_t maxlenof_radius() const;
    unsigned int num_samples() const;
    void set_num_samples(const unsigned int new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class TurnMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int num_samples; /**< 
      Number of intermediate samples to use for turning.
     */
      float angle; /**< Angle in radians to turn. */
    } TurnMessage_data_t;

    TurnMessage_data_t *data;

   public:
    TurnMessage(const float ini_angle, const unsigned int ini_num_samples);
    TurnMessage();
    ~TurnMessage();

    TurnMessage(const TurnMessage *m);
    /* Methods */
    float angle() const;
    void set_angle(const float new_angle);
    size_t maxlenof_angle() const;
    unsigned int num_samples() const;
    void set_num_samples(const unsigned int new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class KickMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float strength; /**< Kick strength */
      LegEnum leg; /**< Leg to kick with */
    } KickMessage_data_t;

    KickMessage_data_t *data;

   public:
    KickMessage(const LegEnum ini_leg, const float ini_strength);
    KickMessage();
    ~KickMessage();

    KickMessage(const KickMessage *m);
    /* Methods */
    LegEnum leg() const;
    void set_leg(const LegEnum new_leg);
    size_t maxlenof_leg() const;
    float strength() const;
    void set_strength(const float new_strength);
    size_t maxlenof_strength() const;
    virtual Message * clone() const;
  };

  class ParkMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float time_sec; /**< Time in seconds when to reach the position. */
    } ParkMessage_data_t;

    ParkMessage_data_t *data;

   public:
    ParkMessage(const float ini_time_sec);
    ParkMessage();
    ~ParkMessage();

    ParkMessage(const ParkMessage *m);
    /* Methods */
    float time_sec() const;
    void set_time_sec(const float new_time_sec);
    size_t maxlenof_time_sec() const;
    virtual Message * clone() const;
  };

  class GetUpMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float time_sec; /**< Time in seconds when to reach the position. */
    } GetUpMessage_data_t;

    GetUpMessage_data_t *data;

   public:
    GetUpMessage(const float ini_time_sec);
    GetUpMessage();
    ~GetUpMessage();

    GetUpMessage(const GetUpMessage *m);
    /* Methods */
    float time_sec() const;
    void set_time_sec(const float new_time_sec);
    size_t maxlenof_time_sec() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  HumanoidMotionInterface();
  ~HumanoidMotionInterface();

 public:
  /* Methods */
  bool is_moving() const;
  void set_moving(const bool new_moving);
  size_t maxlenof_moving() const;
  float max_step_length() const;
  void set_max_step_length(const float new_max_step_length);
  size_t maxlenof_max_step_length() const;
  float max_step_height() const;
  void set_max_step_height(const float new_max_step_height);
  size_t maxlenof_max_step_height() const;
  float max_step_side() const;
  void set_max_step_side(const float new_max_step_side);
  size_t maxlenof_max_step_side() const;
  float max_step_turn() const;
  void set_max_step_turn(const float new_max_step_turn);
  size_t maxlenof_max_step_turn() const;
  float zmp_offset_forward() const;
  void set_zmp_offset_forward(const float new_zmp_offset_forward);
  size_t maxlenof_zmp_offset_forward() const;
  float zmp_offset_sideward() const;
  void set_zmp_offset_sideward(const float new_zmp_offset_sideward);
  size_t maxlenof_zmp_offset_sideward() const;
  float l_hip_roll_compensation() const;
  void set_l_hip_roll_compensation(const float new_l_hip_roll_compensation);
  size_t maxlenof_l_hip_roll_compensation() const;
  float r_hip_roll_compensation() const;
  void set_r_hip_roll_compensation(const float new_r_hip_roll_compensation);
  size_t maxlenof_r_hip_roll_compensation() const;
  float hip_height() const;
  void set_hip_height(const float new_hip_height);
  size_t maxlenof_hip_height() const;
  float torso_sideward_orientation() const;
  void set_torso_sideward_orientation(const float new_torso_sideward_orientation);
  size_t maxlenof_torso_sideward_orientation() const;
  bool is_arms_enabled() const;
  void set_arms_enabled(const bool new_arms_enabled);
  size_t maxlenof_arms_enabled() const;
  float shoulder_pitch_median() const;
  void set_shoulder_pitch_median(const float new_shoulder_pitch_median);
  size_t maxlenof_shoulder_pitch_median() const;
  float shoulder_pitch_amplitude() const;
  void set_shoulder_pitch_amplitude(const float new_shoulder_pitch_amplitude);
  size_t maxlenof_shoulder_pitch_amplitude() const;
  float elbow_roll_median() const;
  void set_elbow_roll_median(const float new_elbow_roll_median);
  size_t maxlenof_elbow_roll_median() const;
  float elbow_roll_amplitude() const;
  void set_elbow_roll_amplitude(const float new_elbow_roll_amplitude);
  size_t maxlenof_elbow_roll_amplitude() const;
  unsigned int msgid() const;
  void set_msgid(const unsigned int new_msgid);
  size_t maxlenof_msgid() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
