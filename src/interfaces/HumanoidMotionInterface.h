
/***************************************************************************
 *  HumanoidMotionInterface.h - Fawkes BlackBoard Interface - HumanoidMotionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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
#include <interface/field_iterator.h>

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
  const char * tostring_LegEnum(LegEnum value) const;

  /** From which position to standup. */
  typedef enum {
    STANDUP_DETECT /**< Detect via accelerometer. */,
    STANDUP_BACK /**< Standup from lying on the back. */,
    STANDUP_FRONT /**< Standup from lying on the tummy. */
  } StandupEnum;
  const char * tostring_StandupEnum(StandupEnum value) const;

  /** The motion patterns that need specific stiffness settings */
  typedef enum {
    WALK /**< The walk pattern */,
    KICK /**< The kick pattern */
  } StiffnessMotionPatternEnum;
  const char * tostring_StiffnessMotionPatternEnum(StiffnessMotionPatternEnum value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    bool walking; /**< True if the robot is currently moving. */
    LegEnum supporting_leg; /**< Marks the supporting leg */
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
    bool arms_enabled; /**< 
      If true the arms are controlled during walking for balancing.
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
    uint32_t msgid; /**< 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
     */
  } HumanoidMotionInterface_data_t;
#pragma pack(pop)

  HumanoidMotionInterface_data_t *data;

 public:
  /* messages */
  class SetWalkParamsMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
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
#pragma pack(pop)

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
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      bool arms_enabled; /**< 
      If true the arms are controlled during walking for balancing.
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
    } SetWalkArmsParamsMessage_data_t;
#pragma pack(pop)

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
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } StopMessage_data_t;
#pragma pack(pop)

    StopMessage_data_t *data;

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
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float distance; /**< Distance in m to walk. */
      uint32_t num_samples; /**< 
      Number of intermediate samples to use for walking.
     */
    } WalkStraightMessage_data_t;
#pragma pack(pop)

    WalkStraightMessage_data_t *data;

   public:
    WalkStraightMessage(const float ini_distance, const uint32_t ini_num_samples);
    WalkStraightMessage();
    ~WalkStraightMessage();

    WalkStraightMessage(const WalkStraightMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    uint32_t num_samples() const;
    void set_num_samples(const uint32_t new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class WalkSidewaysMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float distance; /**< Distance in m to walk. */
      uint32_t num_samples; /**< 
      Number of intermediate samples to use for strafing.
     */
    } WalkSidewaysMessage_data_t;
#pragma pack(pop)

    WalkSidewaysMessage_data_t *data;

   public:
    WalkSidewaysMessage(const float ini_distance, const uint32_t ini_num_samples);
    WalkSidewaysMessage();
    ~WalkSidewaysMessage();

    WalkSidewaysMessage(const WalkSidewaysMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    uint32_t num_samples() const;
    void set_num_samples(const uint32_t new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class WalkArcMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float angle; /**< Angle in radians to turn over the way. */
      float radius; /**< Radius in m of the circle in m. */
      uint32_t num_samples; /**< 
      Number of intermediate samples to use for walking.
     */
    } WalkArcMessage_data_t;
#pragma pack(pop)

    WalkArcMessage_data_t *data;

   public:
    WalkArcMessage(const float ini_angle, const float ini_radius, const uint32_t ini_num_samples);
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
    uint32_t num_samples() const;
    void set_num_samples(const uint32_t new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class WalkMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< Fraction of MaxStepX. Use negative for backwards. [-1.0 to 1.0] */
      float y; /**< Fraction of MaxStepY. Use negative for right. [-1.0 to 1.0] */
      float theta; /**< Fraction of MaxStepTheta. Use negative for clockwise [-1.0 to 1.0] */
      float speed; /**< Fraction of MaxStepFrequency [0.0 to 1.0] */
    } WalkMessage_data_t;
#pragma pack(pop)

    WalkMessage_data_t *data;

   public:
    WalkMessage(const float ini_x, const float ini_y, const float ini_theta, const float ini_speed);
    WalkMessage();
    ~WalkMessage();

    WalkMessage(const WalkMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float theta() const;
    void set_theta(const float new_theta);
    size_t maxlenof_theta() const;
    float speed() const;
    void set_speed(const float new_speed);
    size_t maxlenof_speed() const;
    virtual Message * clone() const;
  };

  class TurnMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float angle; /**< Angle in radians to turn. */
      uint32_t num_samples; /**< 
      Number of intermediate samples to use for turning.
     */
    } TurnMessage_data_t;
#pragma pack(pop)

    TurnMessage_data_t *data;

   public:
    TurnMessage(const float ini_angle, const uint32_t ini_num_samples);
    TurnMessage();
    ~TurnMessage();

    TurnMessage(const TurnMessage *m);
    /* Methods */
    float angle() const;
    void set_angle(const float new_angle);
    size_t maxlenof_angle() const;
    uint32_t num_samples() const;
    void set_num_samples(const uint32_t new_num_samples);
    size_t maxlenof_num_samples() const;
    virtual Message * clone() const;
  };

  class KickMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      LegEnum leg; /**< Leg to kick with */
      float strength; /**< Kick strength */
    } KickMessage_data_t;
#pragma pack(pop)

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
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float time_sec; /**< Time in seconds when to reach the position. */
    } ParkMessage_data_t;
#pragma pack(pop)

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
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float time_sec; /**< Time in seconds when to reach the position. */
    } GetUpMessage_data_t;
#pragma pack(pop)

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

  class StandupMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      StandupEnum from_pos; /**< Position from where to standup. */
    } StandupMessage_data_t;
#pragma pack(pop)

    StandupMessage_data_t *data;

   public:
    StandupMessage(const StandupEnum ini_from_pos);
    StandupMessage();
    ~StandupMessage();

    StandupMessage(const StandupMessage *m);
    /* Methods */
    StandupEnum from_pos() const;
    void set_from_pos(const StandupEnum new_from_pos);
    size_t maxlenof_from_pos() const;
    virtual Message * clone() const;
  };

  class YawPitchHeadMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float yaw; /**< Desired yaw (horizontal orientation). */
      float pitch; /**< Desired pitch (vertical orientation). */
      float time_sec; /**< Time in seconds when to reach the target. */
    } YawPitchHeadMessage_data_t;
#pragma pack(pop)

    YawPitchHeadMessage_data_t *data;

   public:
    YawPitchHeadMessage(const float ini_yaw, const float ini_pitch, const float ini_time_sec);
    YawPitchHeadMessage();
    ~YawPitchHeadMessage();

    YawPitchHeadMessage(const YawPitchHeadMessage *m);
    /* Methods */
    float yaw() const;
    void set_yaw(const float new_yaw);
    size_t maxlenof_yaw() const;
    float pitch() const;
    void set_pitch(const float new_pitch);
    size_t maxlenof_pitch() const;
    float time_sec() const;
    void set_time_sec(const float new_time_sec);
    size_t maxlenof_time_sec() const;
    virtual Message * clone() const;
  };

  class SetStiffnessParamsMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      StiffnessMotionPatternEnum motion_pattern; /**< the motion pattern to update */
      float head_yaw; /**< head_yaw */
      float head_pitch; /**< head_pitch */
      float l_shoulder_pitch; /**< l_shoulder_pitch */
      float l_shoulder_roll; /**< l_shoulder_roll */
      float l_elbow_yaw; /**< l_elbow_yaw */
      float l_elbow_roll; /**< l_elbow_roll */
      float l_hip_yaw_pitch; /**< l_hip_yaw_pitch */
      float l_hip_roll; /**< l_hip_roll */
      float l_hip_pitch; /**< l_hip_pitch */
      float l_knee_pitch; /**< l_knee_pitch */
      float l_ankle_pitch; /**< l_ankle_pitch */
      float l_ankle_roll; /**< l_ankle_roll */
      float r_hip_yaw_pitch; /**< r_hip_yaw_pitch */
      float r_hip_roll; /**< r_hip_roll */
      float r_hip_pitch; /**< r_hip_pitch */
      float r_knee_pitch; /**< r_knee_pitch */
      float r_ankle_pitch; /**< r_ankle_pitch */
      float r_ankle_roll; /**< r_ankle_roll */
      float r_shoulder_pitch; /**< r_shoulder_pitch */
      float r_shoulder_roll; /**< r_shoulder_roll */
      float r_elbow_yaw; /**< r_elbow_yaw */
      float r_elbow_roll; /**< r_elbow_roll */
    } SetStiffnessParamsMessage_data_t;
#pragma pack(pop)

    SetStiffnessParamsMessage_data_t *data;

   public:
    SetStiffnessParamsMessage(const StiffnessMotionPatternEnum ini_motion_pattern, const float ini_head_yaw, const float ini_head_pitch, const float ini_l_shoulder_pitch, const float ini_l_shoulder_roll, const float ini_l_elbow_yaw, const float ini_l_elbow_roll, const float ini_l_hip_yaw_pitch, const float ini_l_hip_roll, const float ini_l_hip_pitch, const float ini_l_knee_pitch, const float ini_l_ankle_pitch, const float ini_l_ankle_roll, const float ini_r_hip_yaw_pitch, const float ini_r_hip_roll, const float ini_r_hip_pitch, const float ini_r_knee_pitch, const float ini_r_ankle_pitch, const float ini_r_ankle_roll, const float ini_r_shoulder_pitch, const float ini_r_shoulder_roll, const float ini_r_elbow_yaw, const float ini_r_elbow_roll);
    SetStiffnessParamsMessage();
    ~SetStiffnessParamsMessage();

    SetStiffnessParamsMessage(const SetStiffnessParamsMessage *m);
    /* Methods */
    StiffnessMotionPatternEnum motion_pattern() const;
    void set_motion_pattern(const StiffnessMotionPatternEnum new_motion_pattern);
    size_t maxlenof_motion_pattern() const;
    float head_yaw() const;
    void set_head_yaw(const float new_head_yaw);
    size_t maxlenof_head_yaw() const;
    float head_pitch() const;
    void set_head_pitch(const float new_head_pitch);
    size_t maxlenof_head_pitch() const;
    float l_shoulder_pitch() const;
    void set_l_shoulder_pitch(const float new_l_shoulder_pitch);
    size_t maxlenof_l_shoulder_pitch() const;
    float l_shoulder_roll() const;
    void set_l_shoulder_roll(const float new_l_shoulder_roll);
    size_t maxlenof_l_shoulder_roll() const;
    float l_elbow_yaw() const;
    void set_l_elbow_yaw(const float new_l_elbow_yaw);
    size_t maxlenof_l_elbow_yaw() const;
    float l_elbow_roll() const;
    void set_l_elbow_roll(const float new_l_elbow_roll);
    size_t maxlenof_l_elbow_roll() const;
    float l_hip_yaw_pitch() const;
    void set_l_hip_yaw_pitch(const float new_l_hip_yaw_pitch);
    size_t maxlenof_l_hip_yaw_pitch() const;
    float l_hip_roll() const;
    void set_l_hip_roll(const float new_l_hip_roll);
    size_t maxlenof_l_hip_roll() const;
    float l_hip_pitch() const;
    void set_l_hip_pitch(const float new_l_hip_pitch);
    size_t maxlenof_l_hip_pitch() const;
    float l_knee_pitch() const;
    void set_l_knee_pitch(const float new_l_knee_pitch);
    size_t maxlenof_l_knee_pitch() const;
    float l_ankle_pitch() const;
    void set_l_ankle_pitch(const float new_l_ankle_pitch);
    size_t maxlenof_l_ankle_pitch() const;
    float l_ankle_roll() const;
    void set_l_ankle_roll(const float new_l_ankle_roll);
    size_t maxlenof_l_ankle_roll() const;
    float r_hip_yaw_pitch() const;
    void set_r_hip_yaw_pitch(const float new_r_hip_yaw_pitch);
    size_t maxlenof_r_hip_yaw_pitch() const;
    float r_hip_roll() const;
    void set_r_hip_roll(const float new_r_hip_roll);
    size_t maxlenof_r_hip_roll() const;
    float r_hip_pitch() const;
    void set_r_hip_pitch(const float new_r_hip_pitch);
    size_t maxlenof_r_hip_pitch() const;
    float r_knee_pitch() const;
    void set_r_knee_pitch(const float new_r_knee_pitch);
    size_t maxlenof_r_knee_pitch() const;
    float r_ankle_pitch() const;
    void set_r_ankle_pitch(const float new_r_ankle_pitch);
    size_t maxlenof_r_ankle_pitch() const;
    float r_ankle_roll() const;
    void set_r_ankle_roll(const float new_r_ankle_roll);
    size_t maxlenof_r_ankle_roll() const;
    float r_shoulder_pitch() const;
    void set_r_shoulder_pitch(const float new_r_shoulder_pitch);
    size_t maxlenof_r_shoulder_pitch() const;
    float r_shoulder_roll() const;
    void set_r_shoulder_roll(const float new_r_shoulder_roll);
    size_t maxlenof_r_shoulder_roll() const;
    float r_elbow_yaw() const;
    void set_r_elbow_yaw(const float new_r_elbow_yaw);
    size_t maxlenof_r_elbow_yaw() const;
    float r_elbow_roll() const;
    void set_r_elbow_roll(const float new_r_elbow_roll);
    size_t maxlenof_r_elbow_roll() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  HumanoidMotionInterface();
  ~HumanoidMotionInterface();

 public:
  /* Methods */
  bool is_walking() const;
  void set_walking(const bool new_walking);
  size_t maxlenof_walking() const;
  LegEnum supporting_leg() const;
  void set_supporting_leg(const LegEnum new_supporting_leg);
  size_t maxlenof_supporting_leg() const;
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
  uint32_t msgid() const;
  void set_msgid(const uint32_t new_msgid);
  size_t maxlenof_msgid() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
