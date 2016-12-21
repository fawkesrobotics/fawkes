
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

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    bool moving; /**< True if the robot is moving. */
    bool arms_enabled; /**< 
      If true the arms are controlled during walking for balancing.
     */
    uint32_t msgid; /**< 
      The ID of the message that is currently being
      processed, or 0 if no message is being processed.
     */
  } HumanoidMotionInterface_data_t;

  HumanoidMotionInterface_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
 public:
  /* messages */
  class StopMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } StopMessage_data_t;

    StopMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
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
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float distance; /**< Distance in m to walk. */
    } WalkStraightMessage_data_t;

    WalkStraightMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    WalkStraightMessage(const float ini_distance);
    WalkStraightMessage();
    ~WalkStraightMessage();

    WalkStraightMessage(const WalkStraightMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    virtual Message * clone() const;
  };

  class WalkSidewaysMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float distance; /**< Distance in m to walk. */
    } WalkSidewaysMessage_data_t;

    WalkSidewaysMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    WalkSidewaysMessage(const float ini_distance);
    WalkSidewaysMessage();
    ~WalkSidewaysMessage();

    WalkSidewaysMessage(const WalkSidewaysMessage *m);
    /* Methods */
    float distance() const;
    void set_distance(const float new_distance);
    size_t maxlenof_distance() const;
    virtual Message * clone() const;
  };

  class WalkArcMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float angle; /**< Angle in radians to turn over the way. */
      float radius; /**< Radius in m of the circle in m. */
    } WalkArcMessage_data_t;

    WalkArcMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    WalkArcMessage(const float ini_angle, const float ini_radius);
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
    virtual Message * clone() const;
  };

  class WalkVelocityMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float x; /**< 
      Fraction of MaxStepX. Use negative for backwards. [-1.0 to 1.0].
     */
      float y; /**< 
      Fraction of MaxStepY. Use negative for right. [-1.0 to 1.0].
     */
      float theta; /**< 
      Fraction of MaxStepTheta. Use negative for clockwise [-1.0 to 1.0].
     */
      float speed; /**< 
      Fraction of MaxStepFrequency [0.0 to 1.0].
     */
    } WalkVelocityMessage_data_t;

    WalkVelocityMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    WalkVelocityMessage(const float ini_x, const float ini_y, const float ini_theta, const float ini_speed);
    WalkVelocityMessage();
    ~WalkVelocityMessage();

    WalkVelocityMessage(const WalkVelocityMessage *m);
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
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float angle; /**< Angle in radians to turn. */
    } TurnMessage_data_t;

    TurnMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    TurnMessage(const float ini_angle);
    TurnMessage();
    ~TurnMessage();

    TurnMessage(const TurnMessage *m);
    /* Methods */
    float angle() const;
    void set_angle(const float new_angle);
    size_t maxlenof_angle() const;
    virtual Message * clone() const;
  };

  class KickMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t leg; /**< Leg to kick with */
      float strength; /**< Kick strength */
    } KickMessage_data_t;

    KickMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
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
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ParkMessage_data_t;

    ParkMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    ParkMessage();
    ~ParkMessage();

    ParkMessage(const ParkMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class GetUpMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } GetUpMessage_data_t;

    GetUpMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    GetUpMessage();
    ~GetUpMessage();

    GetUpMessage(const GetUpMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class StandupMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      int32_t from_pos; /**< Position from where to standup. */
    } StandupMessage_data_t;

    StandupMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
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

  class MoveHeadMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      float yaw; /**< Desired yaw (horizontal orientation). */
      float pitch; /**< Desired pitch (vertical orientation). */
      float speed; /**< Maximum speed in [0.0..1.0]. */
    } MoveHeadMessage_data_t;

    MoveHeadMessage_data_t *data;

  interface_enum_map_t enum_map_LegEnum;
  interface_enum_map_t enum_map_StandupEnum;
   public:
    MoveHeadMessage(const float ini_yaw, const float ini_pitch, const float ini_speed);
    MoveHeadMessage();
    ~MoveHeadMessage();

    MoveHeadMessage(const MoveHeadMessage *m);
    /* Methods */
    float yaw() const;
    void set_yaw(const float new_yaw);
    size_t maxlenof_yaw() const;
    float pitch() const;
    void set_pitch(const float new_pitch);
    size_t maxlenof_pitch() const;
    float speed() const;
    void set_speed(const float new_speed);
    size_t maxlenof_speed() const;
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
  bool is_arms_enabled() const;
  void set_arms_enabled(const bool new_arms_enabled);
  size_t maxlenof_arms_enabled() const;
  uint32_t msgid() const;
  void set_msgid(const uint32_t new_msgid);
  size_t maxlenof_msgid() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
