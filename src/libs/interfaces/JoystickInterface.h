
/***************************************************************************
 *  JoystickInterface.h - Fawkes BlackBoard Interface - JoystickInterface
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

#ifndef __INTERFACES_JOYSTICKINTERFACE_H_
#define __INTERFACES_JOYSTICKINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class JoystickInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(JoystickInterface)
 /// @endcond
 public:
  /* constants */
  static const uint32_t BUTTON_1;
  static const uint32_t BUTTON_2;
  static const uint32_t BUTTON_3;
  static const uint32_t BUTTON_4;
  static const uint32_t BUTTON_5;
  static const uint32_t BUTTON_6;
  static const uint32_t BUTTON_7;
  static const uint32_t BUTTON_8;
  static const uint32_t BUTTON_9;
  static const uint32_t BUTTON_10;
  static const uint32_t BUTTON_11;
  static const uint32_t BUTTON_12;
  static const uint32_t BUTTON_13;
  static const uint32_t BUTTON_14;
  static const uint32_t BUTTON_15;
  static const uint32_t BUTTON_16;
  static const uint32_t BUTTON_17;
  static const uint32_t BUTTON_18;
  static const uint32_t BUTTON_19;
  static const uint32_t BUTTON_20;
  static const uint32_t BUTTON_21;
  static const uint32_t BUTTON_22;
  static const uint32_t BUTTON_23;
  static const uint32_t BUTTON_24;
  static const uint32_t BUTTON_25;
  static const uint32_t BUTTON_26;
  static const uint32_t BUTTON_27;
  static const uint32_t BUTTON_28;
  static const uint32_t BUTTON_29;
  static const uint32_t BUTTON_30;
  static const uint32_t BUTTON_31;
  static const uint32_t BUTTON_32;
  static const uint8_t JFF_RUMBLE;
  static const uint8_t JFF_PERIODIC;
  static const uint8_t JFF_RAMP;
  static const uint8_t JFF_SPRING;
  static const uint8_t JFF_FRICTION;
  static const uint8_t JFF_DAMPER;
  static const uint8_t JFF_INERTIA;
  static const uint8_t JFF_CONSTANT;

  /** Effect direction. */
  typedef enum {
    DIRECTION_DOWN = 0 /**< Down. */,
    DIRECTION_LEFT = 16384 /**< Left. */,
    DIRECTION_UP = 32768 /**< Up. */,
    DIRECTION_RIGHT = 49152 /**< Right. */
  } Direction;
  const char * tostring_Direction(Direction value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint8_t num_axes; /**< 
      The number of axes of this joystick
     */
    uint8_t num_buttons; /**< 
      The number of buttons of this joystick.
     */
    uint8_t supported_ff_effects; /**< 
      Bit field indicating available force-feedback effects.
     */
    uint32_t pressed_buttons; /**< 
      A bit field of enabled buttons. For each currently clicked button the
      corresponding bit is set to 1. Use the BUTTON_* constants for bit-wise
      comparisons.
     */
    float axis[8]; /**< Values of axes. */
    uint8_t ff_effects; /**< 
      Currently running effects. Either 0 if no effect is running, or a bit-wise
      ored field of the JFF constants.
     */
  } JoystickInterface_data_t;

  JoystickInterface_data_t *data;

  interface_enum_map_t enum_map_Direction;
 public:
  /* messages */
  class StartRumbleMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint16_t length; /**< Effect length in ms.
       Setting to 0 will make the effect to play continuously until stopped.
     */
      uint16_t delay; /**< Delay before effect starts in ms. */
      int32_t direction; /**< Direction of effect */
      uint16_t strong_magnitude; /**< Magnitude of heavy motor. */
      uint16_t weak_magnitude; /**< Magnitude of light motor. */
    } StartRumbleMessage_data_t;

    StartRumbleMessage_data_t *data;

  interface_enum_map_t enum_map_Direction;
   public:
    StartRumbleMessage(const uint16_t ini_length, const uint16_t ini_delay, const Direction ini_direction, const uint16_t ini_strong_magnitude, const uint16_t ini_weak_magnitude);
    StartRumbleMessage();
    ~StartRumbleMessage();

    StartRumbleMessage(const StartRumbleMessage *m);
    /* Methods */
    uint16_t length() const;
    void set_length(const uint16_t new_length);
    size_t maxlenof_length() const;
    uint16_t delay() const;
    void set_delay(const uint16_t new_delay);
    size_t maxlenof_delay() const;
    Direction direction() const;
    void set_direction(const Direction new_direction);
    size_t maxlenof_direction() const;
    uint16_t strong_magnitude() const;
    void set_strong_magnitude(const uint16_t new_strong_magnitude);
    size_t maxlenof_strong_magnitude() const;
    uint16_t weak_magnitude() const;
    void set_weak_magnitude(const uint16_t new_weak_magnitude);
    size_t maxlenof_weak_magnitude() const;
    virtual Message * clone() const;
  };

  class StopRumbleMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } StopRumbleMessage_data_t;

    StopRumbleMessage_data_t *data;

  interface_enum_map_t enum_map_Direction;
   public:
    StopRumbleMessage();
    ~StopRumbleMessage();

    StopRumbleMessage(const StopRumbleMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class StopAllMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } StopAllMessage_data_t;

    StopAllMessage_data_t *data;

  interface_enum_map_t enum_map_Direction;
   public:
    StopAllMessage();
    ~StopAllMessage();

    StopAllMessage(const StopAllMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  JoystickInterface();
  ~JoystickInterface();

 public:
  /* Methods */
  uint8_t num_axes() const;
  void set_num_axes(const uint8_t new_num_axes);
  size_t maxlenof_num_axes() const;
  uint8_t num_buttons() const;
  void set_num_buttons(const uint8_t new_num_buttons);
  size_t maxlenof_num_buttons() const;
  uint8_t supported_ff_effects() const;
  void set_supported_ff_effects(const uint8_t new_supported_ff_effects);
  size_t maxlenof_supported_ff_effects() const;
  uint32_t pressed_buttons() const;
  void set_pressed_buttons(const uint32_t new_pressed_buttons);
  size_t maxlenof_pressed_buttons() const;
  float * axis() const;
  float axis(unsigned int index) const;
  void set_axis(unsigned int index, const float new_axis);
  void set_axis(const float * new_axis);
  size_t maxlenof_axis() const;
  uint8_t ff_effects() const;
  void set_ff_effects(const uint8_t new_ff_effects);
  size_t maxlenof_ff_effects() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
