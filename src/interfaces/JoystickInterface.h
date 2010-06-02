
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

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint8_t num_axes; /**< 
      The number of axes of this joystick
     */
    uint8_t num_buttons; /**< 
      The number of buttons of this joystick.
     */
    uint32_t pressed_buttons; /**< 
      A bit field of enabled buttons. For each currently clicked button the
      corresponding bit is set to 1. Use the BUTTON_* constants for bit-wise
      comparisons.
     */
    float axis_x[4]; /**< X values of axes */
    float axis_y[4]; /**< Y values of axes */
  } JoystickInterface_data_t;
#pragma pack(pop)

  JoystickInterface_data_t *data;

 public:
  /* messages */
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
  uint32_t pressed_buttons() const;
  void set_pressed_buttons(const uint32_t new_pressed_buttons);
  size_t maxlenof_pressed_buttons() const;
  float * axis_x() const;
  float axis_x(unsigned int index) const;
  void set_axis_x(unsigned int index, const float new_axis_x);
  void set_axis_x(const float * new_axis_x);
  size_t maxlenof_axis_x() const;
  float * axis_y() const;
  float axis_y(unsigned int index) const;
  void set_axis_y(unsigned int index, const float new_axis_y);
  void set_axis_y(const float * new_axis_y);
  size_t maxlenof_axis_y() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
