
/***************************************************************************
 *  GripperInterface.h - Fawkes BlackBoard Interface - GripperInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Sebastian Reuter
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

#ifndef __INTERFACES_GRIPPERINTERFACE_H_
#define __INTERFACES_GRIPPERINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class GripperInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(GripperInterface)
 /// @endcond
 public:
  /* constants */

  /** Indicator of current or desired gripper state. */
  typedef enum {
    OPEN /**< Gripper is open */,
    CLOSED /**< Gripper is closed */
  } GripperState;
  const char * tostring_GripperState(GripperState value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t gripper_state; /**< 
      The current state of the gripper.
     */
  } GripperInterface_data_t;
#pragma pack(pop)

  GripperInterface_data_t *data;

  interface_enum_map_t enum_map_GripperState;
 public:
  /* messages */
  class OpenGripperMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } OpenGripperMessage_data_t;
#pragma pack(pop)

    OpenGripperMessage_data_t *data;

  interface_enum_map_t enum_map_GripperState;
   public:
    OpenGripperMessage();
    ~OpenGripperMessage();

    OpenGripperMessage(const OpenGripperMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class CloseGripperMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } CloseGripperMessage_data_t;
#pragma pack(pop)

    CloseGripperMessage_data_t *data;

  interface_enum_map_t enum_map_GripperState;
   public:
    CloseGripperMessage();
    ~CloseGripperMessage();

    CloseGripperMessage(const CloseGripperMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  GripperInterface();
  ~GripperInterface();

 public:
  /* Methods */
  GripperState gripper_state() const;
  void set_gripper_state(const GripperState new_gripper_state);
  size_t maxlenof_gripper_state() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
