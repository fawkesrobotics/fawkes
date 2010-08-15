
/***************************************************************************
 *  BatteryInterface.h - Fawkes BlackBoard Interface - BatteryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Daniel Beck
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

#ifndef __INTERFACES_BATTERYINTERFACE_H_
#define __INTERFACES_BATTERYINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class BatteryInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(BatteryInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t current; /**< Battery Current [mA] */
    uint32_t voltage; /**< Battery Voltage [mV] */
    uint32_t temperature; /**< Battery Temperature [°C] */
    float absolute_soc; /**< Absolute state of charge [%] */
    float relative_soc; /**< Relative state of charge [%] */
  } BatteryInterface_data_t;
#pragma pack(pop)

  BatteryInterface_data_t *data;

 public:
  /* messages */
  class PushButtonMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } PushButtonMessage_data_t;
#pragma pack(pop)

    PushButtonMessage_data_t *data;

   public:
    PushButtonMessage();
    ~PushButtonMessage();

    PushButtonMessage(const PushButtonMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class SleepMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } SleepMessage_data_t;
#pragma pack(pop)

    SleepMessage_data_t *data;

   public:
    SleepMessage();
    ~SleepMessage();

    SleepMessage(const SleepMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  BatteryInterface();
  ~BatteryInterface();

 public:
  /* Methods */
  uint32_t current() const;
  void set_current(const uint32_t new_current);
  size_t maxlenof_current() const;
  uint32_t voltage() const;
  void set_voltage(const uint32_t new_voltage);
  size_t maxlenof_voltage() const;
  uint32_t temperature() const;
  void set_temperature(const uint32_t new_temperature);
  size_t maxlenof_temperature() const;
  float absolute_soc() const;
  void set_absolute_soc(const float new_absolute_soc);
  size_t maxlenof_absolute_soc() const;
  float relative_soc() const;
  void set_relative_soc(const float new_relative_soc);
  size_t maxlenof_relative_soc() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
