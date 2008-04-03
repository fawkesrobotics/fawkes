
/***************************************************************************
 *  battery.h - Fawkes BlackBoard Interface - BatteryInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACES_BATTERY_H_
#define __INTERFACES_BATTERY_H_

#include <interface/interface.h>
#include <interface/message.h>

class BatteryInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(BatteryInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int current; /**< Battery Current [mA] */
    unsigned int voltage; /**< Battery Voltage [mV] */
    unsigned int temperature; /**< Battery Temperature [°C] */
  } BatteryInterface_data_t;

  BatteryInterface_data_t *data;

 public:
  /* messages */
  class PushButtonMessage : public Message
  {
   public:
    PushButtonMessage();
    ~PushButtonMessage();

    PushButtonMessage(const PushButtonMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class SleepMessage : public Message
  {
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
  virtual Message * create_message(const char *type) const;

  /* Methods */
  unsigned int current();
  void set_current(const unsigned int new_current);
  size_t maxlenof_current() const;
  unsigned int voltage();
  void set_voltage(const unsigned int new_voltage);
  size_t maxlenof_voltage() const;
  unsigned int temperature();
  void set_temperature(const unsigned int new_temperature);
  size_t maxlenof_temperature() const;

};

#endif
