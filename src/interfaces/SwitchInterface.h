
/***************************************************************************
 *  SwitchInterface.h - Fawkes BlackBoard Interface - SwitchInterface
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

#ifndef __INTERFACES_SWITCHINTERFACE_H_
#define __INTERFACES_SWITCHINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class SwitchInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SwitchInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int short_activations; /**< 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
     */
    unsigned int long_activations; /**< 
      Number of consecutive short clicks (turned on). Can be used to recognize
      patterns of clicks. This is an optional field.
     */
    unsigned int activation_count; /**< 
      Number that is to be incremented whenever a short or long activation
      happened. Can be used to decide if a change in status happened.
     */
    float value; /**< 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
     */
    float history; /**< 
      This value records the number of seconds a switch has been
      enabled continuously -- or not. The time is recorded in
      seconds. A positive value indicates time the switch was turned
      on, a negative value indicates the time (when converted to the
      absolute value) the button has not been pressed. Zero means
      "just initialized".
     */
    bool enabled; /**< 
      True if the switch is currently enabled.
     */
  } SwitchInterface_data_t;

  SwitchInterface_data_t *data;

 public:
  /* messages */
  class SetMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float value; /**< 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
     */
      bool enabled; /**< 
      True if the switch is currently enabled.
     */
    } SetMessage_data_t;

    SetMessage_data_t *data;

   public:
    SetMessage(const bool ini_enabled, const float ini_value);
    SetMessage();
    ~SetMessage();

    SetMessage(const SetMessage *m);
    /* Methods */
    bool is_enabled() const;
    void set_enabled(const bool new_enabled);
    size_t maxlenof_enabled() const;
    float value() const;
    void set_value(const float new_value);
    size_t maxlenof_value() const;
    virtual Message * clone() const;
  };

  class EnableSwitchMessage : public Message
  {
   public:
    EnableSwitchMessage();
    ~EnableSwitchMessage();

    EnableSwitchMessage(const EnableSwitchMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class DisableSwitchMessage : public Message
  {
   public:
    DisableSwitchMessage();
    ~DisableSwitchMessage();

    DisableSwitchMessage(const DisableSwitchMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class EnableDurationMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float duration; /**< Duration in seconds for which
    the switch should be enabled. */
      float value; /**< 
      If switches support multiple states these can be indicated with
      this value. For example for a switch that notes the intensity it
      could be a value in the valid range.
     */
    } EnableDurationMessage_data_t;

    EnableDurationMessage_data_t *data;

   public:
    EnableDurationMessage(const float ini_duration, const float ini_value);
    EnableDurationMessage();
    ~EnableDurationMessage();

    EnableDurationMessage(const EnableDurationMessage *m);
    /* Methods */
    float duration() const;
    void set_duration(const float new_duration);
    size_t maxlenof_duration() const;
    float value() const;
    void set_value(const float new_value);
    size_t maxlenof_value() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SwitchInterface();
  ~SwitchInterface();

 public:
  /* Methods */
  bool is_enabled() const;
  void set_enabled(const bool new_enabled);
  size_t maxlenof_enabled() const;
  float value() const;
  void set_value(const float new_value);
  size_t maxlenof_value() const;
  float history() const;
  void set_history(const float new_history);
  size_t maxlenof_history() const;
  unsigned int short_activations() const;
  void set_short_activations(const unsigned int new_short_activations);
  size_t maxlenof_short_activations() const;
  unsigned int long_activations() const;
  void set_long_activations(const unsigned int new_long_activations);
  size_t maxlenof_long_activations() const;
  unsigned int activation_count() const;
  void set_activation_count(const unsigned int new_activation_count);
  size_t maxlenof_activation_count() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
