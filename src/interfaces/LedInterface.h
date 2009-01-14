
/***************************************************************************
 *  LedInterface.h - Fawkes BlackBoard Interface - LedInterface
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

#ifndef __INTERFACES_LEDINTERFACE_H_
#define __INTERFACES_LEDINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

class LedInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(LedInterface)
 /// @endcond
 public:
  /* constants */
  static const float ON;
  static const float OFF;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    float intensity; /**< Intensity value. */
  } LedInterface_data_t;

  LedInterface_data_t *data;

 public:
  /* messages */
  class SetIntensityMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float time_sec; /**< 
      Time in seconds when to reach the intensity.
     */
      float intensity; /**< Intensity value. */
    } SetIntensityMessage_data_t;

    SetIntensityMessage_data_t *data;

   public:
    SetIntensityMessage(const float ini_time_sec, const float ini_intensity);
    SetIntensityMessage();
    ~SetIntensityMessage();

    SetIntensityMessage(const SetIntensityMessage *m);
    /* Methods */
    float time_sec() const;
    void set_time_sec(const float new_time_sec);
    size_t maxlenof_time_sec() const;
    float intensity() const;
    void set_intensity(const float new_intensity);
    size_t maxlenof_intensity() const;
    virtual Message * clone() const;
  };

  class TurnOnMessage : public Message
  {
   public:
    TurnOnMessage();
    ~TurnOnMessage();

    TurnOnMessage(const TurnOnMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class TurnOffMessage : public Message
  {
   public:
    TurnOffMessage();
    ~TurnOffMessage();

    TurnOffMessage(const TurnOffMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  LedInterface();
  ~LedInterface();

 public:
  /* Methods */
  float intensity() const;
  void set_intensity(const float new_intensity);
  size_t maxlenof_intensity() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
