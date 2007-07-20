
/***************************************************************************
 *  motor.h - Fawkes BlackBoard Interface - MotorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg
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
 *  along with this program; if not, write to the Free Software
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth floor, Boston, MA 02111-1307, USA.
 */

#ifndef __INTERFACES_MOTOR_H_
#define __INTERFACES_MOTOR_H_

#include <interface/interface.h>
#include <interface/message.h>

class MotorInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(MotorInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int RPM1; /**< RPM of motor 1. */
    int RPM2; /**< RPM of motor 2. */
    int RPM3; /**< RPM of motor 3. */
    unsigned long int ControllerID; /**< The ID of the Controller Thread, which controlls the motors. */
  } MotorInterface_data_t;

  MotorInterface_data_t *data;

 public:
  /* messages */
  class JoystickMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float CmdForward; /**< The forward command. */
      float CmdSideward; /**< The sideward command. */
      float CmdRotation; /**< The rotation command. */
      float CmdSpeed; /**< The speed command. */
    } JoystickMessage_data_t;

    JoystickMessage_data_t *data;

   public:
    JoystickMessage(float iniCmdForward, float iniCmdSideward, float iniCmdRotation, float iniCmdSpeed);
    JoystickMessage();
    ~JoystickMessage();

    /* Methods */
    float getCmdForward();
    void setCmdForward(float newCmdForward);
    float getCmdSideward();
    void setCmdSideward(float newCmdSideward);
    float getCmdRotation();
    void setCmdRotation(float newCmdRotation);
    float getCmdSpeed();
    void setCmdSpeed(float newCmdSpeed);
  };

  class NavigatorMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float CmdForward; /**< The forward command. */
      float CmdSideward; /**< The sideward command. */
      float CmdRotation; /**< The rotation command. */
      float CmdVelocity; /**< The velocity command. */
    } NavigatorMessage_data_t;

    NavigatorMessage_data_t *data;

   public:
    NavigatorMessage(float iniCmdForward, float iniCmdSideward, float iniCmdRotation, float iniCmdVelocity);
    NavigatorMessage();
    ~NavigatorMessage();

    /* Methods */
    float getCmdForward();
    void setCmdForward(float newCmdForward);
    float getCmdSideward();
    void setCmdSideward(float newCmdSideward);
    float getCmdRotation();
    void setCmdRotation(float newCmdRotation);
    float getCmdVelocity();
    void setCmdVelocity(float newCmdVelocity);
  };

  class SwitchMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int ThreadId; /**< The Thread Id of the Thread
    		which is allowed to control the motors. */
    } SwitchMessage_data_t;

    SwitchMessage_data_t *data;

   public:
    SwitchMessage(unsigned int iniThreadId);
    SwitchMessage();
    ~SwitchMessage();

    /* Methods */
    unsigned int getThreadId();
    void setThreadId(unsigned int newThreadId);
  };

  class SubscribeMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int Subscriber; /**< 0 if unsubscribe and 1 if subscribe. */
    } SubscribeMessage_data_t;

    SubscribeMessage_data_t *data;

   public:
    SubscribeMessage(unsigned int iniSubscriber);
    SubscribeMessage();
    ~SubscribeMessage();

    /* Methods */
    unsigned int getSubscriber();
    void setSubscriber(unsigned int newSubscriber);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  MotorInterface();
  ~MotorInterface();

 public:
  /* Methods */
  int getRPM1();
  void setRPM1(int newRPM1);
  int getRPM2();
  void setRPM2(int newRPM2);
  int getRPM3();
  void setRPM3(int newRPM3);
  unsigned long int getControllerID();
  void setControllerID(unsigned long int newControllerID);

};

#endif
