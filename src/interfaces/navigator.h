
/***************************************************************************
 *  navigator.h - Fawkes BlackBoard Interface - NavigatorInterface
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

#ifndef __INTERFACES_NAVIGATOR_H_
#define __INTERFACES_NAVIGATOR_H_

#include <interface/interface.h>
#include <interface/message.h>

class NavigatorInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(NavigatorInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int Foo; /**< Foo */
  } NavigatorInterface_data_t;

  NavigatorInterface_data_t *data;

 public:
  /* messages */
  class TargetMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float Distance; /**< Distance to the target. */
      float Angle; /**< Angle of the target. */
      float X; /**< X-coordinate of the target. */
      float Y; /**< Y-coordinate of the target. */
    } TargetMessage_data_t;

    TargetMessage_data_t *data;

   public:
    TargetMessage(float iniDistance, float iniAngle, float iniX, float iniY);
    TargetMessage();
    ~TargetMessage();

    /* Methods */
    float getDistance();
    void setDistance(float newDistance);
    float getAngle();
    void setAngle(float newAngle);
    float getX();
    void setX(float newX);
    float getY();
    void setY(float newY);
  };

  class VelocityMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float Velocity; /**< Velocity of the robot. */
    } VelocityMessage_data_t;

    VelocityMessage_data_t *data;

   public:
    VelocityMessage(float iniVelocity);
    VelocityMessage();
    ~VelocityMessage();

    /* Methods */
    float getVelocity();
    void setVelocity(float newVelocity);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  NavigatorInterface();
  ~NavigatorInterface();

 public:
  /* Methods */
  int getFoo();
  void setFoo(int newFoo);

};

#endif
