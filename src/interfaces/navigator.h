
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
    int foo; /**< Foo */
  } NavigatorInterface_data_t;

  NavigatorInterface_data_t *data;

 public:
  /* messages */
  class TargetMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float distance; /**< Distance to the target. */
      float angle; /**< Angle of the target. */
      float x; /**< X-coordinate of the target. */
      float y; /**< Y-coordinate of the target. */
    } TargetMessage_data_t;

    TargetMessage_data_t *data;

   public:
    TargetMessage(float ini_distance, float ini_angle, float ini_x, float ini_y);
    TargetMessage();
    ~TargetMessage();

    /* Methods */
    float distance();
    void set_distance(const float new_distance);
    float angle();
    void set_angle(const float new_angle);
    float x();
    void set_x(const float new_x);
    float y();
    void set_y(const float new_y);
  };

  class VelocityMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float velocity; /**< Velocity of the robot. */
    } VelocityMessage_data_t;

    VelocityMessage_data_t *data;

   public:
    VelocityMessage(float ini_velocity);
    VelocityMessage();
    ~VelocityMessage();

    /* Methods */
    float velocity();
    void set_velocity(const float new_velocity);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  NavigatorInterface();
  ~NavigatorInterface();

 public:
  /* Methods */
  int foo();
  void set_foo(const int new_foo);

};

#endif
