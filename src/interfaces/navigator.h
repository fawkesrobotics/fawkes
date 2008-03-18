
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
      float x; /**< X-coordinate of the target, in the robot's coordinate system. */
      float y; /**< Y-coordinate of the target, in the robot's coordinate system. */
      float orientation; /**< The orientation of the robot at the target. */
    } TargetMessage_data_t;

    TargetMessage_data_t *data;

   public:
    TargetMessage(const float ini_x, const float ini_y, const float ini_orientation);
    TargetMessage();
    ~TargetMessage();

    /* Methods */
    float x();
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y();
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float orientation();
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
  };

  class MaxVelocityMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float velocity; /**< Maximum velocity of the robot. */
    } MaxVelocityMessage_data_t;

    MaxVelocityMessage_data_t *data;

   public:
    MaxVelocityMessage(const float ini_velocity);
    MaxVelocityMessage();
    ~MaxVelocityMessage();

    /* Methods */
    float velocity();
    void set_velocity(const float new_velocity);
    size_t maxlenof_velocity() const;
  };

  class ObstacleMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float x; /**< X-coordinate of the obstacle. */
      float y; /**< Y-coordinate of the obstacle. */
      float width; /**< Width of the obstacle. */
    } ObstacleMessage_data_t;

    ObstacleMessage_data_t *data;

   public:
    ObstacleMessage(const float ini_x, const float ini_y, const float ini_width);
    ObstacleMessage();
    ~ObstacleMessage();

    /* Methods */
    float x();
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y();
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float width();
    void set_width(const float new_width);
    size_t maxlenof_width() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  NavigatorInterface();
  ~NavigatorInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  int foo();
  void set_foo(const int new_foo);
  size_t maxlenof_foo() const;

};

#endif
