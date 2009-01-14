
/***************************************************************************
 *  NavigatorInterface.h - Fawkes BlackBoard Interface - NavigatorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Martin Liebenberg
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

#ifndef __INTERFACES_NAVIGATORINTERFACE_H_
#define __INTERFACES_NAVIGATORINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

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
    unsigned int msgid; /**< The ID of the message that is currently being
      processed, or 0 if no message is being processed. */
    float x; /**< Current X-coordinate in the navigator coordinate system. */
    float y; /**< Current Y-coordinate in the navigator coordinate system. */
    float dest_x; /**< X-coordinate of the current destination, or 0.0 if no target has been set. */
    float dest_y; /**< Y-coordinate of the current destination, or 0.0 if no target has been set. */
    float dest_dist; /**< Distance to destination in m. */
    bool final; /**< True, if the last goto command has been finished,
      false if it is still running */
  } NavigatorInterface_data_t;

  NavigatorInterface_data_t *data;

 public:
  /* messages */
  class CartesianGotoMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float x; /**< X-coordinate of the target, in the robot's coordinate system. */
      float y; /**< Y-coordinate of the target, in the robot's coordinate system. */
      float orientation; /**< The orientation of the robot at the target. */
    } CartesianGotoMessage_data_t;

    CartesianGotoMessage_data_t *data;

   public:
    CartesianGotoMessage(const float ini_x, const float ini_y, const float ini_orientation);
    CartesianGotoMessage();
    ~CartesianGotoMessage();

    CartesianGotoMessage(const CartesianGotoMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float orientation() const;
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
    virtual Message * clone() const;
  };

  class PolarGotoMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      float phi; /**< Angle between the robot's front and the target. */
      float dist; /**< Distance to the target. */
      float orientation; /**< The orientation of the robot at the target. */
    } PolarGotoMessage_data_t;

    PolarGotoMessage_data_t *data;

   public:
    PolarGotoMessage(const float ini_phi, const float ini_dist, const float ini_orientation);
    PolarGotoMessage();
    ~PolarGotoMessage();

    PolarGotoMessage(const PolarGotoMessage *m);
    /* Methods */
    float phi() const;
    void set_phi(const float new_phi);
    size_t maxlenof_phi() const;
    float dist() const;
    void set_dist(const float new_dist);
    size_t maxlenof_dist() const;
    float orientation() const;
    void set_orientation(const float new_orientation);
    size_t maxlenof_orientation() const;
    virtual Message * clone() const;
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

    MaxVelocityMessage(const MaxVelocityMessage *m);
    /* Methods */
    float velocity() const;
    void set_velocity(const float new_velocity);
    size_t maxlenof_velocity() const;
    virtual Message * clone() const;
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

    ObstacleMessage(const ObstacleMessage *m);
    /* Methods */
    float x() const;
    void set_x(const float new_x);
    size_t maxlenof_x() const;
    float y() const;
    void set_y(const float new_y);
    size_t maxlenof_y() const;
    float width() const;
    void set_width(const float new_width);
    size_t maxlenof_width() const;
    virtual Message * clone() const;
  };

  class ResetOdometryMessage : public Message
  {
   public:
    ResetOdometryMessage();
    ~ResetOdometryMessage();

    ResetOdometryMessage(const ResetOdometryMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  NavigatorInterface();
  ~NavigatorInterface();

 public:
  /* Methods */
  float x() const;
  void set_x(const float new_x);
  size_t maxlenof_x() const;
  float y() const;
  void set_y(const float new_y);
  size_t maxlenof_y() const;
  float dest_x() const;
  void set_dest_x(const float new_dest_x);
  size_t maxlenof_dest_x() const;
  float dest_y() const;
  void set_dest_y(const float new_dest_y);
  size_t maxlenof_dest_y() const;
  float dest_dist() const;
  void set_dest_dist(const float new_dest_dist);
  size_t maxlenof_dest_dist() const;
  unsigned int msgid() const;
  void set_msgid(const unsigned int new_msgid);
  size_t maxlenof_msgid() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);

};

} // end namespace fawkes

#endif
