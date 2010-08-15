
/***************************************************************************
 *  SoccerPenaltyInterface.h - Fawkes BlackBoard Interface - SoccerPenaltyInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008-2010  Tim Niemueller
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

#ifndef __INTERFACES_SOCCERPENALTYINTERFACE_H_
#define __INTERFACES_SOCCERPENALTYINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class SoccerPenaltyInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SoccerPenaltyInterface)
 /// @endcond
 public:
  /* constants */
  static const uint16_t SPL_PENALTY_NONE;
  static const uint16_t SPL_PENALTY_BALL_HOLDING;
  static const uint16_t SPL_PENALTY_PLAYER_PUSHING;
  static const uint16_t SPL_PENALTY_OBSTRUCTION;
  static const uint16_t SPL_PENALTY_INACTIVE_PLAYER;
  static const uint16_t SPL_PENALTY_ILLEGAL_DEFENDER;
  static const uint16_t SPL_PENALTY_LEAVING_THE_FIELD;
  static const uint16_t SPL_PENALTY_PLAYING_WITH_HANDS;
  static const uint16_t SPL_PENALTY_REQ_FOR_PICKUP;
  static const uint16_t SPL_PENALTY_MANUAL;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint16_t penalty; /**< Current penalty code. */
    uint16_t remaining; /**< Estimated time in seconds until the robot is unpenalized. */
  } SoccerPenaltyInterface_data_t;
#pragma pack(pop)

  SoccerPenaltyInterface_data_t *data;

 public:
  /* messages */
  class SetPenaltyMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint16_t penalty; /**< Current penalty code. */
    } SetPenaltyMessage_data_t;
#pragma pack(pop)

    SetPenaltyMessage_data_t *data;

   public:
    SetPenaltyMessage(const uint16_t ini_penalty);
    SetPenaltyMessage();
    ~SetPenaltyMessage();

    SetPenaltyMessage(const SetPenaltyMessage *m);
    /* Methods */
    uint16_t penalty() const;
    void set_penalty(const uint16_t new_penalty);
    size_t maxlenof_penalty() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SoccerPenaltyInterface();
  ~SoccerPenaltyInterface();

 public:
  /* Methods */
  uint16_t penalty() const;
  void set_penalty(const uint16_t new_penalty);
  size_t maxlenof_penalty() const;
  uint16_t remaining() const;
  void set_remaining(const uint16_t new_remaining);
  size_t maxlenof_remaining() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
