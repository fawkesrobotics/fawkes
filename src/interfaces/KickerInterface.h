
/***************************************************************************
 *  KickerInterface.h - Fawkes BlackBoard Interface - KickerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Daniel Beck
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

#ifndef __INTERFACES_KICKERINTERFACE_H_
#define __INTERFACES_KICKERINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class KickerInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(KickerInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Enumeration defining on which side of the robot the ball shall be
        guided (and thus on which side the arm is to be erected).
       */
  typedef enum {
    GUIDE_BALL_LEFT /**< 
        Constant defining that the kicker shall activate the ball guidance device
        in such a way that the left arm is erected.
       */,
    GUIDE_BALL_RIGHT /**< 
        Constant defining that the kicker shall activate the ball guidance device
        in such a way that the right arm is erected.
       */
  } GuideBallSideEnum;
  const char * tostring_GuideBallSideEnum(GuideBallSideEnum value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t num_kicks_left; /**< 
      Number of Left-Kicks
     */
    int32_t num_kicks_center; /**< 
      Number of Center-Kicks
     */
    int32_t num_kicks_right; /**< 
      Number of Right-Kicks
     */
    GuideBallSideEnum guide_ball_side; /**< Side where the ball
      guidance arm is currently erected. */
    uint32_t current_intensity; /**< 
      The currently set intensity.
     */
  } KickerInterface_data_t;
#pragma pack(pop)

  KickerInterface_data_t *data;

 public:
  /* messages */
  class KickMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      bool left; /**< True to kick with left kicker. */
      bool center; /**< True to kick with central kicker. */
      bool right; /**< True to kick with right kicker. */
      uint32_t intensity; /**< Intensity in the range [0..255]. */
    } KickMessage_data_t;
#pragma pack(pop)

    KickMessage_data_t *data;

   public:
    KickMessage(const bool ini_left, const bool ini_center, const bool ini_right, const uint32_t ini_intensity);
    KickMessage();
    ~KickMessage();

    KickMessage(const KickMessage *m);
    /* Methods */
    bool is_left() const;
    void set_left(const bool new_left);
    size_t maxlenof_left() const;
    bool is_center() const;
    void set_center(const bool new_center);
    size_t maxlenof_center() const;
    bool is_right() const;
    void set_right(const bool new_right);
    size_t maxlenof_right() const;
    uint32_t intensity() const;
    void set_intensity(const uint32_t new_intensity);
    size_t maxlenof_intensity() const;
    virtual Message * clone() const;
  };

  class ResetCounterMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ResetCounterMessage_data_t;
#pragma pack(pop)

    ResetCounterMessage_data_t *data;

   public:
    ResetCounterMessage();
    ~ResetCounterMessage();

    ResetCounterMessage(const ResetCounterMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class GuideBallMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      GuideBallSideEnum guide_ball_side; /**< Side where to guide the ball and erect the arm. */
    } GuideBallMessage_data_t;
#pragma pack(pop)

    GuideBallMessage_data_t *data;

   public:
    GuideBallMessage(const GuideBallSideEnum ini_guide_ball_side);
    GuideBallMessage();
    ~GuideBallMessage();

    GuideBallMessage(const GuideBallMessage *m);
    /* Methods */
    GuideBallSideEnum guide_ball_side() const;
    void set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side);
    size_t maxlenof_guide_ball_side() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  KickerInterface();
  ~KickerInterface();

 public:
  /* Methods */
  int32_t num_kicks_left() const;
  void set_num_kicks_left(const int32_t new_num_kicks_left);
  size_t maxlenof_num_kicks_left() const;
  int32_t num_kicks_center() const;
  void set_num_kicks_center(const int32_t new_num_kicks_center);
  size_t maxlenof_num_kicks_center() const;
  int32_t num_kicks_right() const;
  void set_num_kicks_right(const int32_t new_num_kicks_right);
  size_t maxlenof_num_kicks_right() const;
  GuideBallSideEnum guide_ball_side() const;
  void set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side);
  size_t maxlenof_guide_ball_side() const;
  uint32_t current_intensity() const;
  void set_current_intensity(const uint32_t new_current_intensity);
  size_t maxlenof_current_intensity() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
