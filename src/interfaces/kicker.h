
/***************************************************************************
 *  kicker.h - Fawkes BlackBoard Interface - KickerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007  Daniel Beck
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

#ifndef __INTERFACES_KICKER_H_
#define __INTERFACES_KICKER_H_

#include <interface/interface.h>
#include <interface/message.h>

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

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int current_intensity; /**< 
      The currently set intensity.
     */
    int num_kicks_left; /**< 
      Number of Left-Kicks
     */
    int num_kicks_center; /**< 
      Number of Center-Kicks
     */
    int num_kicks_right; /**< 
      Number of Right-Kicks
     */
    GuideBallSideEnum guide_ball_side; /**< Side where the ball
      guidance arm is currently erected. */
  } KickerInterface_data_t;

  KickerInterface_data_t *data;

 public:
  /* messages */
  class KickMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      unsigned int intensity; /**< Intensity in the range [0..255]. */
      bool left; /**< True to kick with left kicker. */
      bool center; /**< True to kick with central kicker. */
      bool right; /**< True to kick with right kicker. */
    } KickMessage_data_t;

    KickMessage_data_t *data;

   public:
    KickMessage(bool ini_left, bool ini_center, bool ini_right, unsigned int ini_intensity);
    KickMessage();
    ~KickMessage();

    /* Methods */
    bool is_left();
    void set_left(const bool new_left);
    bool is_center();
    void set_center(const bool new_center);
    bool is_right();
    void set_right(const bool new_right);
    unsigned int intensity();
    void set_intensity(const unsigned int new_intensity);
  };

  class ResetCounterMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
    } ResetCounterMessage_data_t;

    ResetCounterMessage_data_t *data;

   public:
    ResetCounterMessage();
    ~ResetCounterMessage();

    /* Methods */
  };

  class GuideBallMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      GuideBallSideEnum guide_ball_side; /**< Side where to guide the ball and erect the arm. */
    } GuideBallMessage_data_t;

    GuideBallMessage_data_t *data;

   public:
    GuideBallMessage(GuideBallSideEnum ini_guide_ball_side);
    GuideBallMessage();
    ~GuideBallMessage();

    /* Methods */
    GuideBallSideEnum guide_ball_side();
    void set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side);
  };

  virtual bool message_valid(const Message *message) const;
 private:
  KickerInterface();
  ~KickerInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  int num_kicks_left();
  void set_num_kicks_left(const int new_num_kicks_left);
  int num_kicks_center();
  void set_num_kicks_center(const int new_num_kicks_center);
  int num_kicks_right();
  void set_num_kicks_right(const int new_num_kicks_right);
  GuideBallSideEnum guide_ball_side();
  void set_guide_ball_side(const GuideBallSideEnum new_guide_ball_side);
  unsigned int current_intensity();
  void set_current_intensity(const unsigned int new_current_intensity);

};

#endif
