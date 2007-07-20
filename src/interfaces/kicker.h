
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
 *  along with this program; if not, write to the Free Software
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
    int NumKicksLeft; /**< Number of Left-Kicks */
    int NumKicksCenter; /**< Number of Center-Kicks */
    int NumKicksRight; /**< Number of Right-Kicks */
    GuideBallSideEnum GuideBallSide; /**< Side where the ball
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
      int Intensity; /**< Intensity */
      bool CmdKickLeft; /**< CmdKickLeft */
      bool CmdKickCenter; /**< CmdKickCenter */
      bool CmdKickRight; /**< CmdKickRight */
    } KickMessage_data_t;

    KickMessage_data_t *data;

   public:
    KickMessage(bool iniCmdKickLeft, bool iniCmdKickCenter, bool iniCmdKickRight, int iniIntensity);
    KickMessage();
    ~KickMessage();

    /* Methods */
    bool isCmdKickLeft();
    void setCmdKickLeft(bool newCmdKickLeft);
    bool isCmdKickCenter();
    void setCmdKickCenter(bool newCmdKickCenter);
    bool isCmdKickRight();
    void setCmdKickRight(bool newCmdKickRight);
    int getIntensity();
    void setIntensity(int newIntensity);
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
      GuideBallSideEnum GuideBallSide; /**< Side where to guide the ball and erect the arm. */
    } GuideBallMessage_data_t;

    GuideBallMessage_data_t *data;

   public:
    GuideBallMessage(GuideBallSideEnum iniGuideBallSide);
    GuideBallMessage();
    ~GuideBallMessage();

    /* Methods */
    GuideBallSideEnum getGuideBallSide();
    void setGuideBallSide(GuideBallSideEnum newGuideBallSide);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  KickerInterface();
  ~KickerInterface();

 public:
  /* Methods */
  int getNumKicksLeft();
  void setNumKicksLeft(int newNumKicksLeft);
  int getNumKicksCenter();
  void setNumKicksCenter(int newNumKicksCenter);
  int getNumKicksRight();
  void setNumKicksRight(int newNumKicksRight);
  GuideBallSideEnum getGuideBallSide();
  void setGuideBallSide(GuideBallSideEnum newGuideBallSide);

};

#endif
