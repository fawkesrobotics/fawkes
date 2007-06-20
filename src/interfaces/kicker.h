
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
 friend Interface *  private_newKickerInterface();
/// @endcond
 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int NumKicksRight; /**< Number of Right-Kicks */
    int NumKicksCenter; /**< Number of Center-Kicks */
    int NumKicksLeft; /**< Number of Left-Kicks */
  } KickerInterface_data_t;

  KickerInterface_data_t *data;

 public:
  /* constants */

  /* messages */
  class KickMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int Intensity; /**< Intensity */
      bool CmdKickRight; /**< CmdKickRight */
      bool CmdKickCenter; /**< CmdKickCenter */
      bool CmdKickLeft; /**< CmdKickLeft */
      bool CmdResetCounter; /**< CmdResetCounter */
      bool CmdActivateBallGuidance; /**< CmdActivateBallGuidance */
    } KickMessage_data_t;

    KickMessage_data_t *data;

   public:
    KickMessage(bool iniCmdKickRight, bool iniCmdKickCenter, bool iniCmdKickLeft, int iniIntensity, bool iniCmdResetCounter, bool iniCmdActivateBallGuidance);
    KickMessage();
    ~KickMessage();

    /* Methods */
    bool isCmdKickRight();
    void setCmdKickRight(bool newCmdKickRight);
    bool isCmdKickCenter();
    void setCmdKickCenter(bool newCmdKickCenter);
    bool isCmdKickLeft();
    void setCmdKickLeft(bool newCmdKickLeft);
    int getIntensity();
    void setIntensity(int newIntensity);
    bool isCmdResetCounter();
    void setCmdResetCounter(bool newCmdResetCounter);
    bool isCmdActivateBallGuidance();
    void setCmdActivateBallGuidance(bool newCmdActivateBallGuidance);
  };

  virtual bool messageValid(const Message *message) const;
 private:
  KickerInterface();
  ~KickerInterface();

 public:
  /* Methods */
  int getNumKicksRight();
  void setNumKicksRight(int newNumKicksRight);
  int getNumKicksCenter();
  void setNumKicksCenter(int newNumKicksCenter);
  int getNumKicksLeft();
  void setNumKicksLeft(int newNumKicksLeft);

};

#endif
