
/***************************************************************************
 *  kicker.cpp - Fawkes BlackBoard Interface - KickerInterface
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

#include <interfaces/kicker.h>

#include <string.h>
#include <stdlib.h>

/** @class KickerInterface interfaces/kicker.h
 * KickerInterface Fawkes BlackBoard Interface.
 * 
      In these variables it is stored how often the right, center or 
      left kick have been triggered.
    
 */



/** Constructor */
KickerInterface::KickerInterface() : Interface()
{
  data_size = sizeof(KickerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickerInterface_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::~KickerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get NumKicksLeft value.
 * Number of Left-Kicks
 * @return NumKicksLeft value
 */
int
KickerInterface::getNumKicksLeft()
{
  return data->NumKicksLeft;
}

/** Set NumKicksLeft value.
 * Number of Left-Kicks
 * @param newNumKicksLeft new NumKicksLeft value
 */
void
KickerInterface::setNumKicksLeft(int newNumKicksLeft)
{
  data->NumKicksLeft = newNumKicksLeft;
}

/** Get NumKicksCenter value.
 * Number of Center-Kicks
 * @return NumKicksCenter value
 */
int
KickerInterface::getNumKicksCenter()
{
  return data->NumKicksCenter;
}

/** Set NumKicksCenter value.
 * Number of Center-Kicks
 * @param newNumKicksCenter new NumKicksCenter value
 */
void
KickerInterface::setNumKicksCenter(int newNumKicksCenter)
{
  data->NumKicksCenter = newNumKicksCenter;
}

/** Get NumKicksRight value.
 * Number of Right-Kicks
 * @return NumKicksRight value
 */
int
KickerInterface::getNumKicksRight()
{
  return data->NumKicksRight;
}

/** Set NumKicksRight value.
 * Number of Right-Kicks
 * @param newNumKicksRight new NumKicksRight value
 */
void
KickerInterface::setNumKicksRight(int newNumKicksRight)
{
  data->NumKicksRight = newNumKicksRight;
}

/** Get GuideBallSide value.
 * Side where the ball
      guidance arm is currently erected.
 * @return GuideBallSide value
 */
KickerInterface::GuideBallSideEnum
KickerInterface::getGuideBallSide()
{
  return data->GuideBallSide;
}

/** Set GuideBallSide value.
 * Side where the ball
      guidance arm is currently erected.
 * @param newGuideBallSide new GuideBallSide value
 */
void
KickerInterface::setGuideBallSide(GuideBallSideEnum newGuideBallSide)
{
  data->GuideBallSide = newGuideBallSide;
}

/* =========== messages =========== */
/** @class KickerInterface::KickMessage interfaces/kicker.h
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniCmdKickLeft initial value for CmdKickLeft
 * @param iniCmdKickCenter initial value for CmdKickCenter
 * @param iniCmdKickRight initial value for CmdKickRight
 * @param iniIntensity initial value for Intensity
 */
KickerInterface::KickMessage::KickMessage(bool iniCmdKickLeft, bool iniCmdKickCenter, bool iniCmdKickRight, int iniIntensity) : Message()
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data->CmdKickLeft = iniCmdKickLeft;
  data->CmdKickCenter = iniCmdKickCenter;
  data->CmdKickRight = iniCmdKickRight;
  data->Intensity = iniIntensity;
}
/** Constructor */
KickerInterface::KickMessage::KickMessage() : Message()
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::KickMessage::~KickMessage()
{
}
/* Methods */
/** Get CmdKickLeft value.
 * CmdKickLeft
 * @return CmdKickLeft value
 */
bool
KickerInterface::KickMessage::isCmdKickLeft()
{
  return data->CmdKickLeft;
}

/** Set CmdKickLeft value.
 * CmdKickLeft
 * @param newCmdKickLeft new CmdKickLeft value
 */
void
KickerInterface::KickMessage::setCmdKickLeft(bool newCmdKickLeft)
{
  data->CmdKickLeft = newCmdKickLeft;
}

/** Get CmdKickCenter value.
 * CmdKickCenter
 * @return CmdKickCenter value
 */
bool
KickerInterface::KickMessage::isCmdKickCenter()
{
  return data->CmdKickCenter;
}

/** Set CmdKickCenter value.
 * CmdKickCenter
 * @param newCmdKickCenter new CmdKickCenter value
 */
void
KickerInterface::KickMessage::setCmdKickCenter(bool newCmdKickCenter)
{
  data->CmdKickCenter = newCmdKickCenter;
}

/** Get CmdKickRight value.
 * CmdKickRight
 * @return CmdKickRight value
 */
bool
KickerInterface::KickMessage::isCmdKickRight()
{
  return data->CmdKickRight;
}

/** Set CmdKickRight value.
 * CmdKickRight
 * @param newCmdKickRight new CmdKickRight value
 */
void
KickerInterface::KickMessage::setCmdKickRight(bool newCmdKickRight)
{
  data->CmdKickRight = newCmdKickRight;
}

/** Get Intensity value.
 * Intensity
 * @return Intensity value
 */
int
KickerInterface::KickMessage::getIntensity()
{
  return data->Intensity;
}

/** Set Intensity value.
 * Intensity
 * @param newIntensity new Intensity value
 */
void
KickerInterface::KickMessage::setIntensity(int newIntensity)
{
  data->Intensity = newIntensity;
}

/** @class KickerInterface::ResetCounterMessage interfaces/kicker.h
 * ResetCounterMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
KickerInterface::ResetCounterMessage::ResetCounterMessage() : Message()
{
  data_size = sizeof(ResetCounterMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (ResetCounterMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::ResetCounterMessage::~ResetCounterMessage()
{
}
/* Methods */
/** @class KickerInterface::GuideBallMessage interfaces/kicker.h
 * GuideBallMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniGuideBallSide initial value for GuideBallSide
 */
KickerInterface::GuideBallMessage::GuideBallMessage(GuideBallSideEnum iniGuideBallSide) : Message()
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
  data->GuideBallSide = iniGuideBallSide;
}
/** Constructor */
KickerInterface::GuideBallMessage::GuideBallMessage() : Message()
{
  data_size = sizeof(GuideBallMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (GuideBallMessage_data_t *)data_ptr;
}
/** Destructor */
KickerInterface::GuideBallMessage::~GuideBallMessage()
{
}
/* Methods */
/** Get GuideBallSide value.
 * Side where to guide the ball and erect the arm.
 * @return GuideBallSide value
 */
KickerInterface::GuideBallSideEnum
KickerInterface::GuideBallMessage::getGuideBallSide()
{
  return data->GuideBallSide;
}

/** Set GuideBallSide value.
 * Side where to guide the ball and erect the arm.
 * @param newGuideBallSide new GuideBallSide value
 */
void
KickerInterface::GuideBallMessage::setGuideBallSide(GuideBallSideEnum newGuideBallSide)
{
  data->GuideBallSide = newGuideBallSide;
}

/** Check if message is valid an can be queued.
 * @param message Message to check
 */
bool
KickerInterface::messageValid(const Message *message) const
{
  const KickMessage *m0 = dynamic_cast<const KickMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const ResetCounterMessage *m1 = dynamic_cast<const ResetCounterMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const GuideBallMessage *m2 = dynamic_cast<const GuideBallMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(KickerInterface)
/// @endcond

