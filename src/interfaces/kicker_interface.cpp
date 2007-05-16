
/***************************************************************************
 *  kicker_interface.cpp - Fawkes BlackBoard Interface - KickerInterface
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

#include <interfaces/kicker_interface.h>

#include <string.h>
#include <stdlib.h>

/** @class KickerInterface interfaces/kicker_interface.h
 * KickerInterface Fawkes BlackBoard Interface.
 * In these variables it is stored how often the right, center or 
      left kick have been triggered
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

/* =========== messages =========== */
/** @class KickerInterface::KickMessage interfaces/kicker_interface.h
 * KickMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param iniCmdKickRight initial value for CmdKickRight
 * @param iniCmdKickCenter initial value for CmdKickCenter
 * @param iniCmdKickLeft initial value for CmdKickLeft
 * @param iniIntensity initial value for Intensity
 * @param iniCmdResetCounter initial value for CmdResetCounter
 * @param iniCmdActivateBallGuidance initial value for CmdActivateBallGuidance
 */
KickerInterface::KickMessage::KickMessage(bool iniCmdKickRight, bool iniCmdKickCenter, bool iniCmdKickLeft, int iniIntensity, bool iniCmdResetCounter, bool iniCmdActivateBallGuidance) : Message()
{
  data_size = sizeof(KickMessage_data_t);
  data_ptr  = malloc(data_size);
  data      = (KickMessage_data_t *)data_ptr;
  data->CmdKickRight = iniCmdKickRight;
  data->CmdKickCenter = iniCmdKickCenter;
  data->CmdKickLeft = iniCmdKickLeft;
  data->Intensity = iniIntensity;
  data->CmdResetCounter = iniCmdResetCounter;
  data->CmdActivateBallGuidance = iniCmdActivateBallGuidance;
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

/** Get CmdResetCounter value.
 * CmdResetCounter
 * @return CmdResetCounter value
 */
bool
KickerInterface::KickMessage::isCmdResetCounter()
{
  return data->CmdResetCounter;
}

/** Set CmdResetCounter value.
 * CmdResetCounter
 * @param newCmdResetCounter new CmdResetCounter value
 */
void
KickerInterface::KickMessage::setCmdResetCounter(bool newCmdResetCounter)
{
  data->CmdResetCounter = newCmdResetCounter;
}

/** Get CmdActivateBallGuidance value.
 * CmdActivateBallGuidance
 * @return CmdActivateBallGuidance value
 */
bool
KickerInterface::KickMessage::isCmdActivateBallGuidance()
{
  return data->CmdActivateBallGuidance;
}

/** Set CmdActivateBallGuidance value.
 * CmdActivateBallGuidance
 * @param newCmdActivateBallGuidance new CmdActivateBallGuidance value
 */
void
KickerInterface::KickMessage::setCmdActivateBallGuidance(bool newCmdActivateBallGuidance)
{
  data->CmdActivateBallGuidance = newCmdActivateBallGuidance;
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
  return false;
}

/// @cond INTERNALS
Interface *
private_newKickerInterface()
{
  return new KickerInterface();
}
/// @endcond
/** Create instance of KickerInterface.
 * @return instance of KickerInterface
 */
extern "C"
Interface *
newKickerInterface()
{
  return private_newKickerInterface();
}

/** Destroy KickerInterface instance.
 * @param interface KickerInterface instance to destroy.
 */
extern "C"
void
deleteKickerInterface(Interface *interface)
{
  delete interface;
}

