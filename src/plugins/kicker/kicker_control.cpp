
/***************************************************************************
 *  kicker_control.cpp - Kicker Control
 *
 *  Generated: Tue May 15 13:30:43 2007
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
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "kicker_control.h"

#include <iostream>
#include <unistd.h>

/** @class KickerControl plugins/kicker/kicker_control.h
 * Encapsultes the communication with the IOWarrior.
 * @author Daniel Beck
 */

using namespace std;

#define WAIT_BEFORE_RETRACT  100000

#define KICKER_RIGHT         0x00080000
#define KICKER_CENTER        0x00100000
#define KICKER_LEFT          0x00200000
#define INTENSITY_OFFSET     0x01000000

/** Constructor. */
KickerControl::KickerControl()
{
  opened = false;
}

/** Destructor. */
KickerControl::~KickerControl()
{
  close();
}

/** Opens the IOWarrior.
 * @returns true if device was opened succesfully
 */
bool
KickerControl::open()
{
  if (!opened)
    {
      ioHandle = IowKitOpenDevice();
      if (NULL == ioHandle)
	{
	  opened = false;
	  return false;
	}
      else
	{
	  opened = true;
	  set_intensity(0, true);
	  kick(false, false, false);
	  reset_counter();
	}
    }
  
  return true;
}

/** Closes the IOWarrior.
 * @returns true if device was closed succesfully
 */
void
KickerControl::close()
{
  if(opened)
    {
      set_intensity(0);
      kick(false, false, false);
      
      IowKitCloseDevice(ioHandle);
    }
}

/** Sets the intensity.
 * @param _intensity any value between 0 and 0xFF
 * @param force no message is sent to the IOWarrior if the specified intensity
 * equals the current intensity. By setting force to true it is enforced that a
 * new command is sent to the IOWarrior
 */
bool
KickerControl::set_intensity(unsigned char _intensity, bool force)
{
  if (opened)
    {
      if (intensity > 0xFF)
	intensity = 0xFF;
      
      if (intensity != _intensity || force)
	{
	  intensity = _intensity;
	  
	  DWORD val;
	  val = INTENSITY_OFFSET * intensity;
	  return write(val);
	}
      
      return true;
    }

  return false;
}

/** Triggers a kick.
 * @param kick_right if true the right kicker is triggered
 * @param kick_center if true the center/high kicker is triggered
 * @param kick_left if true the left kicker is triggered
 */
bool
KickerControl::kick(bool kick_right,
		    bool kick_center,
		    bool kick_left)
{
  DWORD val = 0;
  
  if (kick_right)
    {
      val += KICKER_RIGHT;
      numKicks[0]++;
    }
  
  if (kick_center)
    {
      val += KICKER_CENTER;
      numKicks[1]++;
    }
  
  if (kick_left)
    {
      val += KICKER_LEFT;
      numKicks[2]++;
    }
  
  val += INTENSITY_OFFSET * intensity;
  
  if (!write(val))
    return false;

  usleep(WAIT_BEFORE_RETRACT);

  if (!write(INTENSITY_OFFSET * intensity))
    return false;

  return true;
}

/** Trigger the right kicker.
 * @param _intensity adjusts to this intensity before kick is triggered
 */
bool
KickerControl::kick_right(unsigned char _intensity)
{
  set_intensity(_intensity);
  return kick(true, false, false);
}

/** Trigger the center kicker.
 * @param _intensity adjusts to this intensity before kick is triggered
 */
bool
KickerControl::kick_center(unsigned char _intensity)
{
  set_intensity(_intensity);
  return kick(false, true, false);
}

/** Trigger the left kicker.
 * @param _intensity adjusts to this intensity before kick is triggered
 */
bool
KickerControl::kick_left(unsigned char _intensity)
{
  set_intensity(_intensity);
  return kick(false, false, true);
}

/** Returns the number of kicks done with each kicker.
 * @param right reference to a variable where the result will be stored
 * @param center reference to a variable where the result will be stored
 * @param left reference to a variable where the result will be stored
 */
void
KickerControl::get_num_kicks(unsigned int& right,
			     unsigned int& center,
			     unsigned int& left)
{
  right = numKicks[0];
  center= numKicks[1];
  left = numKicks[2];
}

/** Returns the total number of kicks done with all kickers.
 * @returns total number of kicks
 */
unsigned int
KickerControl::get_num_kicks_total()
{
  unsigned int total = 0;

  for (unsigned int i = 0; i < 3; i++)
    {
      total += numKicks[i];
    }
  
  return total;
}

/** Resets the kick counter.
 */
void
KickerControl::reset_counter()
{
  for (unsigned int i = 0; i < 3; i++)
    numKicks[i] = 0;
}

/** Sends commands to the IOWarrior.
 * @param val the value that is sent to the IOWarrior
 */
bool
KickerControl::write(DWORD val)
{
  if (opened)
    {
      IOWKIT40_IO_REPORT report;
      ULONG res;
      
      report.ReportID = 0;
      report.Value = val;

      res = IowKitWrite(ioHandle, IOW_PIPE_IO_PINS,
			(char*)&report, IOWKIT40_IO_REPORT_SIZE);
      if (res != IOWKIT40_IO_REPORT_SIZE)
	{
	  /** Though it works flawlessly it seems that not as many bytes as expected
	      are written to the device. This needs further investigation!
	  */

	  /**
	  cout << "Expected to write " << IOWKIT40_IO_REPORT_SIZE << " bytes." << endl;
	  cout << "Actually wrote " << res << " bytes." << endl;
	  */

	  return false;
	}

      return true;
    }
  
  return false;
}
