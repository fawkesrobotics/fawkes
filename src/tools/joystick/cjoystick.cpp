
/***************************************************************************
 *  cjoystick.cpp - Joystick interface
 *
 *  Generated: Sat Jun 02 17:31:27 2007
 *  Copyright  2007  Nils Springob <nils.springob@crazy-idea.de>
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */
 
#include <tools/joystick/cjoystick.h>
 
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <iostream>


using namespace std;

/** @class CJoystick <tools/joystick/cjoystick.h>
 *   Interface to a joystick.
 * 
 *  @author Nils Springob
 */
/** @var CJoystick::m_Buttons
 *      Contains the values of the buttons of the joystick.
 */
/** @var CJoystick::m_AxesInt
 *      Contains the values of the axes of the joystick.
 */
/** @var CJoystick::m_DevName
 *      The device name of the joystick.
 */
/** @var CJoystick::m_FD
 *      File descriptor of the joystick device.
 */
/** @var CJoystick::m_Error
 *      Error code for the joystick.
 */

/** Standard Constructor.
 *  Initializes joystick.
 *  @param dev_name device name of the joystick (defaults to /dev/js0)
 */
CJoystick::CJoystick (const std::string &dev_name)
{
  m_Error = 0;

  if((m_FD = open(dev_name.c_str(), O_RDONLY)) < 0)
    {
      cerr << " * CJoystick: Could not open joystick device '" << dev_name << "'!" << endl;
      m_Error = 1;
      throw "Error opening Joystick device";
    }

  fcntl( m_FD, F_SETFL, O_NONBLOCK );  /* use non-blocking mode */

  m_DevName = dev_name;

  char NumAxes = 2;
  char NumButtons = 2;

  ioctl(m_FD, JSIOCGAXES, &NumAxes);
  ioctl(m_FD, JSIOCGBUTTONS, &NumButtons);

  cout << " * CJoystick: found joystick with " << (int)NumAxes
       << " axes and " << (int)NumButtons << " buttons" << endl;

  m_Buttons.resize(NumButtons);
  m_AxesInt.resize(NumAxes);
}

/** Default Destructor. */
CJoystick::~CJoystick()
{
  close(m_FD);
}

/** Update current joystick state. */
int CJoystick::Update()
{
  struct js_event events[32];

  int num = read(m_FD, events, sizeof(events));

  if (num<0)
    return 0;

  num /= sizeof(events[0]);

  for (int i=0; i<num; i++)
    {
      switch(events[i].type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_BUTTON:

          if (events[i].number < m_Buttons.size())
            m_Buttons[events[i].number] = (events[i].value != 0);
          break;

        case JS_EVENT_AXIS:
          if (events[i].number < m_AxesInt.size())
            m_AxesInt[events[i].number] = events[i].value;
          break;
        }
    }
  return 0;
}

/** Returns an error.
 *  @return integer value
 */
int CJoystick::getError() const 
{
  return m_Error;
}

/** Returns the number of the axes of the joystick.
 *  @return number of axis
 */
unsigned int CJoystick::getNumAxes() const 
{
  return m_AxesInt.size();
}

/** Returns number of buttons of the joystick.
 *  @return number of buttons
 */
unsigned int CJoystick::getNumButtons() const 
{
  return m_Buttons.size();
}

/** Returns the name of the device.
 *  @return device name
 */
const std::string &CJoystick::getDeviceName() const
{
  return m_DevName;
}

/** Returns the axis value of the axis given by nr.
 *  @param nr number of the axis
 *  @return integer value of the axis with number nr 
 */
int CJoystick::getAxisInt(unsigned int nr) const
{
  if (nr<m_AxesInt.size()) return m_AxesInt[nr];
  throw "Invalid Joystick Axes index";
}
 
/** Returns the value of the axis given by nr,
 *   between -1 and 1.
 *   @param nr number of the axis
 *   @return float value between -1 and 1
 */
float CJoystick::getAxis(unsigned int nr) const
{
  return (float)getAxisInt(nr)/32767.0;
}

/** Returns -1 if nr is negative and 1 if nr is positive
 *      and 0 if nr is 0.
 *   @param nr number of the axis
 *   @return -1, 1 or 0
 */
int CJoystick::getAxisSig(unsigned int nr) const
{
  return (nr==0)? (0):((nr>0)? 1:-1);
}

/** Returns the x value for a given axis pair by poffset.
 *   @param poffset offset for the axis pair
 *   @return float value between -1 and 1
 */
float CJoystick::getX(unsigned int poffset) const 
{
  return getAxis(2*poffset+0);
}

/** Returns the y value for a given axis pair by poffset.
 *   @param poffset offset for the axis pair
 *   @return float value between -1 and 1
 */
float CJoystick::getY(unsigned int poffset) const
{
  return getAxis(2*poffset+1);
}

/** Returns true if the given button by nr is pressed
 *    and false if not.
 *   @param nr number of the axis
 *   @return true or false
 */
bool CJoystick::getButton(unsigned int nr) const 
{
  if (nr<m_Buttons.size()) return m_Buttons[nr];
  throw "Invalid Joystick Button index";
}

