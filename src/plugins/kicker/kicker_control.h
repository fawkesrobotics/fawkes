
/***************************************************************************
 *  kicker_control.h - Kicker Control
 *
 *  Generated: Tue May 15 13:29:57 2007
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

#ifndef __PLUGINS_KICKER_KICKER_CONTROL_H_
#define __PLUGINS_KICKER_KICKER_CONTROL_H_

#include <iowkit.h>

class KickerControl
{
 public:
  KickerControl();
  ~KickerControl();

  bool set_intensity(unsigned char intensity, bool force = false);

  void kick(bool kick_right,
	    bool kick_center,
	    bool kick_left);
  
  void kick_right(unsigned char intensity = 0xFF);
  void kick_center(unsigned char intensity = 0xFF);
  void kick_left(unsigned char intensity = 0xFF);

  void get_num_kicks(unsigned int& right,
		     unsigned int& center,
		     unsigned int& left);
  
  unsigned get_num_kicks_total();

  void reset_counter();

 private:
  bool write(DWORD val);
 
  bool opened;
  IOWKIT_HANDLE ioHandle;
  DWORD intensity;

  unsigned int numKicks[3];
};


#endif /* __PLUGINS_KICKER_KICKER_CONTROL_H_ */
