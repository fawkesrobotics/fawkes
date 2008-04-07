
/***************************************************************************
 *  kicker_control.h - Kicker Control
 *
 *  Generated: Tue May 15 13:29:57 2007
 *  Copyright  2007  Daniel Beck
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

#ifndef __PLUGINS_KICKER_KICKER_CONTROL_H_
#define __PLUGINS_KICKER_KICKER_CONTROL_H_

#include <iowkit.h>

class KickerControl
{
 public:
  KickerControl();
  ~KickerControl();

  bool open();
  void close();

  bool set_intensity(unsigned char intensity, bool force = false);

  bool kick(bool kick_right,
	    bool kick_center,
	    bool kick_left);
  
  bool kick_right(unsigned char intensity = 0xFF);
  bool kick_center(unsigned char intensity = 0xFF);
  bool kick_left(unsigned char intensity = 0xFF);

  bool guidance_right();
  bool guidance_left();

  bool is_guidance_right() const;
  bool is_guidance_left() const;

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

  unsigned int num_kicks[3];
  bool _guidance_right;
};


#endif /* __PLUGINS_KICKER_KICKER_CONTROL_H_ */
