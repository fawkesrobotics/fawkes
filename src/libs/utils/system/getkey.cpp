/***************************************************************************
 *  getkey.cpp - getkey returns a keypress in non-blocking manner
 *
 *  Created: Thu Jun 04 19:08:13 2009 (from RCSoftX)
 *  Copyright  2009  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <utils/system/getkey.h>

namespace fawkes {

/** Set non-blocking flag on STDIN.
 * Sets the 0_NONBLOCK Flag to 1, so that the read command in the
 * getkey()-method wont block the programm till a input is made (see also
 * libc manual, pages 105 and 117).
 */
static void
set_nonblock_flag()  
{
  int oldflags;

  oldflags  = fcntl( STDIN_FILENO, F_GETFL, 0 );
  oldflags |= O_NONBLOCK;
  fcntl( STDIN_FILENO, F_SETFL, oldflags );
}


/** Clear non-blocking flag on STDIN. */
static void
clear_nonblock_flag()
{
  int oldflags;
  
  oldflags  = fcntl( STDIN_FILENO, F_GETFL, 0 );
  oldflags &= ~O_NONBLOCK; 
  fcntl( STDIN_FILENO, F_SETFL, oldflags );
}


/** Get value of a single key-press non-blocking.
 * This method checks if a new keypress has happened and returns the value in
 * this case. Otherwise it returns 0. The method does not block.
 * @return key pressed or 0
 */
char
getkey()
{
  char buf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  ssize_t n = 0;
  struct termios tattr,              // new terminal attributes
    saved_attributes;              // restore the original settings of the terminal
  
  set_nonblock_flag();
  tcgetattr( STDIN_FILENO, &saved_attributes );   // save the original attributes
  
  tcgetattr( STDIN_FILENO, &tattr );		    // set the new attributes for the terminal:
  tattr.c_lflag   &= ~(ICANON);		    // Clear ICANON
  tattr.c_lflag   &= ~(ECHO);		            // and ECHO
  tattr.c_cc[VMIN] = 0;                           // noncanonical, direction transmission
  tattr.c_cc[VTIME]= 0;                           // of input characters (MIN=0,TIME=0)
  tcsetattr( STDIN_FILENO, TCSANOW, &tattr );
  
  n = read( STDIN_FILENO, buf, 1 );
  
  tcsetattr( STDIN_FILENO, TCSANOW, &saved_attributes );
  clear_nonblock_flag();
  
  return buf[0];
}

} // end namespace fawkes

