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
#include <core/exception.h>
#include <cerrno>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>


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

  oldflags  = fcntl(STDIN_FILENO, F_GETFL, 0);
  oldflags |= O_NONBLOCK;
  fcntl(STDIN_FILENO, F_SETFL, oldflags);
}


/** Clear non-blocking flag on STDIN. */
static void
clear_nonblock_flag()
{
  int oldflags;
  
  oldflags  = fcntl(STDIN_FILENO, F_GETFL, 0);
  oldflags &= ~O_NONBLOCK; 
  fcntl(STDIN_FILENO, F_SETFL, oldflags);
}


/** Get value of a single key-press non-blocking.
 * This method checks if a new keypress has happened and returns the value in
 * this case. Otherwise it returns 0. The method does not block.
 * @param timeout_decisecs If less than 0 wait forever, if 0 non-blocking
 * (returns 0 if no key pressed immediately, if greater than 0 it is the
 * timeout in deciseconds.
 * @return key pressed or 0 (no key read)
 */
char
getkey(int timeout_decisecs)
{
  bool blocking = (timeout_decisecs != 0);
  char buf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  struct termios tattr,              // new terminal attributes
    saved_attributes;                // restore the original settings
  
  if (! blocking) set_nonblock_flag();
  tcgetattr(STDIN_FILENO, &saved_attributes);   // save the original attributes
  
  tcgetattr(STDIN_FILENO, &tattr);        // set the new attributes
  tattr.c_lflag   &= ~(ICANON);		  // Clear ICANON
  tattr.c_lflag   &= ~(ECHO);		  // and ECHO
  if (timeout_decisecs < 0) {
    tattr.c_cc[VMIN] = 1;                 // wait for one byte
    tattr.c_cc[VTIME]= 0;                 // no timeout
  } else if (timeout_decisecs > 0) {
    tattr.c_cc[VMIN] = 0;                 // do not wait for incoming bytes
    tattr.c_cc[VTIME]= timeout_decisecs;  // timeout
  } else {
    tattr.c_cc[VMIN] = 0;                 // do not wait for incoming bytes
    tattr.c_cc[VTIME]= 0;                 // no timeout
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &tattr);
  
  ssize_t read_bytes = read(STDIN_FILENO, buf, 1);
  
  tcsetattr(STDIN_FILENO, TCSANOW, &saved_attributes);
  if (! blocking) clear_nonblock_flag();
  
  if (read_bytes == 1) {
    return buf[0];
  } else {
    throw Exception(errno, "Failed to read key from keyboard (getkey)");
  }
}

} // end namespace fawkes

