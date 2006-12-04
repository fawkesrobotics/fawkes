
/***************************************************************************
 *  thread.cpp - Fawkes Example Plugin Thread
 *
 *  Generated: Wed Nov 22 17:13:57 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <plugins/example/thread.h>

/** @class ExampleThread plugins/example/thread.h
 * Thread of example plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param hook hook to register this thread for
 * @param name thread name
 * @param modc modulo count, every modc iterations a message is printed to stdout
 */
ExampleThread::ExampleThread(FawkesThread::WakeupHook hook, const char *name,
			     unsigned int modc)
{
  _hook = hook;
  _name = name;
  this->modc = modc;
  m = 0;
}


/** Destructor. */
ExampleThread::~ExampleThread()
{
}


/** Wakeup hook to register for.
 * @return hook when to wakeup this thread
 */
FawkesThread::WakeupHook
ExampleThread::hook() const
{
  return _hook;
}



/** Get name of thread.
 * @return name of thread.
 */
const char *
ExampleThread::name() const
{
  return _name;
}


/** Thread loop.
 * If num iterations module modc is 0 print out messaege, otherwise do nothing.
 */
void
ExampleThread::loop()
{
  if ( (m % modc) == 0 ) {
    printf("ExampleThread %s called %u times\n", _name, m);
  }
  ++m;
}
