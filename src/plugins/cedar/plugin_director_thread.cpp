
/***************************************************************************
 *  plugin_director_thread.cpp - CEDAR plugin manager access
 *
 *  Created: Fri Dec 13 18:23:33 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "plugin_director_thread.h"

using namespace fawkes;

/** @class CedarPluginDirectorThread "plugin_director_thread.h"
 * Plugin manager access for CEDAR.
 * @author Tim Niemueller
 */

/** Constructor. */
CedarPluginDirectorThread::CedarPluginDirectorThread()
  : Thread("CedarPluginDirectorThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
CedarPluginDirectorThread::~CedarPluginDirectorThread()
{
}


void
CedarPluginDirectorThread::loop()
{
}
