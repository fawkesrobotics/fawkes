
/***************************************************************************
 *  ballposlog_plugin.cpp - Fawkes ball position log plugin for demonstration
 *
 *  Created: Thu Jan 24 16:58:45 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/examples/ballposlog/plugin.h>
#include <plugins/examples/ballposlog/thread.h>

/** @class BallPosLogPlugin <plugins/examples/ballposlog/plugin.h>
 * Simple ball position logger plugin.
 * This plugin exists for demonstration purposes. It is part of the Fawkes
 * introductory talk on January 25th 2008 or AG RoboCup.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
BallPosLogPlugin::BallPosLogPlugin()
  : Plugin("ballposlog")
{
  thread_list.push_back(new BallPosLogThread());
}

EXPORT_PLUGIN(BallPosLogPlugin)
