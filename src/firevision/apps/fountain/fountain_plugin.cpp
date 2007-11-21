
/***************************************************************************
 *  fountain_plugin.cpp - FireVision Fountain Plugin
 *
 *  Created: Fri Nov 16 11:29:48 2007 (Ella in lab for the first time)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <apps/fountain/fountain_plugin.h>
#include <apps/fountain/fountain_thread.h>

/** @class FvFountainPlugin <fountain_plugin.h>
 * FireVision Fountain Plugin.
 * This is the FireVision fountain plugin. It is a simple plugin that will
 * start a FuseServer that will then fulfill remote requests for internal
 * vision data.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FvFountainPlugin::FvFountainPlugin()
  : Plugin(Plugin::VISION, "fvfountain")
{
  thread_list.push_back(new FountainThread());
}

EXPORT_PLUGIN(FvFountainPlugin)
