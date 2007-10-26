
/***************************************************************************
 *  omni_ball_plugin.cpp - Omni Ball Plugin
 *
 *  Created: Thu July 05 19:00:19 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id: base_thread.cpp 344 2007-10-04 16:37:23Z tim $
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

#include <apps/omni_ball/omni_ball_plugin.h>
#include <apps/omni_ball/omni_ball_pipeline_thread.h>


/** @class FvOmniBallPlugin <apps/omni_ball/omni_ball_plugin.h>
 * OmniVision Ball Plugin.
 *
 * @author Daniel Beck
 */


/** Constructor.
 * Just adds the pipeline thread to the list of threads
 */
FvOmniBallPlugin::FvOmniBallPlugin()
  : Plugin(Plugin::VISION, "fvomniball")
{
  thread_list.push_back( new FvOmniBallPipelineThread() );
}

EXPORT_PLUGIN(FvOmniBallPlugin)
