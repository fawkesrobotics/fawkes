
/***************************************************************************
 *  plugin.cpp - FireVision Facer Plugin
 *
 *  Created: Sat Apr 19 12:36:55 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <apps/facer/plugin.h>
#include <apps/facer/pipeline_thread.h>

/** @class FvFacerPlugin <apps/facer/plugin.h>
 * FireVision Facer Plugin.
 * This is the FireVision facer plugin. It is used for face detection, training
 * and recognition.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FvFacerPlugin::FvFacerPlugin()
  : Plugin("fvfacer")
{
  thread_list.push_back(new FacerPipelineThread());
}

EXPORT_PLUGIN(FvFacerPlugin)
