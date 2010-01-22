
/***************************************************************************
 *  srsave_plugin.cpp - SwissRanger Save Plugin
 *
 *  Created: Fri Jan 22 10:47:29 2010
 *  Copyright  2007-2010  Tim Niemueller [www.niemueller.de]
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

#include "srsave_plugin.h"
#include "pipeline_thread.h"

using namespace fawkes;

/** @class FvSrSavePlugin "srsave_plugin.h"
 * SwissRanger Save Plugin.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FvSrSavePlugin::FvSrSavePlugin(fawkes::Configuration *config)
  : Plugin(config)
{
  thread_list.push_back( new FvSrSavePipelineThread() );
}

PLUGIN_DESCRIPTION("Write point files from SwissRanger data")
EXPORT_PLUGIN(FvSrSavePlugin)
