
/***************************************************************************
 *  plugin.cpp - Front Plugin
 *
 *  Created: Sun Dec 09 23:38:20 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <apps/front/plugin.h>
#include <apps/front/pipeline_thread.h>


/** @class FvFrontPlugin <apps/front/plugin.h>
 * Front Ball Plugin.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * Just adds the pipeline thread to the list of threads
 */
FvFrontPlugin::FvFrontPlugin()
  : Plugin("fvfront")
{
  thread_list.push_back( new FvFrontPipelineThread() );
}

EXPORT_PLUGIN(FvFrontPlugin)
