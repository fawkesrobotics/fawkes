
/***************************************************************************
 *  fvretriever_plugin.cpp - FireVision Retriever Plugin
 *
 *  Created: Tue Jun 26 17:35:33 2007
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

#include <apps/retriever/fvretriever_plugin.h>
#include <apps/retriever/retriever_thread.h>

/** @class FvRetrieverPlugin <fvretriever_plugin.h>
 * FireVision Retriever Plugin.
 * This is the FireVision retriever plugin. It is a simple plugin that will
 * fetch images from a specific camera defined as a configuration setting.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
FvRetrieverPlugin::FvRetrieverPlugin()
  : Plugin("fvretriever")
{
  thread_list.push_back(new FvRetrieverThread());
}

EXPORT_PLUGIN(FvRetrieverPlugin)
