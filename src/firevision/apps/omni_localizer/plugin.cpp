/*
    Copyright (c) 2007 Volker Krause <volker.krause@rwth-aachen.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#include <apps/omni_localizer/plugin.h>
#include <apps/omni_localizer/pipeline_thread.h>

/** @class FvOmniLocalizerPlugin <apps/omni_localizer/plugin.h>
 * Omnivision self-localization plugin.
 *
 * @author Volker Krause
 */

/** Constructor. */
FvOmniLocalizerPlugin::FvOmniLocalizerPlugin() :
    Plugin( "fvomnilocalizer" )
{
  thread_list.push_back( new FvOmniLocalizerPipelineThread() );
}

EXPORT_PLUGIN( FvOmniLocalizerPlugin )
