
/***************************************************************************
 *  openprs_plugin.cpp - Plugin to use OpenPRS
 *
 *  Created: Thu Aug 14 15:48:22 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "openprs_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin to use OpenPRS from Fawkes.
 * This plugin integrates OpenPRS and provides an aspect for other plugins
 * to create OpenPRS kernels.
 * @author Tim Niemueller
 */
class OpenPRSPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenPRSPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new OpenPRSThread());
  }
};


PLUGIN_DESCRIPTION("OpenPRS integration kernel provider")
EXPORT_PLUGIN(OpenPRSPlugin)
