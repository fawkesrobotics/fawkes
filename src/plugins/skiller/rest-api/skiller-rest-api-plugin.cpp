
/***************************************************************************
 *  skiller-rest-api-plugin.cpp - Behavior Engine REST API
 *
 *  Created: Sun Mar 25 01:27:11 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "skiller-rest-api.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS Executive REST API plugin.
 * @author Tim Niemueller
 */
class SkillerRestApiPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  SkillerRestApiPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new SkillerRestApi());
  }
};


PLUGIN_DESCRIPTION("Behavior Engine REST API")
EXPORT_PLUGIN(SkillerRestApiPlugin)
