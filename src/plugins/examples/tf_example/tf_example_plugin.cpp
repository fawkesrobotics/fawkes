
/***************************************************************************
 *  tf_example_plugin.cpp - tf example plugin
 *
 *  Created: Tue Oct 25 17:59:08 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include <core/plugin.h>

#include "tf_example_thread.h"

using namespace fawkes;

/** Plugin to publish static transforms.
 * @author Tim Niemueller
 */
class TfExamplePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  TfExamplePlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new TfExampleThread());
  }
};

PLUGIN_DESCRIPTION("TF example plugin")
EXPORT_PLUGIN(TfExamplePlugin)
