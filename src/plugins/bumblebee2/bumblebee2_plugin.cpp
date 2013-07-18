
/***************************************************************************
 *  bumblebee2_plugin.cpp - Acquire data from Bumblebee2 stereo camera
 *
 *  Created: Wed Jul 17 13:14:37 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
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

#include "bumblebee2_thread.h"

using namespace fawkes;

/** Plugin to segment a tabletop via PCL.
 * @author Tim Niemueller
 */
class Bumblebee2Plugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  Bumblebee2Plugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new Bumblebee2Thread());
  }
};

PLUGIN_DESCRIPTION("Data acquisition from Bumblebee2 stereo camera")
EXPORT_PLUGIN(Bumblebee2Plugin)
