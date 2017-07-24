
/***************************************************************************
 *  metrics_plugin.cpp - Metrics exporter plugin
 *
 *  Created: Sat May 06 19:41:18 2017 (German Open 2017)
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
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

#include "metrics_thread.h"

using namespace fawkes;

/** Plugin to export metrics for Prometheus.
 * @author Tim Niemueller
 */
class MetricsPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  MetricsPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new MetricsThread());
  }
};

PLUGIN_DESCRIPTION("Exports metrics for Prometheus")
EXPORT_PLUGIN(MetricsPlugin)
