/***************************************************************************
 *  clips_pddl_parser_plugin.cpp - CLIPS plugin for parsing PDDL
 *
 *  Created: Fri 16 Feb 2018 17:44:08 CET 17:44
 *  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "clips_pddl_parser_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS PDDL Parser Plugin.
 *  Provides a CLIPS feature to the CLIPS environment that allows parsing PDDL
 *  domain files.
 *  @author Till Hofmann
 */

class ClipsPddlParserPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ClipsPddlParserPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new ClipsPddlParserThread());
  }
};

PLUGIN_DESCRIPTION("CLIPS feature to parse PDDL domains")
EXPORT_PLUGIN(ClipsPddlParserPlugin)
