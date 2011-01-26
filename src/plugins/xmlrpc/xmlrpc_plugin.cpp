
/***************************************************************************
 *  xmlrpc_plugin.h - Fawkes XmlRpc Plugin
 *
 *  Created: Sun Aug 30 12:41:36 2009 (NRW Council Election)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "xmlrpc_plugin.h"
#include "xmlrpc_thread.h"

using namespace fawkes;

/** @class XmlRpcPlugin "xmlrpc_plugin.h"
 * XmlRpc plugin for Fawkes.
 * This provides an extensible XmlRpc API for Fawkes.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
XmlRpcPlugin::XmlRpcPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new XmlRpcThread());

  /*
  bool custom_server = false;
  try {
    custom_server = config->get_bool("/xmlrpc/custom_server");
  } catch (Exception &e) {}

  if (! custom_server) add_dependency("webview");
  */
}


PLUGIN_DESCRIPTION("Provide XML-RPC API for Fawkes")
EXPORT_PLUGIN(XmlRpcPlugin)
