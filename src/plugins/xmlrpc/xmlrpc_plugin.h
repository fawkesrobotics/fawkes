
/***************************************************************************
 *  xmlrpc_plugin.h - Fawkes XmlRpc Plugin
 *
 *  Created: Sun Aug 30 12:43:17 2009 (NRW Council Election)
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

#ifndef __PLUGINS_XMLRPC_XMLRPC_PLUGIN_H_
#define __PLUGINS_XMLRPC_XMLRPC_PLUGIN_H_

#include <core/plugin.h>

class XmlRpcPlugin : public fawkes::Plugin
{
 public:
  XmlRpcPlugin(fawkes::Configuration *config);
};

#endif
