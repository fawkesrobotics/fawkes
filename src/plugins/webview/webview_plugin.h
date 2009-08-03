
/***************************************************************************
 *  webview_plugin.h - Fawkes Webview Plugin
 *
 *  Created: Mon Oct 13 17:46:03 2008 (I5 Developer's Day)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_WEBVIEW_PLUGIN_H_
#define __PLUGINS_WEBVIEW_WEBVIEW_PLUGIN_H_

#include <core/plugin.h>

class WebviewPlugin : public fawkes::Plugin
{
 public:
  WebviewPlugin(fawkes::Configuration *config);
};

#endif
