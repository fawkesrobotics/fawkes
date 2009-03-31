
/***************************************************************************
 *  config_editor_plugin.h - Base class for config editor plugins
 *
 *  Created: Sun Mar 29 11:52:30 2009
 *  Copyright  2009  Daniel Beck
 *
 *  $Id$
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

#ifndef __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_PLUGIN_H_
#define __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_PLUGIN_H_

#include <string>

#include <gtkmm.h>
#include <libglademm/xml.h>

namespace fawkes {
  class Configuration;
}

class ConfigEditorPlugin
{
 public:
  ConfigEditorPlugin( std::string config_path,
		      std::string glade_file );
  virtual ~ConfigEditorPlugin();

  void initialize();
  void run();

  std::string get_config_path() const;
  void set_config( fawkes::Configuration* config );


 protected:
  virtual void pre_run() =0;
  virtual void post_run( int response ) =0;

  virtual Gtk::Dialog* load_dialog() =0;

  Gtk::Dialog* m_dialog;
  Glib::RefPtr< Gnome::Glade::Xml > m_ref_xml;

  std::string            m_config_path;
  fawkes::Configuration* m_config;

};

#endif /* __TOOLS_CONFIG_EDITOR_CONFIG_EDITOR_PLUGIN_H_ */
