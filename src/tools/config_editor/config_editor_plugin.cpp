
/***************************************************************************
 *  config_editor_plugin.cpp - Base class for config editor plugins
 *
 *  Created: Sun Mar 29 13:26:56 2009
 *  Copyright  2009  Daniel Beck
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

#include "config_editor_plugin.h"

using namespace std;
using namespace fawkes;

/** @class ConfigEditorPlugin tools/config_editor/config_editor_plugin.h
 * Base class for plugins for the Fawkes config editor. A plugin
 * allows to manipulate a certain part of the configuration, most
 * often this is intended to be the config options for a Fawkes
 * plugin.
 * @author Daniel Beck
 */

/** @fn void ConfigEditorPlugin::pre_run()
 * Config editor plugins need to implement this function. It is called
 * before the actual dialog is opened. Ususally, plugins want to parse
 * the configuration, here, and initialize the GUI elements of the
 * dialog.
 */

/** @fn void ConfigEditorPlugin::post_run( int response )
 * This method is called after the dialog is closed. Here, the input
 * the user has made needs to be handled.
 * @param response the response obtained from the run() method of the
 * dialog (Gtk::RESPONSE_OK or Gtk::RESPONSE_CANCEL)
 */

/** @fn Gtk::Dialog* ConfigEditorPlugin::load_dialog()
 * In this function the (custom) dialog of the plugin needs to be
 * initialized.
 * @return pointer to the loaded dialog
 */

/** @var ConfigEditorPlugin::m_dialog
 * The (main-) dialog of the plugin.
 */

/** @var ConfigEditorPlugin::m_ref_xml
 * Glade XML object created from the Glade file of the plugin.
 */

/** @var ConfigEditorPlugin:: m_config
 * The fawkes::Configuration.
 */

/** @var ConfigEditorPlugin::m_config_path
 * The config prefix the plugin is attached to.
 */

/** Constructor.
 * @param config_path the prefix of the part that can be configured
 * with this plugin
 * @param glade_file a Glade file which contains the definition the
 * plugin's GUI components
 */
ConfigEditorPlugin::ConfigEditorPlugin( string config_path, 
					string glade_file )
{
  m_config_path = config_path;
  m_ref_xml = Gnome::Glade::Xml::create( glade_file );
}

/** Destructor. */
ConfigEditorPlugin::~ConfigEditorPlugin()
{
}

/** Get the config prefix specified for this config editor plugin.
 * @return the config prefix
 */
std::string
ConfigEditorPlugin::get_config_path() const
{
  return m_config_path;
}

/** Set the configuration for the plugin to work on.
 * @param config the configuration
 */
void
ConfigEditorPlugin::set_config( fawkes::Configuration* config )
{
  m_config = config;
}

/** Initialize the plugin.
 * This method needs to be called before the plugin can be used.
 */
void
ConfigEditorPlugin::initialize()
{
  m_dialog = load_dialog();
}

/** Run the plugin.
 * Usually, this means opening a dialog where config values can be
 * manipulated and on closing these are written to the config.
 */
void
ConfigEditorPlugin::run()
{
  pre_run();
  int response = m_dialog->run();
  post_run( response );
  m_dialog->hide();
}
