
/***************************************************************************
 *  retriever_config_plugin.h - Config plugin for the retriever plugin
 *
 *  Created: Sun Mar 29 13:56:53 2009
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

#ifndef __TOOLS_CONFIG_EDITOR_RETRIEVER_CONFIG_PLUGIN_H_
#define __TOOLS_CONFIG_EDITOR_RETRIEVER_CONFIG_PLUGIN_H_

#include "config_editor_plugin.h"

#include <gtkmm.h>

#include <string>

class RetrieverConfigDialog : public Gtk::Dialog
{
 public:
  RetrieverConfigDialog(BaseObjectType* cobject,
                        const Glib::RefPtr<Gtk::Builder> &builder);
  virtual ~RetrieverConfigDialog();

  void add_camera( std::string camera_name,
		   std::string camera_string,
		   bool record_images = false,
		   std::string save_path = "" );

  std::map< std::string, std::string > get_cameras() const;

 private:
  class CameraRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    CameraRecord()
    {
      add( name );
      add( type );
      add( id );
      add( params );
      add( record_images );
      add( save_path );
    }
    
    Gtk::TreeModelColumn< Glib::ustring >  name;
    Gtk::TreeModelColumn< Glib::ustring >  type;
    Gtk::TreeModelColumn< Glib::ustring >  id;
    Gtk::TreeModelColumn< Glib::ustring >  params;
    Gtk::TreeModelColumn< bool >           record_images;
    Gtk::TreeModelColumn< Glib::ustring >  save_path;
  };

  // signal handler
  void on_add_clicked();
  void on_delete_clicked();

  CameraRecord m_camera_record;
  Glib::RefPtr< Gtk::ListStore > m_camera_list;
  Gtk::TreeView* m_trv_cameras;

  Gtk::Button* m_btn_add_camera;
  Gtk::Button* m_btn_delete_camera;
};

class RetrieverConfigPlugin : public ConfigEditorPlugin
{
 public:
  RetrieverConfigPlugin( std::string ui_path );
  virtual ~RetrieverConfigPlugin();

 protected:
  virtual void pre_run();
  virtual void post_run( int response );

  virtual Gtk::Dialog* load_dialog();

};

#endif /* __TOOLS_CONFIG_EDITOR_RETRIEVER_CONFIG_PLUGIN_H_ */
