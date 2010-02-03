
/***************************************************************************
 *  retriever_config_plugin.cpp - Config plugin for the retriever plugin
 *
 *  Created: Sun Mar 29 13:59:28 2009
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

#include "retriever_config_plugin.h"

#include <config/config.h>
#include <core/exceptions/software.h>
#include <fvutils/system/camargp.h>

using namespace std;
using namespace fawkes;
using namespace firevision;

/** @class RetrieverConfigDialog "retriever_config_plugin.h"
 * Config dialog of the config editor plugin for the fvretriever.
 * @author Daniel Beck
 */

/** Constructor.
 * Allows to construct a dialog by means of get_widget_derived( ... ).
 * @param cobject base object pointer
 * @param ref_xml Glade XML object representing the Glade input file
 */
RetrieverConfigDialog::RetrieverConfigDialog( BaseObjectType* cobject,
					      const Glib::RefPtr< Gnome::Glade::Xml >& ref_xml )
  : Gtk::Dialog( cobject )
{
  ref_xml->get_widget("trvCameras", m_trv_cameras);
  ref_xml->get_widget("btnAdd", m_btn_add_camera);
  ref_xml->get_widget("btnDelete", m_btn_delete_camera);

  m_btn_add_camera->signal_clicked().connect( sigc::mem_fun( *this, &RetrieverConfigDialog::on_add_clicked ) );
  m_btn_delete_camera->signal_clicked().connect( sigc::mem_fun( *this, &RetrieverConfigDialog::on_delete_clicked ) );

  m_camera_list = Gtk::ListStore::create( m_camera_record );
  m_trv_cameras->set_model( m_camera_list );
  m_trv_cameras->append_column_editable( "Name", m_camera_record.name );
  m_trv_cameras->append_column_editable( "Type", m_camera_record.type );
  m_trv_cameras->append_column_editable( "Id", m_camera_record.id );
  m_trv_cameras->append_column_editable( "Parameter", m_camera_record.params );
  m_trv_cameras->append_column_editable( "Record", m_camera_record.record_images );
  m_trv_cameras->append_column_editable( "Save path", m_camera_record.save_path );
}

/** Destructor. */
RetrieverConfigDialog::~RetrieverConfigDialog()
{
}

/** Adds a camera to the list of cameras.
 * @param camera_name an arbitrary name to identify the camera
 * @param camera_string a camera string that can be parsed by a CameraArgumentParser
 * @param record_images if true the images of that camera are saved
 * @param save_path the directory where the images are saved
 */
void
RetrieverConfigDialog::add_camera( string camera_name,
				   string camera_string,
				   bool record_images,
				   string save_path )
{
  CameraArgumentParser* argp = new CameraArgumentParser( camera_string.c_str() );

  string cam_type = argp->cam_type();
  string cam_id = argp->cam_id();
  string params;
  std::map< string, string > param_map = argp->parameters();

  std::map< string, string >::iterator i = param_map.begin();
  while ( i != param_map.end() )
  {
    params += i->first + "=" + i->second;

    if ( ++i != param_map.end() )
    { params += ":"; }
  }

  Gtk::TreeModel::Row row = *m_camera_list->append();
  row[ m_camera_record.name ]          = camera_name;
  row[ m_camera_record.type ]          = cam_type;
  row[ m_camera_record.id ]            = cam_id;
  row[ m_camera_record.params ]        = params;
  row[ m_camera_record.record_images ] = record_images;
  row[ m_camera_record.save_path ]     = save_path;

  delete argp;
}

/** Obtain the list of cameras shown in the dialog.
 * @return a map camera name => camera string
 */
std::map< string, string >
RetrieverConfigDialog::get_cameras() const
{
  std::map< string, string > cameras;

  Gtk::TreeModel::Row row;
  Glib::ustring name;
  Glib::ustring cam_string;

  for ( Gtk::TreeIter i = m_camera_list->children().begin();
	i != m_camera_list->children().end();
	++i )
  {
    row = *i;
    name = row[ m_camera_record.name ];
    cam_string = row[ m_camera_record.type ] + ":" +
                 row[ m_camera_record.id ] + ":" +
                 row[ m_camera_record.params ];

    cameras[ name ] = cam_string;
  }

  return cameras;
}

void
RetrieverConfigDialog::on_add_clicked()
{
  // add empty row and select it
  Gtk::TreeIter iter = m_camera_list->append();
  Gtk::TreeModel::Row row = *iter;
  row[ m_camera_record.name ]          = "";
  row[ m_camera_record.type ]          = "";
  row[ m_camera_record.id ]            = "";
  row[ m_camera_record.params ]        = "";
  row[ m_camera_record.record_images ] = false;
  row[ m_camera_record.save_path ]     = "";

  m_trv_cameras->set_cursor( m_camera_list->get_path( iter ) );
}

void
RetrieverConfigDialog::on_delete_clicked()
{
  Gtk::TreeIter iter = m_trv_cameras->get_selection()->get_selected();
  m_camera_list->erase( iter );
}


/** @class RetrieverConfigPlugin tools/config_editor/retriever_config_plugin.h
 * Config editor plugin for the fvretriever plugin.
 * @author Daniel Beck
 */

/** Constructor.
 * @param glade_path path to the Glade file for the plugin's dialog
 */
RetrieverConfigPlugin::RetrieverConfigPlugin( string glade_path )
  : ConfigEditorPlugin( "/firevision/retriever", glade_path )
{
}

/** Destructor. */
RetrieverConfigPlugin::~RetrieverConfigPlugin()
{
}

void
RetrieverConfigPlugin::pre_run()
{
  string prefix = m_config_path + "/camera/";
  Configuration::ValueIterator* vit = m_config->search( prefix.c_str() );
  
  m_config->lock();
  while ( vit->next() )
  {
    if ( ! vit->is_string() )
    { 
      throw TypeMismatchException( "Only values of type string are valid for camera "
				   "argument string, but got %s for %s",
				   vit->type(),
				   vit->path() ); 
    }

    string camera_name = string( vit->path() ).substr( prefix.length() );
    
    RetrieverConfigDialog* dlg = dynamic_cast< RetrieverConfigDialog* >( m_dialog );
    dlg->add_camera( camera_name, vit->get_string() );
  }
  m_config->unlock();

  delete vit;
}

void
RetrieverConfigPlugin::post_run( int response )
{
  switch( response )
  {
  case ( Gtk::RESPONSE_OK ):
    {
      RetrieverConfigDialog* dlg = dynamic_cast< RetrieverConfigDialog* >( m_dialog );
      std::map< string, string > cameras = dlg->get_cameras();

      Glib::ustring path;
      
      for ( std::map< string, string >::iterator i = cameras.begin();
	    i != cameras.end();
	    ++i )
      {
	path = m_config_path + "/camera/" + i->first;
	m_config->set_string( path.c_str(), i->second );
      }

      break;
    }
  case ( Gtk::RESPONSE_CANCEL ):
    break;

  default:
    printf("unknonw response\n");
    break;
  }
}

Gtk::Dialog*
RetrieverConfigPlugin::load_dialog()
{
  RetrieverConfigDialog* dlg = NULL;
  m_ref_xml->get_widget_derived( "PluginDialog", dlg);

  return dlg;
}
