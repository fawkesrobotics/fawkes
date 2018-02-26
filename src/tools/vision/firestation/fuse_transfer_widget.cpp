
/***************************************************************************
 *  fuse_transfer_widget.cpp - Fuse transfer widget
 *
 *  Created: Wed Mar 19 17:25:10 2008
 *  Copyright  2008  Daniel Beck
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

#include "fuse_transfer_widget.h"
#include "colormap_viewer_widget.h"

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/net/fuse_lutlist_content.h>

#include <fvmodels/color/lookuptable.h>

#include <netinet/in.h>
#include <cstring>

using namespace fawkes;
using namespace firevision;

/** @class FuseTransferWidget "fuse_transfer_widget.h"
 * This class implements the logic for a GUI that allows to transfer LUTs via FUSE.
 *
 * @author Daniel Beck
 */

/** Constructor. */
FuseTransferWidget::FuseTransferWidget()
{
  m_local_colormap_viewer  = new ColormapViewerWidget();
  m_remote_colormap_viewer = new ColormapViewerWidget();

  m_local_lut_list  = Gtk::ListStore::create(m_lut_record);
  m_remote_lut_list = Gtk::ListStore::create(m_lut_record);

  m_signal_update_local_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_local_lut_list) );
  m_signal_update_remote_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_remote_lut_list) );
  m_signal_get_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::get_lut_list) );
  m_signal_delete_client.connect( sigc::mem_fun( *this, &FuseTransferWidget::delete_clients) );
  m_signal_update_remote_lut.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_remote_lut) );

  m_new_clients.clear();
  m_delete_clients.clear();

  m_cur_client.active = false;

  m_btn_upload = 0;
  m_btn_download = 0;
  m_img_local = 0;
  m_img_remote = 0;
  m_trv_local_lut_list = 0;
  m_trv_remote_lut_list = 0;
}

/** Destructor. */
FuseTransferWidget::~FuseTransferWidget()
{
  delete m_local_colormap_viewer;
  delete m_remote_colormap_viewer;

  FuseClient* c;
  m_new_clients.lock();
  while (m_new_clients.size() != 0)
    {
      c = m_new_clients.front().client;
      m_new_clients.pop();
      c->disconnect();
      c->cancel();
      c->join();
      delete c;
    }
  m_new_clients.unlock();

  if (m_cur_client.active)
    {
      m_cur_client.active = false;
      m_delete_clients.push_locked(m_cur_client.client);
      delete_clients();
    }
}

/** Tell the widget that a new FUSE service has been discovered.
 * The widget will then attempt to connect to the host and list the available LUTs.
 * @param name the name of the service
 * @param host_name the name of the host the service is running on
 * @param port the port
 */
void
FuseTransferWidget::add_fountain_service( const char* name,
					  const char* host_name,
					  uint16_t port )
{
  ClientData data;
  data.client = 0;
  data.service_name = std::string(name);
  data.host_name = std::string(host_name);
  data.port = port;
  data.active = false;
  
  m_new_clients.push_locked(data);
  m_signal_get_lut_list();
}

/** Tell the widget that a service is not available any more.
 * All entries in the list of remote LUTs for the corresponding service will be deleted.
 * @param name the name of the service
 */
void
FuseTransferWidget::remove_fountain_service(const char* name)
{
  Gtk::TreeModel::Children children = m_remote_lut_list->children();
  Gtk::TreeModel::Children::iterator iter = children.begin();
  while( iter != children.end() )
    {
      Gtk::TreeModel::Row row = *iter;
      if (row[m_lut_record.service_name] == Glib::ustring(name))
	{
	  iter = m_local_lut_list->erase(iter);
	  m_local_lut_list->row_deleted( m_local_lut_list->get_path(iter) );
	}
      else
	{
	  ++iter;
	}
    }
}

/** Set the current colormap.
 * The current colormap is the local colormap that is currently trained.
 * @param colormap the colormap
 */
void
FuseTransferWidget::set_current_colormap(YuvColormap* colormap)
{
  m_current_colormap = colormap;

  // delete existing "Current" row
  Gtk::TreeModel::Children children = m_local_lut_list->children();
  Gtk::TreeModel::Children::iterator iter = children.begin();
  while ( iter != children.end() )
    {
      Gtk::TreeModel::Row row = *iter;
      if (row[m_lut_record.filename] == "Current")
	{
	  iter = m_local_lut_list->erase(iter);
	  m_local_lut_list->row_deleted( m_local_lut_list->get_path(iter) );
	}
      else
	{
	  ++iter;
	}
    }

  Gtk::TreeModel::Row row = *m_local_lut_list->prepend();
  row[m_lut_record.filename] = "Current";
  row[m_lut_record.width]    = colormap->width();
  row[m_lut_record.height]   = colormap->height();
  row[m_lut_record.depth]    = colormap->depth();
}

void
FuseTransferWidget::update_local_lut_list()
{
  if (m_trv_local_lut_list)
    { m_trv_local_lut_list->queue_draw(); }
}

void
FuseTransferWidget::update_remote_lut_list()
{
  if (m_trv_remote_lut_list)
    { m_trv_remote_lut_list->queue_draw(); }
}

/** Set the button to trigger the LUT upload.
 * @param btn the upload button
 */
void
FuseTransferWidget::set_upload_btn(Gtk::Button* btn)
{
  m_btn_upload = btn;
  m_btn_upload->signal_clicked().connect( sigc::mem_fun( *this, &FuseTransferWidget::upload_lut) );
}

/** Set the button to trigger the LUT download.
 * @param btn the download button
 */
void
FuseTransferWidget::set_download_btn(Gtk::Button* btn)
{
  m_btn_download = btn;
}

/** Set the Image to display the local LUT.
 * @param img the local LUT image
 */
void
FuseTransferWidget::set_local_img(Gtk::Image* img)
{
  m_img_local = img;
  m_local_colormap_viewer->set_colormap_img(m_img_local);
}

/** Assign a Scale to switch between the layers of the loal colormap.
 * @param scl a Gtk::Scale
 */
void
FuseTransferWidget::set_local_layer_selector(Gtk::Scale* scl)
{
  m_local_colormap_viewer->set_layer_selector(scl);
}

/** Set the Image to display the remote LUT.
 * @param img the remote LUT Image
 */
void
FuseTransferWidget::set_remote_img(Gtk::Image* img)
{
  m_img_remote = img;
  m_remote_colormap_viewer->set_colormap_img(m_img_remote);
}

/** Assign a Scale to switch between the layers of the remote colormap.
 * @param scl a Gtk::Scale
 */
void
FuseTransferWidget::set_remote_layer_selector(Gtk::Scale* scl)
{
  m_remote_colormap_viewer->set_layer_selector(scl);
}

/** Set the TreeView for the list of local LUTs.
 * @param trv the TreeView for the list of local LUTs
 */
void
FuseTransferWidget::set_local_lut_list_trv(Gtk::TreeView* trv)
{
  m_trv_local_lut_list = trv;
  m_trv_local_lut_list->set_model(m_local_lut_list);
  m_trv_local_lut_list->append_column("Filename", m_lut_record.filename);
  m_trv_local_lut_list->append_column("Width", m_lut_record.width);
  m_trv_local_lut_list->append_column("Height", m_lut_record.height);
  m_trv_local_lut_list->append_column("Depth", m_lut_record.depth);
  //  m_trv_local_lut_list->append_column("BPC", m_lut_record.bytes_per_cell);

  m_trv_local_lut_list->signal_cursor_changed().connect( sigc::mem_fun( *this, &FuseTransferWidget::local_lut_selected) );
}

/** Set the TreeView for the list of remote LUTs.
 * @param trv the TreeView for the list of remote LUTs
 */
void
FuseTransferWidget::set_remote_lut_list_trv(Gtk::TreeView* trv)
{
  m_trv_remote_lut_list = trv;
  m_trv_remote_lut_list->set_model(m_remote_lut_list);
  m_trv_remote_lut_list->append_column("Host", m_lut_record.host_name);
  //  m_trv_remote_lut_list->append_column("Port", m_lut_record.port);
  m_trv_remote_lut_list->append_column("ID", m_lut_record.lut_id);
  m_trv_remote_lut_list->append_column("Width", m_lut_record.width);
  m_trv_remote_lut_list->append_column("Height", m_lut_record.height);
  m_trv_remote_lut_list->append_column("Depth", m_lut_record.depth);
  m_trv_remote_lut_list->append_column("BPC", m_lut_record.bytes_per_cell);

  m_trv_remote_lut_list->signal_cursor_changed().connect( sigc::mem_fun( *this, &FuseTransferWidget::remote_lut_selected) );
}

void
FuseTransferWidget::get_lut_list()
{
  if (m_cur_client.active)
    // communication in progress
    { return; }

  m_new_clients.lock();
  if (m_new_clients.size() == 0)
    {
      m_new_clients.unlock();
      return;
    }

  m_cur_client = m_new_clients.front();
  m_cur_client.active = true;
  m_new_clients.pop();
  m_new_clients.unlock();

  try
    {
      m_cur_client.client = new FuseClient( m_cur_client.host_name.c_str(),
					    m_cur_client.port, this );
      m_cur_client.client->connect();
      m_cur_client.client->start();
      m_cur_client.client->enqueue(FUSE_MT_GET_LUT_LIST);
    }
  catch (Exception& e)
    {
      e.print_trace();
      m_cur_client.client->cancel();
      m_cur_client.client->join();
      delete m_cur_client.client;
      m_cur_client.active = false;
    }
}

void
FuseTransferWidget::delete_clients()
{
  FuseClient* c;

  m_delete_clients.lock();
  while (m_delete_clients.size() != 0)
    {
      c = m_delete_clients.front();
      m_delete_clients.pop();
      
      c->disconnect();
      c->cancel();
      c->join();
      delete c;
    }
  m_delete_clients.unlock();
}

void
FuseTransferWidget::update_local_lut()
{
  if ( !m_img_local )
    { return; }

  m_local_colormap_viewer->draw();
}

void
FuseTransferWidget::update_remote_lut()
{
  if ( !m_img_remote )
    { return; }

  m_remote_colormap_viewer->draw();
}

void
FuseTransferWidget::upload_lut()
{
  if ( !m_local_colormap )
    { return; }

  // get current selection remote
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_remote_lut_list->get_selection();
  
  if ( 1 != selection->count_selected_rows() )
    {
      printf("No remote lut selected\n");
      return;
    }
  
  Gtk::TreeModel::iterator i = selection->get_selected();
  Glib::ustring hostname = (*i)[m_lut_record.host_name];
  unsigned int port = (*i)[m_lut_record.port];
  Glib::ustring lut_id = (*i)[m_lut_record.lut_id];

  printf("sending lut to %s:%d id %s\n", hostname.c_str(), port, lut_id.c_str());

  FuseLutContent* lut_content = new FuseLutContent( lut_id.c_str(), 
						    m_local_colormap->get_buffer(),
						    m_local_colormap->width(),
						    m_local_colormap->height(),
						    m_local_colormap->depth(),
						    1 /* bytes per cell*/ );
  
  // create FUSE client
  FuseClient* client = new FuseClient(hostname.c_str(), port, this);

  try
    {
      client->connect();
      client->start();
      
      // send lut
      client->enqueue( new FuseNetworkMessage(FUSE_MT_SET_LUT, lut_content) );
      
      // mark FUSE client for deletion
      m_delete_clients.push_locked(client);
    }
  catch (Exception& e)
    {
      e.print_trace();
      client->cancel();
      client->join();
      delete client;
    }
}

void
FuseTransferWidget::local_lut_selected()
{
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_local_lut_list->get_selection();
  if (selection->count_selected_rows() != 1)
    { return; }

  Gtk::TreeModel::iterator it = selection->get_selected();
  Glib::ustring filename = (*it)[m_lut_record.filename];

  if (filename == "Current")
    {
      m_local_colormap = m_current_colormap;
    }
  else
    {
      // TODO
    }

  m_local_colormap_viewer->set_colormap(m_local_colormap);
  update_local_lut();
}

void
FuseTransferWidget::remote_lut_selected()
{
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_remote_lut_list->get_selection();
  if (selection->count_selected_rows() != 1)
    { return; }

  Gtk::TreeModel::iterator it = selection->get_selected();
  Glib::ustring host_name = (*it)[m_lut_record.host_name];
  unsigned int port       = (*it)[m_lut_record.port];
  Glib::ustring lut_id    = (*it)[m_lut_record.lut_id];

  FuseClient* c = new FuseClient(host_name.c_str(), port, this);
  try
    {
      c->connect();
      c->start();

      FUSE_lutdesc_message_t* lut_desc = (FUSE_lutdesc_message_t*) malloc( sizeof(FUSE_lutdesc_message_t));
      memset(lut_desc, 0, sizeof(FUSE_lutdesc_message_t));
      strncpy(lut_desc->lut_id, lut_id.c_str(), LUT_ID_MAX_LENGTH-1);
      c->enqueue(FUSE_MT_GET_LUT, lut_desc, sizeof(FUSE_lutdesc_message_t));

      m_delete_clients.push_locked(c);
    }
  catch (Exception& e)
    {
      e.print_trace();
      c->cancel();
      c->join();
      delete c;
    }
}

void
FuseTransferWidget::fuse_invalid_server_version(uint32_t local_version, 
						uint32_t remote_version) throw()
{
  printf("Invalid versions: local: %u   remote: %u\n", local_version, remote_version);
}

void
FuseTransferWidget::fuse_connection_established () throw()
{
}

void
FuseTransferWidget::fuse_connection_died() throw()
{
  if (m_cur_client.active)
    {
      m_delete_clients.push_locked(m_cur_client.client);
      m_cur_client.active = false;
    }

  m_signal_delete_client();
}

void
FuseTransferWidget::fuse_inbound_received (FuseNetworkMessage *m) throw()
{
  switch ( m->type() )
    {
    case FUSE_MT_LUT_LIST:
      try
	{
	  FuseLutListContent* content = m->msgc<FuseLutListContent>();
	  if ( content->has_next() )
	    {
	      while ( content->has_next() )
		{
		  // check whether there already is an entry for the given lut_id
		  FUSE_lutinfo_t* lut_info = content->next();
		  char lut_id[LUT_ID_MAX_LENGTH + 1];
		  lut_id[LUT_ID_MAX_LENGTH] = '\0';
		  strncpy(lut_id, lut_info->lut_id, LUT_ID_MAX_LENGTH);

		  Gtk::TreeModel::Children children = m_remote_lut_list->children();
		  Gtk::TreeModel::Children::iterator iter = children.begin();
		  while ( iter != children.end() )
		    {
		      Gtk::TreeModel::Row row = *iter;
		      if ( row[m_lut_record.lut_id] == Glib::ustring(lut_id) )
			{ iter = m_remote_lut_list->erase(iter); }
		      else
			{ ++iter; }
		    }
		  
		  Gtk::TreeModel::Row row = *m_remote_lut_list->append();
		  row[m_lut_record.service_name]   = Glib::ustring(m_cur_client.service_name);
		  row[m_lut_record.host_name]      = Glib::ustring(m_cur_client.host_name);
		  row[m_lut_record.port]           = m_cur_client.port;
		  row[m_lut_record.lut_id]         = Glib::ustring(lut_id);
		  row[m_lut_record.width]          = ntohl(lut_info->width);
		  row[m_lut_record.height]         = ntohl(lut_info->height);
		  row[m_lut_record.depth]          = ntohl(lut_info->depth);
		  row[m_lut_record.bytes_per_cell] = ntohl(lut_info->bytes_per_cell);
		}
	    }
	  delete content;
	}
      catch (Exception& e)
	{
	  e.print_trace();
	}
      
      m_delete_clients.push_locked(m_cur_client.client);
      m_cur_client.active = false;

      m_signal_update_remote_lut_list();
      m_signal_get_lut_list();
      m_signal_delete_client();

      break;

    case FUSE_MT_LUT:
      try
	{
	  FuseLutContent* lut_content = m->msgc<FuseLutContent>();
	  
	  if (m_remote_colormap)
	    { delete m_remote_colormap; }

	  if ( lut_content->width() != 256 ||
	       lut_content->height() != 256 )
	    {
	      m_signal_delete_client();
	      break;
	    }

	  m_remote_colormap = new YuvColormap( lut_content->depth() );
	  m_remote_colormap->set( lut_content->buffer() );

	  delete lut_content;
	}
      catch (Exception& e)
	{
	  e.print_trace();
	}
      m_remote_colormap_viewer->set_colormap(m_remote_colormap);
      m_signal_update_remote_lut();
      m_signal_delete_client();

      break;

    case FUSE_MT_SET_LUT_FAILED:
      printf("LUT upload failed\n");

    case FUSE_MT_SET_LUT_SUCCEEDED:
      printf("LUT upload succeeded\n");
      m_signal_delete_client();
      break;

    default:
      printf("Unhandled message type\n");
    }
}
