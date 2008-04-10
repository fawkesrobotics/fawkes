
/***************************************************************************
 *  fuse_transfer_widget.cpp - Fuse transfer widget
 *
 *  Created: Wed Mar 19 17:25:10 2008
 *  Copyright  2008  Daniel Beck
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

#include <tools/firestation/fuse_transfer_widget.h>
#include <tools/firestation/lut_viewer_widget.h>

#include <fvutils/net/fuse_client.h>
#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_lut_content.h>
#include <fvutils/net/fuse_lutlist_content.h>

#include <models/color/lookuptable.h>

#include <netinet/in.h>
#include <cstring>

/** @class FuseTransferWidget tools/firestation/fuse_transfer_widget.h
 * This class implements the logic for a GUI that allows to transfer LUTs via FUSE.
 *
 * @author Daniel Beck
 */

/** Constructor. */
FuseTransferWidget::FuseTransferWidget()
{
  m_local_lut_viewer = new LutViewerWidget();
  m_remote_lut_viewer = new LutViewerWidget();

  m_local_lut_list = Gtk::ListStore::create(m_lut_record);
  m_remote_lut_list = Gtk::ListStore::create(m_lut_record);

  m_signal_update_local_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_local_lut_list) );
  m_signal_update_remote_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_remote_lut_list) );
  m_signal_get_lut_list.connect( sigc::mem_fun( *this, &FuseTransferWidget::get_lut_list) );
  m_signal_delete_client.connect( sigc::mem_fun( *this, &FuseTransferWidget::delete_client) );
  m_signal_update_remote_lut.connect( sigc::mem_fun( *this, &FuseTransferWidget::update_remote_lut) );

  m_new_clients.clear();
  m_delete_clients.clear();

  m_cur_client.client = 0;

  m_cmb_remote_items = 0;
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
  delete m_local_lut_viewer;
  delete m_remote_lut_viewer;

  FuseClient* c;
  m_new_clients.lock();
  while (m_new_clients.size() != 0)
    {
      c = m_new_clients.front().client;
      m_new_clients.pop();
      c->cancel();
      c->join();
      delete c;
    }
  m_new_clients.unlock();

  if (m_cur_client.client)
    { 
      m_cur_client.client->cancel();
      m_cur_client.client->join();
      delete m_cur_client.client;
      m_cur_client.client = 0;
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
  data.client = new FuseClient(host_name, port, this);
  data.service_name = std::string(name);
  data.host_name = std::string(host_name);
  data.port = port;
  
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
  for (Gtk::TreeModel::Children::iterator iter = children.begin();
       iter != children.end(); ++iter)
    {
      Gtk::TreeModel::Row row = *iter;
      if (row[m_lut_record.service_name] == Glib::ustring(name))
	{
	  m_local_lut_list->erase(iter);
	  m_local_lut_list->row_deleted( m_local_lut_list->get_path(iter) );
	}
    }
}

/** Set the current LUT.
 * The current LUT is the local LUT that is currently trained.
 * @param lut the LUT
 */
void
FuseTransferWidget::set_current_lut(YuvColormap* lut)
{
  m_current_lut = lut;

  // delete existing "Current" row
  Gtk::TreeModel::Children children = m_local_lut_list->children();
  for (Gtk::TreeModel::Children::iterator iter = children.begin();
       iter != children.end(); ++iter)
    {
      Gtk::TreeModel::Row row = *iter;
      if (row[m_lut_record.filename] == "Current")
	{
	  m_local_lut_list->erase(iter);
	  m_local_lut_list->row_deleted( m_local_lut_list->get_path(iter) );
	}
    }

  Gtk::TreeModel::Row row = *m_local_lut_list->prepend();
  row[m_lut_record.filename] = "Current";

  // TODO
  //   row[m_lut_record.width] = width;
  //   row[m_lut_record.height] = height;
  //   row[m_lut_record.bytes_per_cell] = bpc;
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
  m_local_lut_viewer->set_lut_img(m_img_local);
}

/** Set the Image to display the remote LUT.
 * @param img the remote LUT Image
 */
void
FuseTransferWidget::set_remote_img(Gtk::Image* img)
{
  m_img_remote = img;
  m_remote_lut_viewer->set_lut_img(m_img_remote);
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
  m_trv_local_lut_list->append_column("BPC", m_lut_record.bytes_per_cell);

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
  m_trv_remote_lut_list->append_column("BPC", m_lut_record.bytes_per_cell);

  m_trv_remote_lut_list->signal_cursor_changed().connect( sigc::mem_fun( *this, &FuseTransferWidget::remote_lut_selected) );
}

void
FuseTransferWidget::get_lut_list()
{
  m_new_clients.lock();
  if (m_new_clients.size() == 0)
    {
      m_new_clients.unlock();
      return;
    }

  m_cur_client = m_new_clients.front();
  m_new_clients.pop();
  m_new_clients.unlock();

  m_cur_client.client->connect();
  m_cur_client.client->start();
  m_cur_client.client->enqueue(FUSE_MT_GET_LUT_LIST);
}

void
FuseTransferWidget::delete_client()
{
  FuseClient* c;

  m_delete_clients.lock();
  if (m_delete_clients.size() == 0)
    {
      m_delete_clients.unlock();
      return;
    }
  c = m_delete_clients.front();
  m_delete_clients.pop();
  m_delete_clients.unlock();

  c->cancel();
  c->join();
  delete c;
}

void
FuseTransferWidget::update_local_lut()
{
  if ( !m_img_local )
    { return; }

  m_local_lut_viewer->draw();
}

void
FuseTransferWidget::update_remote_lut()
{
  if ( !m_img_remote )
    { return; }

  m_remote_lut_viewer->draw();
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
      m_local_lut = m_current_lut;
    }
  else
    {
      // TODO
    }

  m_local_lut_viewer->set_colormap(m_local_lut);
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
  unsigned int port = (*it)[m_lut_record.port];
  Glib::ustring lut_id = (*it)[m_lut_record.lut_id];

  FuseClient* c = new FuseClient(host_name.c_str(), port, this);
  c->connect();
  c->start();

  FUSE_lutdesc_message_t* lut_desc = (FUSE_lutdesc_message_t*) malloc( sizeof(FUSE_lutdesc_message_t));
  memset(lut_desc, 0, sizeof(FUSE_lutdesc_message_t));
  strncpy(lut_desc->lut_id, lut_id.c_str(), LUT_ID_MAX_LENGTH);
  c->enqueue(FUSE_MT_GET_LUT, lut_desc, sizeof(FUSE_lutdesc_message_t));

  m_cur_client.client = c;
  m_cur_client.host_name = host_name.c_str();
  m_cur_client.port = port;
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
		  FUSE_lutinfo_t* lut_info = content->next();
		  char lut_id[LUT_ID_MAX_LENGTH + 1];
		  lut_id[LUT_ID_MAX_LENGTH] = '\0';
		  strncpy(lut_id, lut_info->lut_id, LUT_ID_MAX_LENGTH);
		  
		  Gtk::TreeModel::Row row = *m_remote_lut_list->append();
		  row[m_lut_record.service_name] = Glib::ustring(m_cur_client.service_name);
		  row[m_lut_record.host_name] = Glib::ustring(m_cur_client.host_name);
		  row[m_lut_record.port] = m_cur_client.port;
		  row[m_lut_record.lut_id] = Glib::ustring(lut_id);
		  row[m_lut_record.width] = ntohl(lut_info->width);
		  row[m_lut_record.height] = ntohl(lut_info->height);
		  row[m_lut_record.bytes_per_cell] = ntohl(lut_info->bytes_per_cell);
		}
	    }
	  delete content;
	}
      catch (Exception& e)
	{
	  e.print_trace();
	}
      m_signal_get_lut_list();

      m_signal_update_remote_lut_list();
      
      m_delete_clients.push_locked(m_cur_client.client);
      m_cur_client.client = 0;
      m_signal_delete_client();

      break;

    case FUSE_MT_LUT:
      try
	{
	  //FuseLutContent* lut_content = m->msgc<FuseLutContent>();
	  
	  if (m_remote_lut)
	    { delete m_remote_lut; }

	  //unsigned int depth = lut_content->depth();
	  m_remote_lut = new YuvColormap();
	}
      catch (Exception& e)
	{
	  e.print_trace();
	}
      m_remote_lut_viewer->set_colormap(m_remote_lut);
      m_signal_update_remote_lut();

      m_delete_clients.push_locked(m_cur_client.client);
      m_cur_client.client = 0;
      m_signal_delete_client();

      break;

    default:
      printf("Unhandled message type\n");
    }
}
