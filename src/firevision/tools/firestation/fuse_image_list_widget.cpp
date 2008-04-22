 
/***************************************************************************
 *  fuse_image_list_widget.cpp - Fuse image list widget
 *
 *  Created: Mon Mar 24 21:12:56 2008
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

#include <tools/firestation/fuse_image_list_widget.h>

#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_imagelist_content.h>

#include <netinet/in.h>
#include <cstring>

/** @class FuseImageListWidget <tools/firestation/fuse_image_list_widget.h>
 * This widget displays all available Fuse images in a tree view. It also can check
 * the registered host for new images, regularly.
 * @author Daniel Beck
 */

/** Constructor. */
FuseImageListWidget::FuseImageListWidget()
{
  m_cur_client.active = false;

  m_trv_image_list = 0;

  m_new_clients.clear();
  m_delete_clients.clear();

  m_image_list = Gtk::TreeStore::create(m_image_record);

  m_signal_get_image_list.connect( sigc::mem_fun( *this, &FuseImageListWidget::get_image_list) );
  m_signal_delete_clients.connect( sigc::mem_fun( *this, &FuseImageListWidget::delete_clients) );
}

/** Destructor. */
FuseImageListWidget::~FuseImageListWidget()
{
  FuseClient* c;
  m_new_clients.lock();
  while (m_new_clients.size() != 0)
    {
      c = m_new_clients.front().client;
      m_new_clients.pop_front();
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
    }
  delete_clients();
}

/** Call this method when new Fountain services are discovered.
 * @param name the name of the service
 * @param host_name the host the service is running on
 * @param port the port the service is running on
 */
void
FuseImageListWidget::add_fountain_service( const char* name,
					   const char* host_name,
					   uint32_t port )
{
  // check whether it's already in the tree
  m_img_list_mutex.lock();
  Gtk::TreeModel::Children children = m_image_list->children();
  for ( Gtk::TreeModel::Children::iterator iter = children.begin();
	iter != children.end(); ++iter )
    {
      Gtk::TreeModel::Row row = *iter;
      if ( row[m_image_record.service_name] == Glib::ustring(name) )
	{ 
	  m_img_list_mutex.unlock();
	  return; 
	}
    }
  m_img_list_mutex.unlock();

  // check if there is already a waiting request for this service
  m_new_clients.lock();
  for ( LockList<ClientData>::iterator iter = m_new_clients.begin();
	iter != m_new_clients.end(); ++iter )
    {
      if (name == iter->service_name)
	{ 
	  m_new_clients.unlock();
	  return;
	}
    }
  m_new_clients.unlock();

  ClientData data;
  data.client = 0;
  data.service_name = std::string(name);
  data.host_name = std::string(host_name);
  data.port = port;
  data.active = false;
  
  m_new_clients.push_back_locked(data);
  m_signal_get_image_list();
}

/** Call this method when a Fountain service vanishes.
 * @param name the name of the service
 */
void
FuseImageListWidget::remove_fountain_service(const char* name)
{
  m_img_list_mutex.lock();
  Gtk::TreeModel::Children children = m_image_list->children();
  Gtk::TreeModel::Children::iterator iter = children.begin();
  while ( iter != children.end() )
    {
      Gtk::TreeModel::Row row = *iter;
      if ( row[m_image_record.service_name] == Glib::ustring(name) )
	{
	  iter = m_image_list->erase(iter);
	  m_image_list->row_deleted( m_image_list->get_path(iter) );
 	}
      else
	{
	  ++iter;
	}
    }
  m_img_list_mutex.unlock();
}

/** Assign the TreeView widget to hold the list of images.
 * @param trv a Gtk::TreeView
 */
void
FuseImageListWidget::set_image_list_trv(Gtk::TreeView* trv)
{
  m_img_list_mutex.lock();
  m_trv_image_list = trv;
  m_trv_image_list->set_model(m_image_list);
  m_trv_image_list->append_column("asdf", m_image_record.display_text);
  m_trv_image_list->set_headers_visible(false);
  m_trv_image_list->signal_cursor_changed().connect( sigc::mem_fun(*this, &FuseImageListWidget::on_image_selected) );
  m_img_list_mutex.unlock();
}

/** Assign the CheckButton to toggle the compression.
 * @param chk a Gtk::CheckButton
 */
void
FuseImageListWidget::set_toggle_compression_chk(Gtk::CheckButton* chk)
{
  m_chk_compression = chk;
  m_chk_compression->signal_toggled().connect( sigc::mem_fun(*this, &FuseImageListWidget::on_compression_toggled) );
}

/** Assign the CheckButton that enables/disables the auto update function.
 * @param chk a Gtk::CheckButton
 */
void
FuseImageListWidget::set_auto_update_chk(Gtk::CheckButton* chk)
{
  m_chk_auto_update = chk;
  m_chk_auto_update->signal_toggled().connect( sigc::mem_fun(*this, &FuseImageListWidget::on_auto_update_toggled) );
}

/** Access the Dispatcher that is signalled when a new image is selected in the list of
 * images.
 * @return reference to the Dispatcher that is activated when an image is selected in the
 *         list of images
 */
Glib::Dispatcher&
FuseImageListWidget::image_selected()
{
  return m_signal_image_selected;
}

/** Get auto-update status.
 * @return true if auto-update is activated
 */
bool
FuseImageListWidget::auto_update()
{
  return m_auto_update;
}

/** Set the auto-update status.
 * @param active (de-)activate auto-update
 * @param interval_sec the update interval in seconds
 */
void
FuseImageListWidget::set_auto_update(bool active, unsigned int interval_sec)
{
  m_auto_update = active;

  if (m_auto_update)
    {
      sigc::connection conn = Glib::signal_timeout().connect( sigc::mem_fun(*this, &FuseImageListWidget::update_image_list),
							      interval_sec * 1000);
    }
}

/** Get the host name, port, and image id of the selected image.
 * @param host_name the host name of the selected image
 * @param port the port of the selected image
 * @param image_id the id of the selected image
 * @param compression true if compression shall be switched on
 * @return true if references could be assigned
 */
bool
FuseImageListWidget::get_selected_image( std::string& host_name, unsigned short& port,
					 std::string& image_id, bool& compression )
{
  if ( !m_trv_image_list )
    { return false; }

  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_image_list->get_selection();

  if ( selection->count_selected_rows() != 1 )
    { return false; }
  
  Gtk::TreeModel::iterator iter = selection->get_selected();
  host_name = iter->get_value(m_image_record.host_name);
  port      = iter->get_value(m_image_record.port);
  image_id  = iter->get_value(m_image_record.image_id);
  m_img_list_mutex.unlock();

  if (m_chk_compression)
    { compression = m_chk_compression->get_active(); }
  else
    { compression = false; }
  
  return true;
}

void
FuseImageListWidget::on_image_selected()
{
  m_img_list_mutex.lock();
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_image_list->get_selection();
  m_img_list_mutex.unlock();

  Gtk::TreeModel::iterator iter = selection->get_selected();
  Glib::ustring image_id;
  image_id = (*iter)[m_image_record.image_id];
  if (image_id != m_cur_image_id)
    { 
      m_cur_image_id = image_id;
      m_signal_image_selected();
    }
}

void
FuseImageListWidget::on_auto_update_toggled()
{
  set_auto_update( m_chk_auto_update->get_active() );
}

void
FuseImageListWidget::on_compression_toggled()
{
  m_signal_image_selected();
}

void
FuseImageListWidget::get_image_list()
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
  m_new_clients.pop_front();
  m_new_clients.unlock();

  try
    {
      m_cur_client.client = new FuseClient( m_cur_client.host_name.c_str(), 
					    m_cur_client.port, this );
      m_cur_client.client->connect();
      m_cur_client.client->start();
      m_cur_client.client->enqueue(FUSE_MT_GET_IMAGE_LIST);
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
FuseImageListWidget::delete_clients()
{
  FuseClient* c = 0;

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

bool
FuseImageListWidget::update_image_list()
{
  m_img_list_mutex.lock();
  Gtk::TreeModel::Children children = m_image_list->children();
  for ( Gtk::TreeModel::Children::iterator iter = children.begin();
	iter != children.end(); ++iter )
    {
      if ( (*iter)[m_image_record.image_id] == "invalid" )
	{
	  ClientData data;
	  data.client = 0;
	  Glib::ustring service_name = (*iter)[m_image_record.service_name];
	  Glib::ustring host_name = (*iter)[m_image_record.host_name];
	  data.service_name = std::string( service_name.c_str() );
	  data.host_name = std::string( host_name.c_str() );
	  data.port = (*iter)[m_image_record.port];
	  data.active = false;

	  m_new_clients.push_back_locked(data);
	}
    }
  m_img_list_mutex.unlock();

  m_signal_get_image_list();

  return m_auto_update;
}

void
FuseImageListWidget::fuse_invalid_server_version(uint32_t local_version, 
						uint32_t remote_version) throw()
{
  printf("Invalid versions: local: %u   remote: %u\n", local_version, remote_version);
}

void
FuseImageListWidget::fuse_connection_established () throw()
{
}

void
FuseImageListWidget::fuse_connection_died() throw()
{
  if (m_cur_client.active)
    {
      m_delete_clients.push_locked(m_cur_client.client);
      m_cur_client.active = false;
    }
  
  m_signal_delete_clients();
}

void
FuseImageListWidget::fuse_inbound_received (FuseNetworkMessage *m) throw()
{
  switch ( m->type() )
    {
    case FUSE_MT_IMAGE_LIST:
      {
	// check whether it's already in the tree
	m_img_list_mutex.lock();
	Gtk::TreeModel::Children children = m_image_list->children();
	Gtk::TreeModel::Children::iterator iter = children.begin();
	while ( iter != children.end() )
	  {
	    Gtk::TreeModel::Row row = *iter;
	    if ( row[m_image_record.service_name] == Glib::ustring(m_cur_client.service_name) )
	      {
		iter = m_image_list->erase(iter);
	      }
	    else
	      {
		++iter;
	      }
	  }

	try
	  {
	    FuseImageListContent* content = m->msgc<FuseImageListContent>();
	    if ( content->has_next() )
	      {
		Gtk::TreeModel::Row row = *m_image_list->append();
		row[m_image_record.display_text] = Glib::ustring(m_cur_client.host_name);
		row[m_image_record.service_name] = Glib::ustring(m_cur_client.service_name);
		row[m_image_record.host_name]    = Glib::ustring(m_cur_client.host_name);
		row[m_image_record.port]         = m_cur_client.port;
		row[m_image_record.colorspace]   = 0;
		row[m_image_record.image_id]     = "invalid";
		row[m_image_record.width]        = 0;
		row[m_image_record.height]       = 0;
		row[m_image_record.buffer_size]  = 0;
		
		Gtk::TreeModel::Path path = m_image_list->get_path(row);

		while ( content->has_next() )
		  {
		    FUSE_imageinfo_t* image_info = content->next();
		    char image_id[IMAGE_ID_MAX_LENGTH + 1];
		    image_id[IMAGE_ID_MAX_LENGTH] = '\0';
		    strncpy(image_id, image_info->image_id, IMAGE_ID_MAX_LENGTH);

		    Gtk::TreeModel::Row childrow = *m_image_list->append( row.children() );
		    childrow[m_image_record.display_text] = Glib::ustring(image_id);
		    childrow[m_image_record.service_name] = Glib::ustring(m_cur_client.service_name);
		    childrow[m_image_record.host_name]    = Glib::ustring(m_cur_client.host_name);
		    childrow[m_image_record.port]         = m_cur_client.port;
		    childrow[m_image_record.colorspace]   = ntohl(image_info->colorspace);
		    childrow[m_image_record.image_id]     = Glib::ustring(image_id);
		    childrow[m_image_record.width]        = ntohl(image_info->width);
		    childrow[m_image_record.height]       = ntohl(image_info->height);
		    childrow[m_image_record.buffer_size]  = ntohl(image_info->buffer_size);
		  }

		m_trv_image_list->expand_row(path, false);
	      }

	    delete content;
	  }
	catch (Exception& e)
	  {
	    e.print_trace();
	  }

	m_img_list_mutex.unlock();

	m_delete_clients.push_locked(m_cur_client.client);
	m_cur_client.active = false;

	m_signal_get_image_list();
	m_signal_delete_clients();
	
	break;
      }

    default:
      printf("Unhandled message type\n");
    }
}
