
/***************************************************************************
 *  fuse_image_list_widget.cpp - Fuse image list widget
 *
 *  Created: Mon Mar 24 21:12:56 2008
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

#include "fuse_image_list_widget.h"

#include <fvutils/net/fuse_message.h>
#include <fvutils/net/fuse_imagelist_content.h>

#include <netinet/in.h>
#include <cstring>
#include <sstream>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FuseImageListWidget <fvwidgets/fuse_image_list_widget.h>
 * This widget displays all available Fuse images in a tree view. It also can check
 * the registered host for new images, regularly.
 * @author Daniel Beck
 */

/** Constructor. */
FuseImageListWidget::FuseImageListWidget()
{
  m_chk_compression   = NULL;
  m_chk_auto_update   = NULL;

  m_cur_client.active = false;

  m_new_clients.clear();
  m_delete_clients.clear();

  m_image_list = Gtk::TreeStore::create(m_image_record);

  m_signal_get_image_list.connect( sigc::mem_fun( *this, &FuseImageListWidget::get_image_list) );
  m_signal_delete_clients.connect( sigc::mem_fun( *this, &FuseImageListWidget::delete_clients) );
  m_signal_update_image_l.connect( sigc::mem_fun( *this, &FuseImageListWidget::update_image_list) );

#if GTK_VERSION_LT(3,0)
  m_popup_menu = Gtk::manage( new Gtk::Menu() );
  Gtk::Menu::MenuList& menulist = m_popup_menu->items();
  menulist.push_back( Gtk::Menu_Helpers::MenuElem("Update now", sigc::mem_fun( *this, &FuseImageListWidget::update_image_list) ) );
  menulist.push_back( Gtk::Menu_Helpers::SeparatorElem() );
  menulist.push_back( Gtk::Menu_Helpers::MenuElem("Add host manually", sigc::mem_fun( *this, &FuseImageListWidget::on_add_host_manually) ) );
#endif

  set_image_list_trv(this);
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
  m_trv_image_list->signal_event().connect( sigc::mem_fun(*this, &FuseImageListWidget::on_image_event) );
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
  m_auto_update  = active;
  m_interval_sec = interval_sec;

  if (m_auto_update)
    {
#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 14 )
      m_timeout_conn = Glib::signal_timeout().connect_seconds( sigc::mem_fun(*this, &FuseImageListWidget::on_update_timeout),
                                                               m_interval_sec);
#else
      m_timeout_conn = Glib::signal_timeout().connect(sigc::mem_fun(*this, &FuseImageListWidget::on_update_timeout), m_interval_sec);
#endif
    }
  else m_timeout_conn.disconnect();
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

  m_img_list_mutex.lock();
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_image_list->get_selection();

  if ( selection->count_selected_rows() != 1 )
    {
      m_img_list_mutex.unlock();
      return false;
    }

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


bool
FuseImageListWidget::on_image_event(GdkEvent *event)
{
  GdkEventButton btn = event->button;
  if (btn.type == GDK_BUTTON_PRESS && btn.button == 3) {
#if GTK_VERSION_LT(3,0)
    m_popup_menu->popup(btn.button, btn.time);
#endif
    return true;
  }
  return false;
}

void
FuseImageListWidget::on_image_selected()
{
  m_img_list_mutex.lock();
  Glib::RefPtr<Gtk::TreeSelection> selection = m_trv_image_list->get_selection();

  Gtk::TreeModel::iterator iter = selection->get_selected();
  Glib::ustring image_id;
  image_id = (*iter)[m_image_record.image_id];
  m_img_list_mutex.unlock();

  if ((image_id != m_cur_image_id) && (image_id != "invalid"))
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
      if (m_auto_update)
        {
#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 14 )
          m_timeout_conn = Glib::signal_timeout().connect_seconds( sigc::mem_fun(*this, &FuseImageListWidget::on_update_timeout),
                                                                   m_interval_sec);
#else
          m_timeout_conn = Glib::signal_timeout().connect(sigc::mem_fun(*this, &FuseImageListWidget::on_update_timeout), m_interval_sec);
#endif
        }
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
FuseImageListWidget::on_update_timeout()
{
  m_signal_update_image_l();
  return m_auto_update;
}

void
FuseImageListWidget::update_image_list()
{
  m_timeout_conn.disconnect();
  if (m_img_list_mutex.try_lock())
    {
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
    }

  m_signal_get_image_list();
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

void
FuseImageListWidget::on_add_host_manually()
{
  Gtk::Dialog* add_host =
    new Gtk::Dialog("Add host manually", true);
  add_host->add_button(Gtk::Stock::ADD, Gtk::RESPONSE_OK);
  add_host->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);

  Gtk::Table* tab = Gtk::manage( new Gtk::Table(2, 2, false) );
  Gtk::Label* hlab = Gtk::manage( new Gtk::Label("Host:") );
  Gtk::Label* plab = Gtk::manage( new Gtk::Label("Port:") );
  Gtk::Entry* hent = Gtk::manage( new Gtk::Entry() );
  Gtk::HBox*  pbox = Gtk::manage( new Gtk::HBox() );

#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::Adjustment> prange = Gtk::Adjustment::create(2208, 1, 65535);
#else
  Gtk::Adjustment prange(2208, 1, 65535);
#endif
  Gtk::SpinButton *pent = Gtk::manage( new Gtk::SpinButton(prange) );

  char * fawkes_ip = getenv("FAWKES_IP");
  if (fawkes_ip) hent->set_text(std::string(fawkes_ip).append(":2208"));
  else hent->set_text("localhost:2208");

  pbox->pack_start(*pent, false, false, 0);
  tab->attach(*hlab, 1, 2, 1, 2);
  tab->attach(*plab, 1, 2, 2, 3);
  tab->attach(*hent, 2, 3, 1, 2);
  tab->attach(*pbox, 2, 3, 2, 3);

  add_host->get_vbox()->pack_start(*tab, false, true, 0);
  add_host->get_vbox()->show_all_children(true);

  if (add_host->run() == Gtk::RESPONSE_OK) {
    std::string name = "fountain on ";
    std::string host = hent->get_text();
    unsigned short port = 2208;

    Glib::ustring::size_type pos;
    if ((pos = host.find(':')) != Glib::ustring::npos)
    {
      Glib::ustring tmp_host = "";
      unsigned int tmp_port = 1234567; //Greater than max port num (i.e. 65535)
      std::istringstream is(host.replace(pos, 1, " "));
      is >> tmp_host;
      is >> tmp_port;

      if (tmp_port != 1234567 && tmp_host.size())
      {
        host = tmp_host;
        port = tmp_port;
      }
    }

    name.append(host);
    add_fountain_service(name.c_str(), host.c_str(), port);
  }

  add_host->hide();
  delete add_host;
}

} // end namespace firevision
