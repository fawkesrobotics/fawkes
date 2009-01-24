
/***************************************************************************
 *  fuse_viewer_gui.cpp -  Fuse (network camera) Viewer Gui
 *
 *  Created: Thu Dec 18 14:16:23 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
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

#include "fuse_viewer_gui.h"

#include <gui_utils/service_selector_cbe.h>
#include <netcomm/fawkes/client.h>
#include <plugin/net/messages.h>
#include <plugin/net/list_message.h>
#include <config/netconf.h>
#include <blackboard/remote.h>

#include <fvwidgets/fuse_image_list_widget.h>
#include <fvutils/system/camargp.h>
#include <cams/net.h>

#include <cstring>
#include <iomanip>
#include <sstream>

using namespace fawkes;

/** @class FuseViewerGtkWindow fuse_viewer_gui.h  <tools/fuse_viewer/fuse_viewer_gui.h>
 * Fawkes network camera viewer.
 * 
 * Currently the image refreshes 300ms after the retrieval and display of the 
 * last refresh (e.g. every 300ms in an ideal system)
 * The FUSE list doesn't get updated (due to a bug?), restarting the fvfountain
 * plugin on the remote host does the job.
 * 
 * @author Christof Rath
 */

/** Constructor.
 * @param cobject C base object
 * @param ref_xml Glade XML
 */
FuseViewerGtkWindow::FuseViewerGtkWindow(BaseObjectType* cobject,
				       const Glib::RefPtr<Gnome::Glade::Xml> ref_xml)
  : Gtk::Window(cobject),
    __conn_dispatcher(FAWKES_CID_PLUGINMANAGER)
{
  ref_xml->get_widget("swFuseList", m_image_list_scroll);
  ref_xml->get_widget("vpImage", m_image_viewport);
  ref_xml->get_widget("afSaveType", m_save_box);
  ref_xml->get_widget("fcbSaveTo", m_save_filechooser);
  ref_xml->get_widget("cbtAutoSave", m_auto_save);
  ref_xml->get_widget("btSaveImage", m_save_btn);
  ref_xml->get_widget("stb", m_stb);
  
  __service_selector = new ServiceSelectorCBE(ref_xml, "cbeHosts", "btnConnect", "wndMain");

  __conn_dispatcher.set_client(__service_selector->get_network_client());
  __conn_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &FuseViewerGtkWindow::on_connected));
  __conn_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &FuseViewerGtkWindow::on_disconnected));
  __conn_dispatcher.signal_message_received().connect(sigc::mem_fun(*this, &FuseViewerGtkWindow::on_message_received));

  __img_list_widget = Gtk::manage(new FuseImageListWidget());
  __img_list_widget->image_selected().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_fuse_image_selected) );
  m_image_list_scroll->add(*__img_list_widget);

  m_save_type = Gtk::manage(new Gtk::ComboBoxText);
  m_save_box->add(*m_save_type);
  
  Gdk::Pixbuf::SListHandle_PixbufFormat fmts = Gdk::Pixbuf::get_formats();
  Gdk::Pixbuf::SListHandle_PixbufFormat::const_iterator it = fmts.begin();
  m_save_type->append_text("Don't save");
  for (; it != fmts.end(); ++it) {
//    printf("type: %s writable: %s\n", (*it).get_name().c_str(), (*it).is_writable() ? "true" : "false");
    if ((*it).is_writable()) {
      m_save_type->append_text((*it).get_name());
    }
  }

  m_save_type->set_active(0);
  m_save_type->set_sensitive(false);
  m_save_type->signal_changed().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_type_change) );
  m_auto_save->signal_toggled().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_auto_save_cbt_change) );
  m_save_btn->signal_clicked().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_image_clicked) );
  show_all_children();

  __cfg_fountain_port   = 0;
  __img_num    = 0;
  __img_widget = NULL;
  __cam        = NULL;
  set_status(false, false);
}

/** Destructor. */
FuseViewerGtkWindow::~FuseViewerGtkWindow()
{
  on_disconnected();
  
  if ( __conn_dispatcher.get_client()->connected() )
  {
    // unsubscribe
    FawkesNetworkMessage* msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                                                         MSG_PLUGIN_UNSUBSCRIBE_WATCH);
    __conn_dispatcher.get_client()->enqueue(msg);
    msg->unref();                       

    __conn_dispatcher.get_client()->deregister_handler(FAWKES_CID_PLUGINMANAGER);
  }
}

/** Signal handler called after FawkesNetworkClient connection is established */
void
FuseViewerGtkWindow::on_connected()
{
  try {
    FawkesNetworkClient *client = __conn_dispatcher.get_client();

    //Read the config
    NetworkConfiguration* netconf = new NetworkConfiguration(client);
    netconf->set_mirror_mode(true);
    __cfg_fountain_port   = netconf->get_uint(FOUNTAIN_PORT_PATH);
    delete netconf;
    
    FawkesNetworkMessage* msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                                                         MSG_PLUGIN_SUBSCRIBE_WATCH);
    client->enqueue(msg);
    msg->unref();
      
    // request list of loaded plugins
    msg = new FawkesNetworkMessage(FAWKES_CID_PLUGINMANAGER,
                                   MSG_PLUGIN_LIST_LOADED);
    client->enqueue(msg);
    msg->unref();
    set_status(true, __has_fuse);
  }
  catch (Exception& e) {
    e.print_trace();
  }
}

/** Signal handler that is called whenever the connection is terminated. */
void
FuseViewerGtkWindow::on_disconnected()
{
  set_status(false, false);

  close_image();
  __img_list_widget->remove_fountain_service("test");
}

/**
 * Signal handler that is called whenever a new fawkes network message is received.
 * Used to determine the remote plugin status
 * @param msg the received message
 */
void
FuseViewerGtkWindow::on_message_received(fawkes::FawkesNetworkMessage* msg)
{
  if (msg->cid() != FAWKES_CID_PLUGINMANAGER)  return;

  // loading
  unsigned int msgid = msg->msgid();
  if ( (msgid == MSG_PLUGIN_LOADED) ||
       (msgid == MSG_PLUGIN_UNLOADED) )
  {
    Glib::ustring fuse = FUSE_PLUGIN_NAME;

    if ( msgid == MSG_PLUGIN_LOADED)
    {
      if ( msg->payload_size() != sizeof(plugin_loaded_msg_t) ) 
      {
        printf("Invalid message size (load succeeded)\n");
      } 
      else 
      {
        plugin_loaded_msg_t* m = (plugin_loaded_msg_t*) msg->payload();
        if (fuse == m->name) open_img_list();
      }
    }
    else if ( msg->msgid() == MSG_PLUGIN_UNLOADED ) 
    {
      if ( msg->payload_size() != sizeof(plugin_unloaded_msg_t) ) 
      {
        printf("Invalid message size (unload succeeded)\n");
      } 
      else 
      {
        plugin_unloaded_msg_t* m = (plugin_unloaded_msg_t*) msg->payload();
        if (fuse == m->name) {
          set_status(__is_connected, false);
          __img_list_widget->remove_fountain_service("test");
          close_image();
        }
      }
    }
  }
  else if (msg->msgid() == MSG_PLUGIN_LOADED_LIST ) 
  {
    Glib::ustring fuse = FUSE_PLUGIN_NAME;
    
    PluginListMessage* plm = msg->msgc<PluginListMessage>();
    while ( plm->has_next() ) 
    {
      char* name = plm->next();

      if ( fuse == name ) open_img_list();

      free(name);
    }
  }
  else if ( msg->msgid() == MSG_PLUGIN_LOADED_LIST_FAILED) 
  {
    printf("Obtaining list of loaded plugins failed\n");
  }

  // unknown message received
  else
  {
    printf("received message with msg-id %d\n", msg->msgid());
  }
}

/** Signal handler that is called when an image is selected in the image list */
void
FuseViewerGtkWindow::on_fuse_image_selected()
{
  std::string host_name;
  unsigned short port;
  std::string image_id;
  bool compression;

  __img_list_widget->get_selected_image(host_name, port, image_id, compression);

  close_image();
  
  try {
    __cam = new NetworkCamera( host_name.c_str(), port, image_id.c_str(), compression );
    __cam->open();
    __cam->start();
    __img_widget = new ImageWidget(__cam, 300);
    m_image_viewport->add(*__img_widget);
    m_image_viewport->set_size_request(__cam->pixel_width(), __cam->pixel_height());
    show_all_children();
    m_save_type->set_sensitive(Gtk::SENSITIVITY_ON);
  }
  catch (Exception& e) {
    __cam = NULL;
    e.print_trace();
  }
  
//  post_open_img_src();

}

/** Signal handler that is called if the 'Auto save' checkbox status changes */
void
FuseViewerGtkWindow::on_auto_save_cbt_change()
{
  if (m_auto_save->get_active()) {
    m_save_btn->set_sensitive(false);

    __img_widget->save_on_refresh_cam(true,
        m_save_filechooser->get_current_folder(),
        m_save_type->get_active_text(),
        __img_num);
  }
  else {
    __img_widget->save_on_refresh_cam(false);
    __img_num = __img_widget->get_image_num();

    m_save_btn->set_sensitive(true);
  }
}

/** Signal handler that is called when the fileformat to save images changes */
void
FuseViewerGtkWindow::on_save_type_change()
{
  if (m_save_type->get_active_row_number()) {
    m_auto_save->set_sensitive(true);

    if (m_auto_save->get_active()) __img_num = __img_widget->get_image_num();
    on_auto_save_cbt_change();
  }
  else {
    m_auto_save->set_active(false);
    m_auto_save->set_sensitive(false);
    m_save_btn->set_sensitive(false);
  }
}

/** Signal handler that is called when the 'Save image' button is pressed */
void
FuseViewerGtkWindow::on_save_image_clicked()
{
  char *ctmp;
  if (asprintf(&ctmp, "%s/%06u.%s", m_save_filechooser->get_current_folder().c_str(),
	       ++__img_num, m_save_type->get_active_text().c_str()) != -1) {
    Glib::ustring fn = ctmp;
    free(ctmp);

    __img_widget->save_image(fn, m_save_type->get_active_text());
  } else {
    printf("Could not save file, asprintf() ran out of memory");
  }
}

/** 
 * Activates the fuse image list. 
 * If or as soon as the fountain plugin is loaded on the remote side
 */
void
FuseViewerGtkWindow::open_img_list()
{
  set_status(__is_connected, true);
  __img_list_widget->add_fountain_service("test", __service_selector->get_hostname().c_str(), __cfg_fountain_port);
  //__img_list_widget->set_auto_update(true, 2);
}

/**
 * Sets the current status (to the statusbar)
 * @param conn TRUE if the connection is established
 * @param fuse TRUE if the fountain plugin is loaded on the remote host
 */
void
FuseViewerGtkWindow::set_status(bool conn, bool fuse)
{
  __is_connected = conn;
  __has_fuse     = fuse;

  char *ctmp;
  if (asprintf(&ctmp, "%s - %s",
	       __is_connected ? "connected" : "disconnected",
	       __has_fuse ? FUSE_PLUGIN_NAME" loaded" : FUSE_PLUGIN_NAME" NOT loaded") != -1) {

    m_stb->push(Glib::ustring(ctmp));
    free(ctmp);
  }
}

/** Closes the image and the camera */
void
FuseViewerGtkWindow::close_image()
{
  if (__img_widget) {
    m_image_viewport->remove();
    delete __img_widget;
    __img_widget = NULL;
    m_save_type->set_sensitive(Gtk::SENSITIVITY_OFF);
  }
  
  if (__cam) {
    __cam->stop();
    __cam->close();
    delete __cam;
    __cam = NULL;
  }
}

