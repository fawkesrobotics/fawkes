 
/***************************************************************************
 *  fuse_image_list_widget.h - Fuse image list widget
 *
 *  Created: Mon Mar 24 21:01:16 2008
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

#ifndef __FIREVISION_FVWIDGETS_FUSE_IMAGE_LIST_WIDGET_H_
#define __FIREVISION_FVWIDGETS_FUSE_IMAGE_LIST_WIDGET_H_

#include <fvutils/net/fuse_client_handler.h>
#include <fvutils/net/fuse_client.h>

#include <core/utils/lock_queue.h>
#include <core/utils/lock_list.h>
#include <core/threading/mutex.h>

#include <gtkmm.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseImageListWidget : FuseClientHandler, public Gtk::TreeView
{
 public:
  FuseImageListWidget();
  virtual ~FuseImageListWidget();

  void add_fountain_service( const char* name,
                             const char* host_name,
                             uint32_t port );
  void remove_fountain_service(const char* name);

  void set_toggle_compression_chk(Gtk::CheckButton* chk);
  void set_auto_update_chk(Gtk::CheckButton* chk);
  void set_image_list_trv(Gtk::TreeView* trv);
  Glib::Dispatcher& image_selected();

  bool auto_update();
  void set_auto_update(bool active, unsigned int interval_sec = 5);

  bool get_selected_image( std::string& host_name, unsigned short& port, 
                           std::string& image_id, bool& compression );

  // Fuse client handler
  void fuse_invalid_server_version( uint32_t local_version, 
                                    uint32_t remote_version ) throw();
  void fuse_connection_established() throw();
  void fuse_connection_died() throw();
  void fuse_inbound_received(FuseNetworkMessage *m) throw();

 private:
  /// @cond INTERNALS
  class ImageRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    ImageRecord()
    {
      add(display_text);
      add(service_name);
      add(host_name);
      add(port);
      add(colorspace);
      add(image_id);
      add(width);
      add(height);
      add(buffer_size);
    }

    Gtk::TreeModelColumn<Glib::ustring> display_text;
    Gtk::TreeModelColumn<Glib::ustring> service_name;
    Gtk::TreeModelColumn<Glib::ustring> host_name;
    Gtk::TreeModelColumn<unsigned int> port;
    Gtk::TreeModelColumn<Glib::ustring> image_id;
    Gtk::TreeModelColumn<unsigned int> colorspace;
    Gtk::TreeModelColumn<unsigned int> width;
    Gtk::TreeModelColumn<unsigned int> height;
    Gtk::TreeModelColumn<unsigned int> buffer_size;
  };

  struct ClientData
  {
    FuseClient* client;
    std::string service_name;
    std::string host_name;
    uint16_t port;
    bool active;
  };
  /// @endcond

  bool on_image_event(GdkEvent *event);
  void on_add_host_manually();
  void on_image_selected();
  void on_auto_update_toggled();
  void on_compression_toggled();
  void get_image_list();
  void delete_clients();
  void update_image_list();
  bool on_update_timeout();

  fawkes::LockList<ClientData> m_new_clients;
  fawkes::LockQueue<FuseClient*> m_delete_clients;

  ClientData m_cur_client;

#if GTK_VERSION_LT(3,0)
  Gtk::Menu* m_popup_menu;
#endif
  Gtk::CheckButton* m_chk_auto_update;
  Gtk::CheckButton* m_chk_compression;
  Gtk::TreeView* m_trv_image_list;
  Glib::RefPtr<Gtk::TreeStore> m_image_list;
  fawkes::Mutex m_img_list_mutex;

  ImageRecord m_image_record;

  Glib::Dispatcher m_signal_get_image_list;
  Glib::Dispatcher m_signal_delete_clients;
  Glib::Dispatcher m_signal_image_selected;
  Glib::Dispatcher m_signal_update_image_l;

  bool             m_auto_update;
  unsigned int     m_interval_sec;
  sigc::connection m_timeout_conn;

  Glib::ustring m_cur_image_id;
};

} // end namespace firevision

#endif /* __FIREVISION_FVWIDGETS_FUSE_IMAGE_LIST_WIDGET_H_ */
