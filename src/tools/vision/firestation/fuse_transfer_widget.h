 
/***************************************************************************
 *  fuse_transfer_widget.h - Fuse transfer widget
 *
 *  Created: Wed Mar 19 17:11:01 2008
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_FUSE_TRANSFER_WIDGET_H_
#define __FIREVISION_TOOLS_FIRESTATION_FUSE_TRANSFER_WIDGET_H_

#include <fvutils/net/fuse_client_handler.h>
#include <core/utils/lock_queue.h>

#include <gtkmm.h>

namespace firevision {
  class FuseClient;
  class YuvColormap;
}
class ColormapViewerWidget;

class FuseTransferWidget : firevision::FuseClientHandler
{
 public:
  FuseTransferWidget();
  virtual ~FuseTransferWidget();
  
  void add_fountain_service( const char* name,
			     const char* host_name,
			     uint16_t port );
  void remove_fountain_service(const char* name);

  void set_current_colormap(firevision::YuvColormap* colormap);

  void set_upload_btn(Gtk::Button* btn_upload);
  void set_download_btn(Gtk::Button* btn_download);
  void set_local_img(Gtk::Image* img_local);
  void set_local_layer_selector(Gtk::Scale* scl);
  void set_remote_img(Gtk::Image* img_remote);
  void set_remote_layer_selector(Gtk::Scale* scl);
  void set_local_lut_list_trv(Gtk::TreeView* lut_list);
  void set_remote_lut_list_trv(Gtk::TreeView* lut_list);

  // Fuse client handler
  void fuse_invalid_server_version(uint32_t local_version, 
				   uint32_t remote_version) throw();
  void fuse_connection_established() throw();
  void fuse_connection_died() throw();
  void fuse_inbound_received(firevision::FuseNetworkMessage *m) throw();

 private:
  class LutRecord : public Gtk::TreeModelColumnRecord
    {
    public:
      LutRecord()
	{
	  add(filename);
	  add(service_name);
	  add(host_name);
	  add(port);
	  add(lut_id);
	  add(width);
	  add(height);
	  add(depth);
	  add(bytes_per_cell);
	  add(type);
	}
      
      /// @cond INTERNALS
      typedef enum
      {
	LUT_COLORMAP,
	LUT_MIRROR
      } LutType;

      Gtk::TreeModelColumn<Glib::ustring> filename;
      Gtk::TreeModelColumn<Glib::ustring> service_name;
      Gtk::TreeModelColumn<Glib::ustring> host_name;
      Gtk::TreeModelColumn<unsigned int> port;
      Gtk::TreeModelColumn<Glib::ustring> lut_id;
      Gtk::TreeModelColumn<unsigned int> width;
      Gtk::TreeModelColumn<unsigned int> height;
      Gtk::TreeModelColumn<unsigned int> depth;
      Gtk::TreeModelColumn<unsigned int> bytes_per_cell;
      Gtk::TreeModelColumn<LutRecord::LutType> type;
      /// @endcond
    };

  // signal handler
  void update_local_lut_list();
  void update_remote_lut_list();
  void get_lut_list();
  void delete_clients();
  void update_local_lut();
  void update_remote_lut();
  void upload_lut();

  void local_lut_selected();
  void remote_lut_selected();

  /// @cond INTERNALS
  struct ClientData
  {
    firevision::FuseClient* client;
    std::string service_name;
    std::string host_name;
    uint16_t port;
    bool active;
  };
  /// @endcond

  fawkes::LockQueue<ClientData> m_new_clients;
  fawkes::LockQueue<firevision::FuseClient*> m_delete_clients;

  ClientData m_cur_client;

  Glib::Dispatcher m_signal_update_local_lut_list;
  Glib::Dispatcher m_signal_update_remote_lut_list;
  Glib::Dispatcher m_signal_get_lut_list;
  Glib::Dispatcher m_signal_delete_client;
  Glib::Dispatcher m_signal_update_remote_lut;

  ColormapViewerWidget* m_local_colormap_viewer;
  ColormapViewerWidget* m_remote_colormap_viewer;

  Gtk::Button* m_btn_upload;
  Gtk::Button* m_btn_download;
  Gtk::Image* m_img_local;
  Gtk::Image* m_img_remote;
  Gtk::TreeView* m_trv_local_lut_list;
  Gtk::TreeView* m_trv_remote_lut_list;
  
  Glib::RefPtr<Gtk::ListStore> m_remote_lut_list;
  Glib::RefPtr<Gtk::ListStore> m_local_lut_list;
  LutRecord m_lut_record;

  firevision::YuvColormap* m_current_colormap;
  firevision::YuvColormap* m_local_colormap;
  firevision::YuvColormap* m_remote_colormap;
};

#endif /* __FIREVISION_TOOLS_FIRESTATION_FUSE_TRANSFER_WIDGET_H_ */
