
/***************************************************************************
 *  fuse_transfer_widget.h - Fuse transfer widget
 *
 *  Created: Wed Mar 19 17:11:01 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
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
#include <map>
#include <string>

class FuseClient;
class YuvColormap;
class LutViewerWidget;

class FuseTransferWidget : FuseClientHandler
{
 public:
  FuseTransferWidget();
  virtual ~FuseTransferWidget();
  
  void add_fountain_service( const char* name,
			     const char* host_name,
			     uint16_t port );
  void remove_fountain_service(const char* name);

  void set_current_lut(YuvColormap* lut);

  void set_upload_btn(Gtk::Button* btn_upload);
  void set_download_btn(Gtk::Button* btn_download);
  void set_local_img(Gtk::Image* img_local);
  void set_remote_img(Gtk::Image* img_remote);
  void set_local_lut_list_trv(Gtk::TreeView* lut_list);
  void set_remote_lut_list_trv(Gtk::TreeView* lut_list);

  // Fuse client handler
  void fuse_invalid_server_version(uint32_t local_version, 
				   uint32_t remote_version) throw();
  void fuse_connection_established () throw();
  void fuse_inbound_received (FuseNetworkMessage *m) throw();

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
	  add(bytes_per_cell);
	}

      Gtk::TreeModelColumn<Glib::ustring> filename;
      Gtk::TreeModelColumn<Glib::ustring> service_name;
      Gtk::TreeModelColumn<Glib::ustring> host_name;
      Gtk::TreeModelColumn<unsigned int> port;
      Gtk::TreeModelColumn<Glib::ustring> lut_id;
      Gtk::TreeModelColumn<unsigned int> width;
      Gtk::TreeModelColumn<unsigned int> height;
      Gtk::TreeModelColumn<unsigned int> bytes_per_cell;
    };


  // signal handler
  void update_local_lut_list();
  void update_remote_lut_list();
  void get_lut_list();
  void delete_client();
  void update_local_lut();
  void update_remote_lut();

  void local_lut_selected();
  void remote_lut_selected();

  struct ClientData
  {
    FuseClient* client;
    std::string service_name;
    std::string host_name;
    uint16_t port;
  };

  LockQueue<ClientData> m_new_clients;
  LockQueue<FuseClient*> m_delete_clients;

  ClientData m_cur_client;

  Glib::Dispatcher m_signal_update_local_lut_list;
  Glib::Dispatcher m_signal_update_remote_lut_list;
  Glib::Dispatcher m_signal_get_lut_list;
  Glib::Dispatcher m_signal_delete_client;
  Glib::Dispatcher m_signal_update_remote_lut;

  LutViewerWidget* m_local_lut_viewer;
  LutViewerWidget* m_remote_lut_viewer;

  Gtk::ComboBox* m_cmb_remote_items;
  Gtk::Button* m_btn_upload;
  Gtk::Button* m_btn_download;
  Gtk::Image* m_img_local;
  Gtk::Image* m_img_remote;
  Gtk::TreeView* m_trv_local_lut_list;
  Gtk::TreeView* m_trv_remote_lut_list;
  
  Glib::RefPtr<Gtk::ListStore> m_remote_lut_list;
  Glib::RefPtr<Gtk::ListStore> m_local_lut_list;
  LutRecord m_lut_record;

  YuvColormap* m_current_lut;
  YuvColormap* m_local_lut;
  YuvColormap* m_remote_lut;
};

#endif /* __FIREVISION_TOOLS_IMAGE_VIEWER_MIRROR_CALIB_H_ */
