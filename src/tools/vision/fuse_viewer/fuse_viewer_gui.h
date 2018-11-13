
/***************************************************************************
 *  fuse_viewer.h - Fuse (network camera) Viewer Gui
 *
 *  Created: Thu Dec 18 14:16:23 2008
 *  Copyright  2008-2009  Christof Rath <c.rath@student.tugraz.at>
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

#ifndef _FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_
#define _FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_

#define FUSE_PLUGIN_NAME      "fvfountain"
#define FOUNTAIN_PORT_PATH    "/firevision/fountain/tcp_port"

#include <netcomm/dns-sd/avahi_thread.h>

#include <map>

#include <gtkmm.h>

namespace firevision {
  class NetworkCamera;
  class FuseImageListWidget;
  class ImageWidget;
}
namespace fawkes {
  class AvahiDispatcher;
}

class FuseViewerGtkWindow : public Gtk::Window
{
public:
  FuseViewerGtkWindow(BaseObjectType* cobject,
                      const Glib::RefPtr<Gtk::Builder> builder);
  virtual ~FuseViewerGtkWindow();

private:
  void on_service_added(fawkes::NetworkService* service);
  void on_service_removed(fawkes::NetworkService* service);

  void on_fuse_image_selected();
  void on_auto_save_cbt_change();
  void on_save_type_change();
  void on_save_image_clicked();

  void close_image();
  void set_status(std::string img_id, std::string host = "", unsigned short port = 0);

private:
  // widgets
  Gtk::ScrolledWindow     *image_list_scroll_;
  Gtk::Viewport           *image_viewport_;
  Gtk::AspectFrame        *save_box_;
  Gtk::ComboBoxText       *save_type_;
  Gtk::FileChooserButton  *save_filechooser_;
  Gtk::CheckButton        *auto_save_;
  Gtk::Button             *save_btn_;
  Gtk::Statusbar          *statusbar_;

  fawkes::AvahiThread     *avahi_thread_;
  fawkes::AvahiDispatcher *avahi_dispatcher_;

  firevision::FuseImageListWidget     *img_list_widget_;

  firevision::ImageWidget             *img_widget_;
  firevision::NetworkCamera           *cam_;


  std::map<std::string, std::string> host_service_map_;

  std::string    cur_service_name_;
  unsigned int   img_num_;
};

#endif /* FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H__ */
