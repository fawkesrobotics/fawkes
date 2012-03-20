
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

#ifndef __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_
#define __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_

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
  Gtk::ScrolledWindow     *__image_list_scroll;
  Gtk::Viewport           *__image_viewport;
  Gtk::AspectFrame        *__save_box;
  Gtk::ComboBoxText       *__save_type;
  Gtk::FileChooserButton  *__save_filechooser;
  Gtk::CheckButton        *__auto_save;
  Gtk::Button             *__save_btn;
  Gtk::Statusbar          *__statusbar;

  fawkes::AvahiThread     *__avahi_thread;
  fawkes::AvahiDispatcher *__avahi_dispatcher;

  firevision::FuseImageListWidget     *__img_list_widget;

  firevision::ImageWidget             *__img_widget;
  firevision::NetworkCamera           *__cam;


  std::map<std::string, std::string> __host_service_map;

  std::string    __cur_service_name;
  unsigned int   __img_num;
};

#endif /* __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_ */
