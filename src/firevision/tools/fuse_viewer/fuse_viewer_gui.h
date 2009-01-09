
/***************************************************************************
 *  fuse_viewer.h - Fuse (network camera) Viewer Gui
 *
 *  Created: Thu Dec 18 14:16:23 2008
 *  Copyright  2007  Daniel Beck
 *             2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_
#define __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_

#define FUSE_PLUGIN_NAME      "fvfountain"
#define FOUNTAIN_PORT_PATH    "/firevision/fountain/tcp_port"

#include <gui_utils/connection_dispatcher.h>
#include <fvwidgets/image_widget.h>
#include <core/threading/thread.h>
#include <utils/math/types.h>


#include <gtkmm.h>
#include <libglademm/xml.h>

class NetworkCamera;
class FuseImageListWidget;

namespace fawkes {
  class ServiceSelectorCBE;
  class FawkesNetworkClient;
  class FawkesNetworkMessage;
}

class FuseViewerGtkWindow : public Gtk::Window
{
public:
  FuseViewerGtkWindow(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> ref_xml);
  virtual ~FuseViewerGtkWindow();

private:
  void on_connected();
  void on_disconnected();
  void on_message_received(fawkes::FawkesNetworkMessage *msg);
  void on_fuse_image_selected();
  void on_auto_save_cbt_change();
  void on_save_type_change();
  void on_save_image_clicked();

  void open_img_list();
  void close_image();
  void set_status(bool conn, bool fuse);
  
private:
  // widgets
  Gtk::ScrolledWindow    *m_image_list_scroll;
  Gtk::Viewport          *m_image_viewport;
  Gtk::AspectFrame       *m_save_box;
  Gtk::ComboBoxText      *m_save_type;
  Gtk::FileChooserButton *m_save_filechooser;
  Gtk::CheckButton       *m_auto_save;
  Gtk::Button            *m_save_btn;
  Gtk::Statusbar         *m_stb;

  fawkes::ServiceSelectorCBE      *__service_selector;
  fawkes::ConnectionDispatcher     __conn_dispatcher;

  FuseImageListWidget *__img_list_widget;

  ImageWidget   *__img_widget;
  NetworkCamera *__cam;

  unsigned int __cfg_fountain_port;
  bool         __is_connected;
  bool         __has_fuse;
  unsigned int __img_num;
};

#endif /* __FIREVISION_TOOLS_LOC_VIEWER_LOC_VIEWER_GUI_H_ */
