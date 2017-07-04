
/***************************************************************************
 *  fuse_viewer_gui.cpp -  Fuse (network camera) Viewer Gui
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

#include "fuse_viewer_gui.h"

#include <gui_utils/avahi_dispatcher.h>
#include <core/exception.h>
#include <fvwidgets/fuse_image_list_widget.h>
#include <fvwidgets/image_widget.h>
#include <fvcams/net.h>

#include <cstring>

using namespace fawkes;
using namespace firevision;

/** @class FuseViewerGtkWindow "fuse_viewer_gui.h"
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
 * @param builder Gtk::Builder
 */
FuseViewerGtkWindow::FuseViewerGtkWindow(BaseObjectType* cobject,
					 const Glib::RefPtr<Gtk::Builder> builder)
  : Gtk::Window(cobject)
{
  builder->get_widget("swFuseList",  __image_list_scroll);
  builder->get_widget("vpImage",     __image_viewport);
  builder->get_widget("afSaveType",  __save_box);
  builder->get_widget("fcbSaveTo",   __save_filechooser);
  builder->get_widget("cbtAutoSave", __auto_save);
  builder->get_widget("btSaveImage", __save_btn);
  builder->get_widget("stb",         __statusbar);

  __img_list_widget = Gtk::manage(new FuseImageListWidget());
  __img_list_widget->image_selected().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_fuse_image_selected) );
//  __img_list_widget->set_auto_update(true, 1);
  __image_list_scroll->add(*__img_list_widget);

  __save_type = Gtk::manage(new Gtk::ComboBoxText);
  __save_box->add(*__save_type);

  std::vector<Gdk::PixbufFormat> fmts = Gdk::Pixbuf::get_formats();
  std::vector<Gdk::PixbufFormat>::const_iterator it = fmts.begin();
#if GTK_VERSION_GE(3,0)
  __save_type->append("Don't save");
#else
  __save_type->append_text("Don't save");
#endif
  for (; it != fmts.end(); ++it) {
    if ((*it).is_writable()) {
#if GTK_VERSION_GE(3,0)
      __save_type->append((*it).get_name());
#else
      __save_type->append_text((*it).get_name());
#endif
    }
  }

  __save_type->set_active(0);
  __save_type->set_sensitive(false);
  __save_type->signal_changed().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_type_change) );
  __auto_save->signal_toggled().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_auto_save_cbt_change) );
  __save_btn->signal_clicked().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_image_clicked) );
  show_all_children();

  __cur_service_name = "";
  __img_num          = 0;
  __img_widget       = NULL;
  __cam              = NULL;

  set_status("");

  __avahi_thread = new AvahiThread();
  __avahi_dispatcher = new AvahiDispatcher;

  __avahi_dispatcher->signal_service_added().connect( sigc::mem_fun( *this, &FuseViewerGtkWindow::on_service_added ) );
  __avahi_dispatcher->signal_service_removed().connect( sigc::mem_fun( *this, &FuseViewerGtkWindow::on_service_removed ) );

  __avahi_thread->watch_service("_fountain._tcp", __avahi_dispatcher);
  __avahi_thread->start();
}

/** Destructor. */
FuseViewerGtkWindow::~FuseViewerGtkWindow()
{
  delete __avahi_thread;
  delete __avahi_dispatcher;
}

/** Signal handler called after AvahiThread detects a new NetworkService */
void
FuseViewerGtkWindow::on_service_added(fawkes::NetworkService* service)
{
  const char* name = service->name();
  const char* host = service->host();

  __host_service_map[host] = name;
  __img_list_widget->add_fountain_service(
      name,
      host,
      service->port());
}

/** Signal handler called after AvahiThread detects a NetworkService removal */
void
FuseViewerGtkWindow::on_service_removed( fawkes::NetworkService* service )
{
  __img_list_widget->remove_fountain_service( service->name() );

  if (__cur_service_name == service->name()) {
    close_image();
  }

  std::map<std::string, std::string>::const_iterator it = __host_service_map.begin();
  for (; it != __host_service_map.end(); ++it) {
    if (__cur_service_name == it->second) {
      __host_service_map.erase(it->first);
      break;
    }
  }
}

/** Signal handler that is called when an image is selected in the image list */
void
FuseViewerGtkWindow::on_fuse_image_selected()
{
  __img_list_widget->set_sensitive(false);
  std::string host;
  unsigned short port;
  std::string image_id;
  bool compression;

  __img_list_widget->get_selected_image(host, port, image_id, compression);

  close_image();

  try {
    __cam = new NetworkCamera( host.c_str(), port, image_id.c_str(), compression );
    __cam->open();
    __cam->start();
    __cur_service_name = __host_service_map[host];

    __img_widget = new ImageWidget(__cam, 300);
    __image_viewport->add(*__img_widget);
    __image_viewport->set_size_request(__cam->pixel_width(), __cam->pixel_height());
    show_all_children();
    __save_type->set_sensitive(true);

    set_status(image_id, host, port);
  }
  catch (Exception& e) {
    __cam = NULL;
    e.print_trace();
  }

  __img_list_widget->set_sensitive(true);
}

/** Signal handler that is called if the 'Auto save' checkbox status changes */
void
FuseViewerGtkWindow::on_auto_save_cbt_change()
{
  if (__auto_save->get_active()) {
    __save_btn->set_sensitive(false);

    __img_widget->save_on_refresh_cam(true,
        __save_filechooser->get_current_folder(),
        __save_type->get_active_text(),
        __img_num);
  }
  else {
    __img_widget->save_on_refresh_cam(false);
    __img_num = __img_widget->get_image_num();

    __save_btn->set_sensitive(true);
  }
}

/** Signal handler that is called when the fileformat to save images changes */
void
FuseViewerGtkWindow::on_save_type_change()
{
  if (__save_type->get_active_row_number()) {
    __auto_save->set_sensitive(true);

    if (__auto_save->get_active()) __img_num = __img_widget->get_image_num();
    on_auto_save_cbt_change();
  }
  else {
    __auto_save->set_active(false);
    __auto_save->set_sensitive(false);
    __save_btn->set_sensitive(false);
  }
}

/** Signal handler that is called when the 'Save image' button is pressed */
void
FuseViewerGtkWindow::on_save_image_clicked()
{
  char *ctmp;
  if (asprintf(&ctmp, "%s/%06u.%s", __save_filechooser->get_current_folder().c_str(),
	       ++__img_num, __save_type->get_active_text().c_str()) != -1) {
    Glib::ustring fn = ctmp;
    free(ctmp);

    __img_widget->save_image(fn, __save_type->get_active_text());
  } else {
    printf("Could not save file, asprintf() ran out of memory");
  }
}

/**
 * Sets the current status (to the statusbar)
 * @param img_id the id of the current selected image
 * @param host the host that provides the image
 * @param port the port to transfer the image
 */
void
FuseViewerGtkWindow::set_status(std::string img_id, std::string host, unsigned short port)
{
  if (!img_id.length()) {
    __statusbar->push(Glib::ustring("Not connected."));
  }
  else {
    char *ctmp = NULL;
    if (asprintf(&ctmp, "Host: %s:%u\tId: %s",
                 host.c_str(), port, img_id.c_str())) {
      __statusbar->push(Glib::ustring(ctmp));
      free(ctmp);
    }
  }
}

/** Closes the image and the camera */
void
FuseViewerGtkWindow::close_image()
{
  if (__img_widget) {
    __image_viewport->remove();
    delete __img_widget;
    __img_widget = NULL;
    __save_type->set_sensitive(false);
  }

  if (__cam) {
    __cam->stop();
    __cam->close();
    delete __cam;
    __cam = NULL;
  }

  set_status("");
}

