
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
  builder->get_widget("swFuseList",  image_list_scroll_);
  builder->get_widget("vpImage",     image_viewport_);
  builder->get_widget("afSaveType",  save_box_);
  builder->get_widget("fcbSaveTo",   save_filechooser_);
  builder->get_widget("cbtAutoSave", auto_save_);
  builder->get_widget("btSaveImage", save_btn_);
  builder->get_widget("stb",         statusbar_);

  img_list_widget_ = Gtk::manage(new FuseImageListWidget());
  img_list_widget_->image_selected().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_fuse_image_selected) );
//  img_list_widget_->set_auto_update(true, 1);
  image_list_scroll_->add(*img_list_widget_);

  save_type_ = Gtk::manage(new Gtk::ComboBoxText);
  save_box_->add(*save_type_);

  std::vector<Gdk::PixbufFormat> fmts = Gdk::Pixbuf::get_formats();
  std::vector<Gdk::PixbufFormat>::const_iterator it = fmts.begin();
#if GTK_VERSION_GE(3,0)
  save_type_->append("Don't save");
#else
  save_type_->append_text("Don't save");
#endif
  for (; it != fmts.end(); ++it) {
    if ((*it).is_writable()) {
#if GTK_VERSION_GE(3,0)
      save_type_->append((*it).get_name());
#else
      save_type_->append_text((*it).get_name());
#endif
    }
  }

  save_type_->set_active(0);
  save_type_->set_sensitive(false);
  save_type_->signal_changed().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_type_change) );
  auto_save_->signal_toggled().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_auto_save_cbt_change) );
  save_btn_->signal_clicked().connect( sigc::mem_fun(*this, &FuseViewerGtkWindow::on_save_image_clicked) );
  show_all_children();

  cur_service_name_ = "";
  img_num_          = 0;
  img_widget_       = NULL;
  cam_              = NULL;

  set_status("");

  avahi_thread_ = new AvahiThread();
  avahi_dispatcher_ = new AvahiDispatcher;

  avahi_dispatcher_->signal_service_added().connect( sigc::mem_fun( *this, &FuseViewerGtkWindow::on_service_added ) );
  avahi_dispatcher_->signal_service_removed().connect( sigc::mem_fun( *this, &FuseViewerGtkWindow::on_service_removed ) );

  avahi_thread_->watch_service("_fountain._tcp", avahi_dispatcher_);
  avahi_thread_->start();
}

/** Destructor. */
FuseViewerGtkWindow::~FuseViewerGtkWindow()
{
  delete avahi_thread_;
  delete avahi_dispatcher_;
}

/** Signal handler called after AvahiThread detects a new NetworkService */
void
FuseViewerGtkWindow::on_service_added(fawkes::NetworkService* service)
{
  const char* name = service->name();
  const char* host = service->host();

  host_service_map_[host] = name;
  img_list_widget_->add_fountain_service(
      name,
      host,
      service->port());
}

/** Signal handler called after AvahiThread detects a NetworkService removal */
void
FuseViewerGtkWindow::on_service_removed( fawkes::NetworkService* service )
{
  img_list_widget_->remove_fountain_service( service->name() );

  if (cur_service_name_ == service->name()) {
    close_image();
  }

  std::map<std::string, std::string>::const_iterator it = host_service_map_.begin();
  for (; it != host_service_map_.end(); ++it) {
    if (cur_service_name_ == it->second) {
      host_service_map_.erase(it->first);
      break;
    }
  }
}

/** Signal handler that is called when an image is selected in the image list */
void
FuseViewerGtkWindow::on_fuse_image_selected()
{
  img_list_widget_->set_sensitive(false);
  std::string host;
  unsigned short port;
  std::string image_id;
  bool compression;

  img_list_widget_->get_selected_image(host, port, image_id, compression);

  close_image();

  try {
    cam_ = new NetworkCamera( host.c_str(), port, image_id.c_str(), compression );
    cam_->open();
    cam_->start();
    cur_service_name_ = host_service_map_[host];

    img_widget_ = new ImageWidget(cam_, 300);
    image_viewport_->add(*img_widget_);
    image_viewport_->set_size_request(cam_->pixel_width(), cam_->pixel_height());
    show_all_children();
    save_type_->set_sensitive(true);

    set_status(image_id, host, port);
  }
  catch (Exception& e) {
    cam_ = NULL;
    e.print_trace();
  }

  img_list_widget_->set_sensitive(true);
}

/** Signal handler that is called if the 'Auto save' checkbox status changes */
void
FuseViewerGtkWindow::on_auto_save_cbt_change()
{
  if (auto_save_->get_active()) {
    save_btn_->set_sensitive(false);

    img_widget_->save_on_refresh_cam(true,
        save_filechooser_->get_current_folder(),
        save_type_->get_active_text(),
        img_num_);
  }
  else {
    img_widget_->save_on_refresh_cam(false);
    img_num_ = img_widget_->get_image_num();

    save_btn_->set_sensitive(true);
  }
}

/** Signal handler that is called when the fileformat to save images changes */
void
FuseViewerGtkWindow::on_save_type_change()
{
  if (save_type_->get_active_row_number()) {
    auto_save_->set_sensitive(true);

    if (auto_save_->get_active()) img_num_ = img_widget_->get_image_num();
    on_auto_save_cbt_change();
  }
  else {
    auto_save_->set_active(false);
    auto_save_->set_sensitive(false);
    save_btn_->set_sensitive(false);
  }
}

/** Signal handler that is called when the 'Save image' button is pressed */
void
FuseViewerGtkWindow::on_save_image_clicked()
{
  char *ctmp;
  if (asprintf(&ctmp, "%s/%06u.%s", save_filechooser_->get_current_folder().c_str(),
	       ++img_num_, save_type_->get_active_text().c_str()) != -1) {
    Glib::ustring fn = ctmp;
    free(ctmp);

    img_widget_->save_image(fn, save_type_->get_active_text());
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
    statusbar_->push(Glib::ustring("Not connected."));
  }
  else {
    char *ctmp = NULL;
    if (asprintf(&ctmp, "Host: %s:%u\tId: %s",
                 host.c_str(), port, img_id.c_str())) {
      statusbar_->push(Glib::ustring(ctmp));
      free(ctmp);
    }
  }
}

/** Closes the image and the camera */
void
FuseViewerGtkWindow::close_image()
{
  if (img_widget_) {
    image_viewport_->remove();
    delete img_widget_;
    img_widget_ = NULL;
    save_type_->set_sensitive(false);
  }

  if (cam_) {
    cam_->stop();
    cam_->close();
    delete cam_;
    cam_ = NULL;
  }

  set_status("");
}

