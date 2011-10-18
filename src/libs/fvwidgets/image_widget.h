/***************************************************************************
 *  image_widget.h - Gtkmm widget to draw an image inside a Gtk::Window
 *
 *  Created: Wed Nov 26 00:00:00 2008
 *  Copyright 2008 Christof Rath <christof.rath@gmail.com>
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

#ifndef __FIREVISION_FVWIDGETS_IMAGE_WIDGET_H_
#define __FIREVISION_FVWIDGETS_IMAGE_WIDGET_H_

#include <core/threading/thread.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/rgb.h>

#include <gtkmm.h>
#ifdef HAVE_GLADEMM
#  include <libglademm/xml.h>
#endif

namespace fawkes {
  class Mutex;
}

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Camera;

class ImageWidget : public Gtk::Image
{
 private:
  class RefThread : public fawkes::Thread
  {
   public:
    RefThread(ImageWidget *widget, unsigned int refresh_delay);
    void set_delay(unsigned int refresh_delay);
    void save_on_refresh(bool enabled, std::string path = "", Glib::ustring type = "", unsigned int img_num = 0);
    void refresh_cam();
    void stop();
    unsigned int get_img_num();

   private:
    void loop();
    void perform_refresh();

    ImageWidget     *__widget;
    bool             __stop;
    bool             __do_refresh;
    unsigned int     __refresh_delay;
    unsigned int     __loop_cnt;
    Glib::Dispatcher __dispatcher;

    bool          __save_imgs;
    std::string   __save_path;
    Glib::ustring __save_type;
    unsigned int  __save_num;
  };

 public:
  ImageWidget(unsigned int width, unsigned int height);
  ImageWidget(Camera *cam, unsigned int refresh_delay = 0, unsigned int width = 0, unsigned int height = 0);
  ImageWidget(BaseObjectType* cobject, Glib::RefPtr<Gtk::Builder> builder);
#ifdef HAVE_GLADEMM
  ImageWidget(BaseObjectType* cobject, Glib::RefPtr<Gnome::Glade::Xml> refxml);
#endif
  virtual ~ImageWidget();

  void set_camera(Camera *cam, unsigned int refresh_delay = 0);
  void enable_camera(bool enable);
  void set_size(unsigned int width, unsigned int height);
  virtual bool show(colorspace_t colorspace, unsigned char *buffer, unsigned int width = 0, unsigned int height = 0);
  void set_refresh_delay(unsigned int refresh_delay);
  void refresh_cam();
  unsigned int get_width() const;
  unsigned int get_height() const;
  Glib::RefPtr<Gdk::Pixbuf> get_buffer() const;
  void set_rgb(unsigned int x, unsigned int y, unsigned char r, unsigned char g, unsigned char b);
  void set_rgb(unsigned int x, unsigned int y, RGB_t rgb);
  bool save_image(std::string filename, Glib::ustring type) const throw();
  void save_on_refresh_cam(bool enabled, std::string path = "", Glib::ustring type = "", unsigned int img_num = 0);
  unsigned int get_image_num();
  sigc::signal<void, colorspace_t, unsigned char *, unsigned int, unsigned int> & signal_show();

 private:
  void set_cam();

  unsigned int __width;
  unsigned int __height;

  Glib::RefPtr<Gdk::Pixbuf> __pixbuf;

  RefThread       *__refresh_thread;
  Camera          *__cam;
  fawkes::Mutex   *__cam_mutex;
  bool             __cam_has_buffer;
  bool             __cam_has_timestamp;
  bool             __cam_enabled;

  sigc::signal<void, colorspace_t, unsigned char *, unsigned int, unsigned int> __signal_show;
};

} // end namespace firevision

#endif /* __FIREVISION_FVWIDGETS_IMAGE_WIDGET_H_ */
