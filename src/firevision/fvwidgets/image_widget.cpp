/***************************************************************************
 *  image_widget.cpp - Gtkmm widget to draw an image inside a Gtk::Window
 *
 *  Created:  26.11.2008
 *  Copyright 2008 Christof Rath <christof.rath@gmail.com>
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


#include "image_widget.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>
#include <cams/camera.h>

#include <iomanip>


/** @class ImageWidget <fvwidgets/image_widget.h>
 * This class is an image container to display fawkes cameras inside a Gtk::Window
 * 
 * @author Christof Rath
 */

/**
 * Creates a new ImageWidget with predefined width and height
 * @param width of the widget
 * @param height of the widget
 */
ImageWidget::ImageWidget(unsigned int width, unsigned int height)
{
  __width  = width;
  __height = height;
  
  __cam            = NULL;
  __cam_mutex      = NULL;
  __refresh_thread = NULL;

  __pixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8, __width, __height);

  set_size_request(__width, __height);
}

/**
 * Creates a new ImageWidget with a Camera as image source
 * @param cam the image source
 * @param refresh_delay if greater 0 a thread gets created that refreshes the Image every refresh_delay milliseconds
 */
ImageWidget::ImageWidget(Camera *cam, unsigned int refresh_delay)
{
  if (!cam) throw fawkes::NullPointerException("Parameter cam may not be NULL");

  __cam            = cam;
  __cam_mutex      = new fawkes::Mutex;
  __cam_has_buffer = false;
  __width          = __cam->pixel_width();
  __height         = __cam->pixel_height();

  __refresh_thread = new RefThread(this, refresh_delay);

  __pixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8, __width, __height);

  __refresh_thread->start();
  __refresh_thread->refresh_cam();
 
  if (refresh_delay) set_refresh_delay(refresh_delay);

  set_size_request(__width, __height);
}

/**
 * Destructor
 */
ImageWidget::~ImageWidget()
{
  __refresh_thread->stop();
  delete __cam_mutex;
}

/**
 * Returns the image buffer width
 * @return width of the contained image
 */
unsigned int
ImageWidget::get_width() const
{
    return __width;
}

/**
 * Returns the image buffer height
 * @return height of the contained image
 */
unsigned int
ImageWidget::get_height() const
{
    return __height;
}

/**
 * Returns the widgets pixel buffer (RGB!)
 * @return the RGB pixel buffer
 */
Glib::RefPtr<Gdk::Pixbuf>
ImageWidget::get_buffer() const
{
    return __pixbuf;
}

/**
 * Sets a pixel to the given RGB colors
 * 
 * @param x position of the pixel
 * @param y position of the pixel
 * @param r component of the color
 * @param g component of the color
 * @param b component of the color
 */
void
ImageWidget::set_rgb(unsigned int x, unsigned int y, unsigned char r, unsigned char g, unsigned char b)
{
  set_rgb (x, y, (RGB_t){r, g, b});
}

/**
 * Sets a pixel to the given RGB colors
 * 
 * @param x position of the pixel
 * @param y position of the pixel
 * @param rgb the color
 */
void
ImageWidget::set_rgb(unsigned int x, unsigned int y, RGB_t rgb)
{
  if (x >= __width) throw fawkes::OutOfBoundsException("x-Coordinate exeeds image width", x, 0, __width);
  if (y >= __height) throw fawkes::OutOfBoundsException("y-Coordinate exeeds image height", x, 0, __height);

  RGB_t * target = RGB_PIXEL_AT(__pixbuf->get_pixels(), __width, x, y);
  *target = rgb;
}

/** 
 * Show image from given colorspace.
 * 
 * @param colorspace colorspace of the supplied buffer
 * @param buffer image buffer
 */
void
ImageWidget::show(colorspace_t colorspace, unsigned char *buffer)
{
  try {
    convert(colorspace, RGB, buffer, __pixbuf->get_pixels(), __width, __height);
    set(__pixbuf);
  }
  catch (fawkes::Exception &e) {
    printf("ImageWidget::show(): Could not set the new image (%s)\n", e.what());
  }
}


/**
 * Sets the refresh delay for automatic camera refreshes
 * 
 * @param refresh_delay im [ms]
 */
void
ImageWidget::set_refresh_delay(unsigned int refresh_delay)
{
  __refresh_thread->set_delay(refresh_delay);
}


/**
 * Performs a refresh during the next loop of the refresh thread
 */
void
ImageWidget::refresh_cam()
{
  __refresh_thread->refresh_cam(); 
}

/**
 * Sets the widgets pixbuf after! (non blocking) retrieving the image over the network
 */
void
ImageWidget::set_cam()
{
  __cam_mutex->lock();

  if (__cam_has_buffer) {
    show(__cam->colorspace(), __cam->buffer());
    __cam->flush();
    __cam_has_buffer = false;
  }

  __cam_mutex->unlock();
}

/**
 * Saves the current content of the Image
 * @param filename of the output
 * @param type of the output (By default, "jpeg", "png", "ico" and "bmp" are possible file formats to save in, but more formats may be installed. TThe list of all writable formats can be determined by using Gdk::Pixbuf::get_formats() with Gdk::PixbufFormat::is_writable().)
 */
bool
ImageWidget::save_image(std::string filename, Glib::ustring type) const throw()
{
  __cam_mutex->lock();

  try {
    __pixbuf->save(filename, type);
    __cam_mutex->unlock();
    return true;
  }
  catch (Glib::Exception &e) {
    __cam_mutex->unlock();
    printf("save failed: %s\n", e.what().c_str());
    return false;
  }
}

/**
 * Saves the content of the image on every refresh
 * 
 * @param enable  enables or disables the feature
 * @param path    to save the images at
 * @param type    file type (@see ImageWidget::save_image)
 * @param img_num of which to start the numbering (actually the first image is numbered img_num + 1)
 */
void
ImageWidget::save_on_refresh_cam(bool enable, std::string path, Glib::ustring type, unsigned int img_num)
{
  __refresh_thread->save_on_refresh(enable, path, type, img_num);
}

/**
 * Returns the latest image number
 * @return the latest image number
 */
unsigned int
ImageWidget::get_image_num()
{
  return __refresh_thread->get_img_num();
}

/**
 * Creates a new refresh thread
 * 
 * @param widget to be refreshed
 * @param refresh_delay time between two refreshes (in [ms])
 */
ImageWidget::RefThread::RefThread(ImageWidget *widget, unsigned int refresh_delay)
: Thread("ImageWidget refresh thread")
{
  set_delete_on_exit(true);

  __widget     = widget;
  __stop       = false;
  __do_refresh = false;
  
  __save_imgs  = false;
  __save_num   = 0;

  __dispatcher.connect( sigc::mem_fun( *widget , &ImageWidget::set_cam ) );

  set_delay(refresh_delay);
}

/**
 * Sets the refresh delay for automatic camera refreshes
 *  
 * @param refresh_delay im [ms]
 */
void
ImageWidget::RefThread::set_delay(unsigned int refresh_delay)
{
  __refresh_delay = refresh_delay;
  __loop_cnt = 0;
}

/**
 * Refreshes the camera during the next loop
 */
void
ImageWidget::RefThread::refresh_cam()
{
  __do_refresh = true;
}

/**
 * Refreshes the Image (getting a new frame from the camera)
 */
void
ImageWidget::RefThread::perform_refresh()
{
  if (!__widget->__cam) throw fawkes::NullPointerException("Camera hasn't been given during creation");
  
  try {
    if (__widget->__cam_mutex->try_lock()) {
      __widget->__cam->capture();
      if (!__stop) {
        __widget->__cam_has_buffer = true;
        __widget->__cam_mutex->unlock();
  
        if (__widget->__cam->ready()) {
          __dispatcher();

          if (__save_imgs) {
            char *ctmp;
            asprintf(&ctmp, "%s/%06u.%s", __save_path.c_str(), ++__save_num, __save_type.c_str());
            Glib::ustring fn = ctmp;
            free(ctmp);
            __widget->save_image(fn, __save_type);
          }
        }
      }
    }
  }
  catch (fawkes::Exception &e) {
    printf("Could not capture the image (%s)\n", e.what());
  }
}


void
ImageWidget::RefThread::loop()
{
  if (!__stop) {
    ++__loop_cnt;

    if (__refresh_delay && !(__loop_cnt % __refresh_delay)) {
      perform_refresh();
      __do_refresh = false;
      __loop_cnt = 0;
    }

    if (__do_refresh) {
      perform_refresh();
      __do_refresh = false;
      __loop_cnt   = 0;
    }
  }
  else exit();

  Glib::usleep(1000);
}

/**
 * Stops (and destroys) the thread as soon as possible (at the next loop)
 */
void
ImageWidget::RefThread::stop()
{
  __stop = true;
}


void
ImageWidget::RefThread::save_on_refresh(bool enabled, std::string path, Glib::ustring type, unsigned int img_num)
{
  __save_imgs = enabled;

  if (__save_imgs) {
    __save_path = path;
    __save_type = type;
    __save_num  = img_num;
  }
}

unsigned int
ImageWidget::RefThread::get_img_num()
{
  return __save_num;
}
