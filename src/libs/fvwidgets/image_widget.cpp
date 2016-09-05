/***************************************************************************
 *  image_widget.cpp - Gtkmm widget to draw an image inside a Gtk::Window
 *
 *  Created:  26.11.2008
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


#include "image_widget.h"

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <fvutils/color/conversions.h>
#include <fvutils/color/yuv.h>
#include <fvutils/scalers/lossy.h>
#include <fvcams/camera.h>

#include <iomanip>


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ImageWidget <fvwidgets/image_widget.h>
 * This class is an image container to display fawkes cameras (or image
 * buffers) inside a Gtk::Container
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
  __cam            = NULL;
  __cam_enabled    = false;
  __cam_mutex      = new fawkes::Mutex;
  __refresh_thread = NULL;

  set_size(width, height);
}

/**
 * Creates a new ImageWidget with a Camera as image source
 * @param cam the image source
 * @param refresh_delay if greater 0 a thread gets created that refreshes
 *        the Image every refresh_delay milliseconds
 * @param width of the widget (if not equal to the camera width the image
 *        gets scaled)
 * @param height of the widget (if not equal to the camera height the
 *        image gets scaled)
 */
ImageWidget::ImageWidget(Camera *cam, unsigned int refresh_delay, unsigned int width, unsigned int height)
{
  if (!cam) throw fawkes::NullPointerException("Parameter cam may not be NULL");

  __cam            = cam;
  __cam_enabled    = true;
  __cam_mutex      = new fawkes::Mutex;
  __cam_has_buffer = false;

  set_size(width, height);

  try {
    fawkes::Time *time = __cam->capture_time();
    delete time;
    __cam_has_timestamp = true;
  }
  catch (fawkes::Exception &e) {
    __cam_has_timestamp = false;
  }

  __refresh_thread = new RefThread(this, refresh_delay);
  __refresh_thread->start();
  __refresh_thread->refresh_cam();
}

/** Constructor for Gtk::Builder.
 * Constructor that can be used to instantiate an ImageWidget as a
 * derived widget from a Gtk builder file.
 *
 * Note: The ImageWidget (and its internal buffer) is set to the size
 * as in the UI file, in case no camera is set afterwards. Use @see
 * ImageWidget::set_size() to resize the ImageWidget afterwards.
 *
 * @param cobject pointer to the base object
 * @param builder Builder
 */
ImageWidget::ImageWidget(BaseObjectType* cobject, Glib::RefPtr<Gtk::Builder> builder)
  : Gtk::Image(cobject)
{
  __cam            = NULL;
  __cam_enabled    = false;
  __cam_mutex      = new fawkes::Mutex;
  __refresh_thread = NULL;
//   set_size(Gtk::Image::get_width(), Gtk::Image::get_height());
}

#ifdef HAVE_GLADEMM
/** Constructor for Glade.
 * Constructor that can be used to instantiate an ImageWidget as a
 * derived widget from a Glade file.
 *
 * Note: The ImageWidget (and its internal buffer) is set to the size
 * as in the glade file, in case no camera is set afterwards. Use @see
 * ImageWidget::set_size() to resize the ImageWidget afterwards.
 *
 * @param cobject pointer to the base object
 * @param refxml the Glade XML file
 */
ImageWidget::ImageWidget(BaseObjectType* cobject, Glib::RefPtr<Gnome::Glade::Xml> refxml)
  : Gtk::Image( cobject )
{
  __cam            = NULL;
  __cam_enabled    = false;
  __cam_mutex      = new fawkes::Mutex;
  __refresh_thread = NULL;

//   set_size(Gtk::Image::get_width(), Gtk::Image::get_height());
}
#endif

/**
 * Destructor
 */
ImageWidget::~ImageWidget()
{
  if (__refresh_thread) __refresh_thread->stop();
  delete __cam_mutex;
}

/** Set the camera from which the ImageWidget obtains the images.
 *
 * Note: The size of the ImageWidget remains untouched and the cameras
 * image gets scaled appropriately. Use ImageWidget::set_size(0, 0) to
 * set the widget to the size of the camera.
 *
 * @param cam the camera
 * @param refresh_delay the delay between two refreshs in milliseconds
 */
void
ImageWidget::set_camera(Camera *cam, unsigned int refresh_delay)
{
  __cam            = cam;
  __cam_enabled    = true;
  __cam_has_buffer = false;

  set_size(__cam->pixel_width(), __cam->pixel_height());

  try {
    fawkes::Time *time = __cam->capture_time();
    delete time;
    __cam_has_timestamp = true;
  }
  catch (fawkes::Exception &e) {
    __cam_has_timestamp = false;
  }

  if ( __refresh_thread ) {
    __refresh_thread->set_delay(refresh_delay);
  } else {
    __refresh_thread = new RefThread(this, refresh_delay);
    __refresh_thread->start();
  }

  __refresh_thread->refresh_cam();
}

/**
 * En-/disable the camera.
 * @param enable if true the camera is enabled and the refresh thread
 * is start, if false the refresh thread is stopped and the camera is
 * disabled
 */
void
ImageWidget::enable_camera(bool enable)
{
  if ( !enable && __cam_enabled ) {
    __refresh_thread->stop();
  } else if ( __refresh_thread && enable && !__cam_enabled ) {
    __refresh_thread->start();
  }

  __cam_enabled = enable;
}

/** Sets the size of the ImageWidget.
 * Updates the internal buffer and the size request for the ImageWidget.
 * If width and/or height are set to 0 (and a Camera is set) the
 * ImageWidget will be set to the camera dimensions.
 *
 * Note: The ImageWidget must be refreshed after changing its size!
 *
 * @param width The new width
 * @param height The new height
 */
void
ImageWidget::set_size(unsigned int width, unsigned int height)
{
  if (!width || ! height) {
    if (__cam) {
      width  = __cam->pixel_width();
      height = __cam->pixel_height();
    }
    else {
      throw fawkes::IllegalArgumentException("ImageWidget::set_size(): width and/or height may not be 0 if no Camera is set");
    }
  }

  if (!__pixbuf || __width != width || __height != height) {
    __width  = width;
    __height = height;

#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 14 )
    __pixbuf.reset();
#else
    __pixbuf.clear();
#endif

    __pixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8, __width, __height);

    set_size_request(__width, __height);
  }
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
 * Warning: If width and/or height not set, it is assumed, that the given
 * buffer has the same dimension as the widget.
 *
 * @param colorspace colorspace of the supplied buffer
 * @param buffer image buffer
 * @param width Width of the provided buffer (may be scaled to ImageWidget
 *        dimensions)
 * @param height Height of the provided buffer (may be scaled to
 *        ImageWidget dimensions)
 * @return TRUE if the buffer chould have been shown
 */
bool
ImageWidget::show(colorspace_t colorspace, unsigned char *buffer, unsigned int width, unsigned int height)
{
  try {
    if (!width || !height || (width == __width && height == __height)) {
      convert(colorspace, RGB, buffer, __pixbuf->get_pixels(), __width, __height);
    }
    else {
      unsigned char *scaled_buffer = (unsigned char *)malloc(colorspace_buffer_size(colorspace, __width, __height));

      if (scaled_buffer) {
        LossyScaler scaler;
        scaler.set_original_buffer(buffer);
        scaler.set_original_dimensions(width, height);
        scaler.set_scaled_buffer(scaled_buffer);
        scaler.set_scaled_dimensions(__width, __height);
        scaler.scale();

        convert(colorspace, RGB, scaled_buffer, __pixbuf->get_pixels(), __width, __height);

        free(scaled_buffer);
      }
    }
  }
  catch (fawkes::Exception &e) {
    printf("ImageWidget::show(): %s\n", e.what());
    return false;
  }

  try {
    set(__pixbuf);
    __signal_show.emit(colorspace, buffer, width, height);
    return true;
  }
  catch (fawkes::Exception &e) {
    printf("ImageWidget::show(): Could not set the new image (%s)\n", e.what());
  }

  return false;
}


/** Signal emits after a new buffer gets successfully shown
 * (see @see ImageWidget::show()).
 *
 * The buffer's validity can not be guaranteed beyond the called functions
 * scope! In case the source of the widget is a Camera, the buffer gets
 * disposed after calling ImageWidget::show.
 *
 * @return The signal_show signal
 */
sigc::signal<void, colorspace_t, unsigned char *, unsigned int, unsigned int> &
ImageWidget::signal_show()
{
  return __signal_show;
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
  if ( __cam_enabled ) {
    __refresh_thread->refresh_cam();
  }
}

/**
 * Sets the widgets pixbuf after (i.e. non blocking) retrieving the image
 * over the network.
 */
void
ImageWidget::set_cam()
{
  if ( !__cam_enabled ) { return; }

  __cam_mutex->lock();

  if (__cam_has_buffer) {
    show(__cam->colorspace(), __cam->buffer(), __cam->pixel_width(), __cam->pixel_height());
    __cam->flush();
    __cam_has_buffer = false;
  }

  __cam_mutex->unlock();
}

/**
 * Saves the current content of the Image
 * @param filename of the output
 * @param type of the output (By default, "jpeg", "png", "ico" and "bmp"
 *        are possible file formats to save in, but more formats may be
 *        installed. The list of all writable formats can be determined
 *        by using Gdk::Pixbuf::get_formats() with
 *        Gdk::PixbufFormat::is_writable().)
 * @return true on success, false otherwise
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
 * @param img_num of which to start the numbering (actually the first
 *        image is numbered img_num + 1)
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
  if (!__widget->__cam) {
    throw fawkes::NullPointerException("Camera hasn't been given during creation");
  }

  try {
    if (__widget->__cam_mutex->try_lock()) {
      __widget->__cam->dispose_buffer();
      __widget->__cam->capture();
      if (!__stop) {
        __widget->__cam_has_buffer = true;
        __widget->__cam_mutex->unlock();

        if (__widget->__cam->ready()) {
          __dispatcher();

          if (__save_imgs) {
            char *ctmp;
            if (__widget->__cam_has_timestamp) {
              try {
                fawkes::Time *ts = __widget->__cam->capture_time();
                if (asprintf(&ctmp, "%s/%06u.%ld.%s", __save_path.c_str(), ++__save_num, ts->in_msec(), __save_type.c_str()) != -1) {
                  Glib::ustring fn = ctmp;
                  __widget->save_image(fn, __save_type);
                  free(ctmp);
                } else {
                  printf("Cannot save image, asprintf() ran out of memory\n");
                }
                delete ts;
              }
              catch (fawkes::Exception &e) {
                printf("Cannot save image (%s)\n", e.what());
              }
            }
            else {
              if (asprintf(&ctmp, "%s/%06u.%s", __save_path.c_str(), ++__save_num, __save_type.c_str()) != -1) {
                Glib::ustring fn = ctmp;
                __widget->save_image(fn, __save_type);
                free(ctmp);
              } else {
                printf("Cannot save image, asprintf() ran out of memory\n");
              }
            }
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


/** Set save on refresh.
 * @param enabled true to enable, false to disable
 * @param path save path
 * @param type save type
 * @param img_num image number to save
 */
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

/** Get image number.
 * @return image number
 */
unsigned int
ImageWidget::RefThread::get_img_num()
{
  return __save_num;
}

} // end namespace firevision
