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
  cam_            = NULL;
  cam_enabled_    = false;
  cam_mutex_      = new fawkes::Mutex;
  refresh_thread_ = NULL;

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

  cam_            = cam;
  cam_enabled_    = true;
  cam_mutex_      = new fawkes::Mutex;
  cam_has_buffer_ = false;

  set_size(width, height);

  try {
    fawkes::Time *time = cam_->capture_time();
    delete time;
    cam_has_timestamp_ = true;
  }
  catch (fawkes::Exception &e) {
    cam_has_timestamp_ = false;
  }

  refresh_thread_ = new RefThread(this, refresh_delay);
  refresh_thread_->start();
  refresh_thread_->refresh_cam();
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
  cam_            = NULL;
  cam_enabled_    = false;
  cam_mutex_      = new fawkes::Mutex;
  refresh_thread_ = NULL;
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
  cam_            = NULL;
  cam_enabled_    = false;
  cam_mutex_      = new fawkes::Mutex;
  refresh_thread_ = NULL;

//   set_size(Gtk::Image::get_width(), Gtk::Image::get_height());
}
#endif

/**
 * Destructor
 */
ImageWidget::~ImageWidget()
{
  if (refresh_thread_) refresh_thread_->stop();
  delete cam_mutex_;
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
  cam_            = cam;
  cam_enabled_    = true;
  cam_has_buffer_ = false;

  set_size(cam_->pixel_width(), cam_->pixel_height());

  try {
    fawkes::Time *time = cam_->capture_time();
    delete time;
    cam_has_timestamp_ = true;
  }
  catch (fawkes::Exception &e) {
    cam_has_timestamp_ = false;
  }

  if ( refresh_thread_ ) {
    refresh_thread_->set_delay(refresh_delay);
  } else {
    refresh_thread_ = new RefThread(this, refresh_delay);
    refresh_thread_->start();
  }

  refresh_thread_->refresh_cam();
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
  if ( !enable && cam_enabled_ ) {
    refresh_thread_->stop();
  } else if ( refresh_thread_ && enable && !cam_enabled_ ) {
    refresh_thread_->start();
  }

  cam_enabled_ = enable;
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
    if (cam_) {
      width  = cam_->pixel_width();
      height = cam_->pixel_height();
    }
    else {
      throw fawkes::IllegalArgumentException("ImageWidget::set_size(): width and/or height may not be 0 if no Camera is set");
    }
  }

  if (!pixbuf_ || width_ != width || height_ != height) {
    width_  = width;
    height_ = height;

#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 14 )
    pixbuf_.reset();
#else
    pixbuf_.clear();
#endif

    pixbuf_ = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, false, 8, width_, height_);

    set_size_request(width_, height_);
  }
}
/**
 * Returns the image buffer width
 * @return width of the contained image
 */
unsigned int
ImageWidget::get_width() const
{
    return width_;
}

/**
 * Returns the image buffer height
 * @return height of the contained image
 */
unsigned int
ImageWidget::get_height() const
{
    return height_;
}

/**
 * Returns the widgets pixel buffer (RGB!)
 * @return the RGB pixel buffer
 */
Glib::RefPtr<Gdk::Pixbuf>
ImageWidget::get_buffer() const
{
    return pixbuf_;
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
  if (x >= width_) throw fawkes::OutOfBoundsException("x-Coordinate exeeds image width", x, 0, width_);
  if (y >= height_) throw fawkes::OutOfBoundsException("y-Coordinate exeeds image height", x, 0, height_);

  RGB_t * target = RGB_PIXEL_AT(pixbuf_->get_pixels(), width_, x, y);
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
    if (!width || !height || (width == width_ && height == height_)) {
      convert(colorspace, RGB, buffer, pixbuf_->get_pixels(), width_, height_);
    }
    else {
      unsigned char *scaled_buffer = (unsigned char *)malloc(colorspace_buffer_size(colorspace, width_, height_));

      if (scaled_buffer) {
        LossyScaler scaler;
        scaler.set_original_buffer(buffer);
        scaler.set_original_dimensions(width, height);
        scaler.set_scaled_buffer(scaled_buffer);
        scaler.set_scaled_dimensions(width_, height_);
        scaler.scale();

        convert(colorspace, RGB, scaled_buffer, pixbuf_->get_pixels(), width_, height_);

        free(scaled_buffer);
      }
    }
  }
  catch (fawkes::Exception &e) {
    printf("ImageWidget::show(): %s\n", e.what());
    return false;
  }

  try {
    set(pixbuf_);
    signal_show_.emit(colorspace, buffer, width, height);
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
  return signal_show_;
}


/**
 * Sets the refresh delay for automatic camera refreshes
 *
 * @param refresh_delay im [ms]
 */
void
ImageWidget::set_refresh_delay(unsigned int refresh_delay)
{
  refresh_thread_->set_delay(refresh_delay);
}


/**
 * Performs a refresh during the next loop of the refresh thread
 */
void
ImageWidget::refresh_cam()
{
  if ( cam_enabled_ ) {
    refresh_thread_->refresh_cam();
  }
}

/**
 * Sets the widgets pixbuf after (i.e. non blocking) retrieving the image
 * over the network.
 */
void
ImageWidget::set_cam()
{
  if ( !cam_enabled_ ) { return; }

  cam_mutex_->lock();

  if (cam_has_buffer_) {
    show(cam_->colorspace(), cam_->buffer(), cam_->pixel_width(), cam_->pixel_height());
    cam_->flush();
    cam_has_buffer_ = false;
  }

  cam_mutex_->unlock();
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
  cam_mutex_->lock();

  try {
    pixbuf_->save(filename, type);
    cam_mutex_->unlock();
    return true;
  }
  catch (Glib::Exception &e) {
    cam_mutex_->unlock();
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
  refresh_thread_->save_on_refresh(enable, path, type, img_num);
}

/**
 * Returns the latest image number
 * @return the latest image number
 */
unsigned int
ImageWidget::get_image_num()
{
  return refresh_thread_->get_img_num();
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

  widget_     = widget;
  stop_       = false;
  do_refresh_ = false;

  save_imgs_  = false;
  save_num_   = 0;

  dispatcher_.connect( sigc::mem_fun( *widget , &ImageWidget::set_cam ) );

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
  refresh_delay_ = refresh_delay;
  loop_cnt_ = 0;
}

/**
 * Refreshes the camera during the next loop
 */
void
ImageWidget::RefThread::refresh_cam()
{
  do_refresh_ = true;
}

/**
 * Refreshes the Image (getting a new frame from the camera)
 */
void
ImageWidget::RefThread::perform_refresh()
{
  if (!widget_->cam_) {
    throw fawkes::NullPointerException("Camera hasn't been given during creation");
  }

  try {
    if (widget_->cam_mutex_->try_lock()) {
      widget_->cam_->dispose_buffer();
      widget_->cam_->capture();
      if (!stop_) {
        widget_->cam_has_buffer_ = true;
        widget_->cam_mutex_->unlock();

        if (widget_->cam_->ready()) {
          dispatcher_();

          if (save_imgs_) {
            char *ctmp;
            if (widget_->cam_has_timestamp_) {
              try {
                fawkes::Time *ts = widget_->cam_->capture_time();
                if (asprintf(&ctmp, "%s/%06u.%ld.%s", save_path_.c_str(), ++save_num_, ts->in_msec(), save_type_.c_str()) != -1) {
                  Glib::ustring fn = ctmp;
                  widget_->save_image(fn, save_type_);
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
              if (asprintf(&ctmp, "%s/%06u.%s", save_path_.c_str(), ++save_num_, save_type_.c_str()) != -1) {
                Glib::ustring fn = ctmp;
                widget_->save_image(fn, save_type_);
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
  if (!stop_) {
    ++loop_cnt_;

    if (refresh_delay_ && !(loop_cnt_ % refresh_delay_)) {
      perform_refresh();
      do_refresh_ = false;
      loop_cnt_ = 0;
    }

    if (do_refresh_) {
      perform_refresh();
      do_refresh_ = false;
      loop_cnt_   = 0;
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
  stop_ = true;
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
  save_imgs_ = enabled;

  if (save_imgs_) {
    save_path_ = path;
    save_type_ = type;
    save_num_  = img_num;
  }
}

/** Get image number.
 * @return image number
 */
unsigned int
ImageWidget::RefThread::get_img_num()
{
  return save_num_;
}

} // end namespace firevision
