
/***************************************************************************
 *  bb2calib.cpp - Bumblebee2 calibration GUI
 *
 *  Created: Thu Jul 18 20:59:47 2013
 *  Copyright  2008-2013  Tim Niemueller [www.niemueller.de]
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

#include "bb2calib.h"

#include <blackboard/remote.h>
#include <interfaces/OpenCVStereoParamsInterface.h>
#include <netcomm/fawkes/client.h>
#include <fvcams/net.h>
#include <fvutils/color/conversions.h>

#include <gui_utils/service_chooser_dialog.h>
#include <gui_utils/interface_dispatcher.h>

#include <cstring>
#include <string>
#include <sstream>
#include <iomanip>
#include <sstream>

using namespace firevision;
using namespace fawkes;

#define FIREVISION_PORT 2208
#define BB2_IMG_RECT_LEFT "bumblebee2-rgb-rectified-left"
#define BB2_IMG_DISPARITY "bumblebee2-disparity"
#define IMG_UPDATE_INTERVAL 200

/** @class Bumblebee2CalibGtkWindow "naogui.h"
 * Bumblebee2 calibration GUI main window.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param builder Gtk builder to get widgets from
 */
Bumblebee2CalibGtkWindow::Bumblebee2CalibGtkWindow(BaseObjectType* cobject,
						   const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Window(cobject)
{
  bb_ = NULL;
  params_if_ = NULL;
  cam_left_rectified_ = cam_disparity_ = NULL;
  buffer_rgb_disparity_ = buffer_rgb_rect_left_ = NULL;

  builder->get_widget("cmb_pre_filter_type", cmb_pre_filter_type);
  builder->get_widget("lab_pre_filter_type", lab_pre_filter_type);
  builder->get_widget("lab_pre_filter_size", lab_pre_filter_size);
  builder->get_widget("lab_pre_filter_cap", lab_pre_filter_cap);
  builder->get_widget("lab_sad_window_size", lab_sad_window_size);
  builder->get_widget("lab_min_disparity", lab_min_disparity);
  builder->get_widget("lab_num_disparities", lab_num_disparities);
  builder->get_widget("lab_texture_threshold", lab_texture_threshold);
  builder->get_widget("lab_uniqueness_ratio", lab_uniqueness_ratio);
  builder->get_widget("lab_speckle_window_size", lab_speckle_window_size);
  builder->get_widget("lab_speckle_range", lab_speckle_range);
  builder->get_widget("lab_try_smaller_windows", lab_try_smaller_windows);
  builder->get_widget("hsc_pre_filter_size", hsc_pre_filter_size);
  builder->get_widget("hsc_pre_filter_cap", hsc_pre_filter_cap);
  builder->get_widget("hsc_sad_window_size", hsc_sad_window_size);
  builder->get_widget("hsc_min_disparity", hsc_min_disparity);
  builder->get_widget("hsc_num_disparities", hsc_num_disparities);
  builder->get_widget("hsc_texture_threshold", hsc_texture_threshold);
  builder->get_widget("hsc_uniqueness_ratio", hsc_uniqueness_ratio);
  builder->get_widget("hsc_speckle_window_size", hsc_speckle_window_size);
  builder->get_widget("cb_try_smaller_windows", cb_try_smaller_windows);
  builder->get_widget("hsc_speckle_range", hsc_speckle_range);
  builder->get_widget("tb_connection", tb_connection);
  builder->get_widget("tb_exit", tb_exit);
  builder->get_widget("img_left_rectified", img_left_rectified);
  builder->get_widget("img_disparity", img_disparity);
  builder->get_widget("img_writer", img_writer);

  cmb_pre_filter_type->signal_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_pre_filter_type_changed));
  hsc_pre_filter_size->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_pre_filter_size_changed));
  hsc_pre_filter_cap->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_pre_filter_cap_changed));
  hsc_sad_window_size->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_sad_window_size_changed));
  hsc_min_disparity->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_min_disparity_changed));
  hsc_num_disparities->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_num_disparities_changed));
  hsc_texture_threshold->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_texture_threshold_changed));
  hsc_uniqueness_ratio->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_uniqueness_ratio_changed));
  hsc_speckle_window_size->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_speckle_window_size_changed));
  hsc_speckle_range->signal_value_changed().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_speckle_range_changed));
  cb_try_smaller_windows->signal_toggled().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_try_smaller_windows_toggled));

  tb_connection->signal_clicked().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_connection_clicked));
  tb_exit->signal_clicked().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_exit_clicked));
;
  connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_connect));
  connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::on_disconnect));

  init();
}


/** Destructor. */
Bumblebee2CalibGtkWindow::~Bumblebee2CalibGtkWindow()
{
  on_disconnect();
}

/**
 * Sets the default values (locale dependent)
 */
void
Bumblebee2CalibGtkWindow::init()
{
}

/** Event handler for combo box changes. */
void
Bumblebee2CalibGtkWindow::on_pre_filter_type_changed()
{
  OpenCVStereoParamsInterface::PreFilterType new_type;
  if (cmb_pre_filter_type->get_active_row_number() == 1) {
    new_type = OpenCVStereoParamsInterface::PFT_NORMALIZED_RESPONSE;
    } else {
    new_type = OpenCVStereoParamsInterface::PFT_XSOBEL;
  }

  if (params_if_ && params_if_->has_writer() && (params_if_->pre_filter_type() != new_type)) {
    printf("Setting pre filter type %s\n", params_if_->tostring_PreFilterType(new_type));
    OpenCVStereoParamsInterface::SetPreFilterTypeMessage *msg =
      new OpenCVStereoParamsInterface::SetPreFilterTypeMessage(new_type);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_pre_filter_size_changed()
{
  unsigned new_value = (unsigned int)hsc_pre_filter_size->get_value() * 2 + 1;

  if (params_if_ && params_if_->has_writer() && (params_if_->pre_filter_size() != new_value)) {
    printf("Setting pre filter size %u\n", new_value);
    OpenCVStereoParamsInterface::SetPreFilterSizeMessage *msg =
      new OpenCVStereoParamsInterface::SetPreFilterSizeMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_pre_filter_cap_changed()
{
  unsigned int new_value = (unsigned int)hsc_pre_filter_cap->get_value();
  if (params_if_ && params_if_->has_writer() && (params_if_->pre_filter_cap() != new_value)) {
    printf("Setting pre filter cap %u\n", new_value);
    OpenCVStereoParamsInterface::SetPreFilterCapMessage *msg =
      new OpenCVStereoParamsInterface::SetPreFilterCapMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_sad_window_size_changed()
{
  unsigned int new_value = (unsigned int)hsc_sad_window_size->get_value() * 2 + 1;

  if (params_if_ && params_if_->has_writer() && (params_if_->sad_window_size() != new_value)) {
    printf("Setting SAD window size %u\n", new_value);
    OpenCVStereoParamsInterface::SetSADWindowSizeMessage *msg =
      new OpenCVStereoParamsInterface::SetSADWindowSizeMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_min_disparity_changed()
{
  int new_value = (int)hsc_min_disparity->get_value();
  if (params_if_ && params_if_->has_writer() && (params_if_->min_disparity() != new_value)) {
    printf("Setting min disparity %i\n", new_value);
    OpenCVStereoParamsInterface::SetMinDisparityMessage *msg =
      new OpenCVStereoParamsInterface::SetMinDisparityMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_num_disparities_changed()
{
  unsigned int new_value = (unsigned int)hsc_num_disparities->get_value() * 16;
  if (params_if_ && params_if_->has_writer() && (params_if_->num_disparities() != new_value)) {
    printf("Setting num disparities %u\n", new_value);
    OpenCVStereoParamsInterface::SetNumDisparitiesMessage *msg =
      new OpenCVStereoParamsInterface::SetNumDisparitiesMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_texture_threshold_changed()
{
  unsigned int new_value = (unsigned int)hsc_texture_threshold->get_value();
  if (params_if_ && params_if_->has_writer() &&
      (params_if_->texture_threshold() != new_value))
  {
    printf("Setting texture threshold %u\n", new_value);
    OpenCVStereoParamsInterface::SetTextureThresholdMessage *msg =
      new OpenCVStereoParamsInterface::SetTextureThresholdMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_uniqueness_ratio_changed()
{
  unsigned int new_value = (unsigned int)hsc_uniqueness_ratio->get_value();
  if (params_if_ && params_if_->has_writer() &&
      (params_if_->uniqueness_ratio() != new_value))
  {
    printf("Setting uniqueness ratio %u\n", new_value);
    OpenCVStereoParamsInterface::SetUniquenessRatioMessage *msg =
      new OpenCVStereoParamsInterface::SetUniquenessRatioMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_speckle_window_size_changed()
{
  unsigned int new_value = (unsigned int)hsc_speckle_window_size->get_value();
  if (params_if_ && params_if_->has_writer() &&
      (params_if_->speckle_window_size() != new_value)) {
    printf("Setting speckle window size %u\n", new_value);
    OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage *msg =
      new OpenCVStereoParamsInterface::SetSpeckleWindowSizeMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for slider changes. */
void
Bumblebee2CalibGtkWindow::on_speckle_range_changed()
{
  unsigned int new_value = (unsigned int)hsc_speckle_range->get_value();
  if (params_if_ && params_if_->has_writer() && (params_if_->speckle_range() != new_value)) {
    printf("Setting speckle range %u\n", new_value);
    OpenCVStereoParamsInterface::SetSpeckleRangeMessage *msg =
      new OpenCVStereoParamsInterface::SetSpeckleRangeMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}


/** Event handler for connection button. */
void
Bumblebee2CalibGtkWindow::on_connection_clicked()
{
  if ( ! connection_dispatcher.get_client()->connected() ) {
    ServiceChooserDialog ssd(*this, connection_dispatcher.get_client());
    ssd.run_and_connect();
  } else {
    connection_dispatcher.get_client()->disconnect();
  }
}


/** Event handler for combo box changes. */
void
Bumblebee2CalibGtkWindow::on_try_smaller_windows_toggled()
{
  bool new_value = cb_try_smaller_windows->get_active();

  if (params_if_ && params_if_->has_writer() &&
      (params_if_->is_try_smaller_windows() != new_value))
  {
    printf("%sabling smaller windows\n", new_value ? "En" : "Dis");
    OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage *msg =
      new OpenCVStereoParamsInterface::SetTrySmallerWindowsMessage(new_value);
    params_if_->msgq_enqueue(msg);
  }
}

/** Event handler for connected event. */
void
Bumblebee2CalibGtkWindow::on_connect()
{
  try {
    bb_ = new RemoteBlackBoard(connection_dispatcher.get_client());
    params_if_ =
      bb_->open_for_reading<OpenCVStereoParamsInterface>("bumblebee2");

    if (! params_if_->has_writer()) {
      throw Exception("No writer for parameter blackboard interface");
    }

    ifd_params_ = new InterfaceDispatcher("Bumblebee2OpenCVParamsIfaceDisp", params_if_);
    ifd_params_->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::update_param_values)));

    bb_->register_listener(ifd_params_, BlackBoard::BBIL_FLAG_DATA);

    tb_connection->set_stock_id(Gtk::Stock::DISCONNECT);

    update_param_values();

    switch (params_if_->pre_filter_type()) {
    case OpenCVStereoParamsInterface::PFT_XSOBEL:
      cmb_pre_filter_type->set_active(0);
      break;
    default:
      cmb_pre_filter_type->set_active(1);
      break;
    }
    hsc_pre_filter_size->set_value(params_if_->pre_filter_size() / 2);
    hsc_pre_filter_cap->set_value(params_if_->pre_filter_cap());
    hsc_sad_window_size->set_value(params_if_->sad_window_size() / 2);
    hsc_min_disparity->set_value(params_if_->min_disparity());
    hsc_num_disparities->set_value(params_if_->num_disparities() / 16);
    hsc_texture_threshold->set_value(params_if_->texture_threshold());
    hsc_uniqueness_ratio->set_value(params_if_->uniqueness_ratio());
    hsc_speckle_window_size->set_value(params_if_->speckle_window_size());
    hsc_speckle_range->set_value(params_if_->speckle_range());
    cb_try_smaller_windows->set_active(params_if_->is_try_smaller_windows());

    cmb_pre_filter_type->set_sensitive(true);
    hsc_pre_filter_size->set_sensitive(true);
    hsc_pre_filter_cap->set_sensitive(true);
    hsc_sad_window_size->set_sensitive(true);
    hsc_min_disparity->set_sensitive(true);
    hsc_num_disparities->set_sensitive(true);
    hsc_texture_threshold->set_sensitive(true);
    hsc_uniqueness_ratio->set_sensitive(true);
    hsc_speckle_window_size->set_sensitive(true);
    hsc_speckle_range->set_sensitive(true);
    cb_try_smaller_windows->set_sensitive(true);

    cam_left_rectified_ = new NetworkCamera(connection_dispatcher.get_client()->get_hostname(),
					    FIREVISION_PORT, BB2_IMG_RECT_LEFT, false);
    cam_left_rectified_->open();
    cam_left_rectified_->start();
    printf("Colorspace: %s\n", colorspace_to_string(cam_left_rectified_->colorspace()));

    cam_disparity_ = new NetworkCamera(connection_dispatcher.get_client()->get_hostname(),
				       FIREVISION_PORT, BB2_IMG_DISPARITY, false);
    cam_disparity_->open();
    cam_disparity_->start();
    buffer_rgb_disparity_ =
      malloc_buffer(RGB, cam_disparity_->pixel_width(), cam_disparity_->pixel_height());
    buffer_rgb_rect_left_ =
      malloc_buffer(RGB, cam_left_rectified_->pixel_width(),
		    cam_left_rectified_->pixel_height());

    sconn_update_images_ =
      Glib::signal_timeout().connect(
        sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::update_images), IMG_UPDATE_INTERVAL);


    this->set_title(std::string("Bumblebee2 Calibration @ ") +
		    connection_dispatcher.get_client()->get_hostname());

  } catch (Exception &e) {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*this, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("Connection failed");
    md.run();
    if ( bb_ ) {
      bb_->unregister_listener(ifd_params_);
      bb_->close(params_if_);
      delete ifd_params_;
      delete bb_;
      params_if_ = NULL;
      bb_ = NULL;
      ifd_params_ = NULL;
    }
    delete cam_left_rectified_;
    delete cam_disparity_;
    if (buffer_rgb_disparity_)  free(buffer_rgb_disparity_);
    if (buffer_rgb_rect_left_)  free(buffer_rgb_rect_left_);
    cam_left_rectified_ = cam_disparity_ = NULL;
    buffer_rgb_disparity_ = buffer_rgb_rect_left_ = NULL;

    connection_dispatcher.get_client()->disconnect();
  }
}

/** Event handler for disconnected event. */
void
Bumblebee2CalibGtkWindow::on_disconnect()
{
  sconn_update_images_.disconnect();

  img_writer->set_from_icon_name(Gtk::Stock::NO.id,
				 Gtk::IconSize(Gtk::ICON_SIZE_SMALL_TOOLBAR));
  img_writer->set_tooltip_text("Not connected and thus no writer");

  img_disparity->clear();
  img_disparity->set("gtk-missing-image");

  img_left_rectified->clear();
  img_left_rectified->set("gtk-missing-image");

  lab_pre_filter_size->set_text("");
  lab_pre_filter_cap->set_text("");
  lab_sad_window_size->set_text("");
  lab_min_disparity->set_text("");
  lab_num_disparities->set_text("");
  lab_texture_threshold->set_text("");
  lab_uniqueness_ratio->set_text("");
  lab_speckle_window_size->set_text("");
  lab_speckle_range->set_text("");
  lab_try_smaller_windows->set_text("");

  cmb_pre_filter_type->set_sensitive(false);
  hsc_pre_filter_size->set_sensitive(false);
  hsc_pre_filter_cap->set_sensitive(false);
  hsc_sad_window_size->set_sensitive(false);
  hsc_min_disparity->set_sensitive(false);
  hsc_num_disparities->set_sensitive(false);
  hsc_texture_threshold->set_sensitive(false);
  hsc_uniqueness_ratio->set_sensitive(false);
  hsc_speckle_window_size->set_sensitive(false);
  hsc_speckle_range->set_sensitive(false);
  cb_try_smaller_windows->set_sensitive(false);

  if (bb_) {
    bb_->unregister_listener(ifd_params_);
    bb_->close(params_if_);

    delete ifd_params_;
    delete bb_;
    params_if_ = NULL;
    bb_ = NULL;
    ifd_params_ = NULL;
  }
  if (cam_disparity_) {
    cam_disparity_->stop();
    cam_disparity_->close();
    delete cam_disparity_;
    cam_disparity_ = NULL;
  }
  if (buffer_rgb_disparity_)  free(buffer_rgb_disparity_);
  buffer_rgb_disparity_ = NULL;

  if (buffer_rgb_rect_left_)  free(buffer_rgb_rect_left_);
  buffer_rgb_rect_left_ = NULL;

  if (cam_left_rectified_) {
    cam_left_rectified_->stop();
    cam_left_rectified_->close();
    delete cam_left_rectified_;
    cam_left_rectified_ = NULL;
  }

  tb_connection->set_stock_id(Gtk::Stock::CONNECT);

  //img_writer->set_stock_id(Gtk::Stock::NO);
  this->set_title("Bumblebee2 Calibration");
}


void
Bumblebee2CalibGtkWindow::update_param_values()
{
  params_if_->read();

  switch (params_if_->pre_filter_type()) {
  case OpenCVStereoParamsInterface::PFT_XSOBEL:
    lab_pre_filter_type->set_text("XSOBEL");
    break;
  default:
    lab_pre_filter_type->set_text("NORM RESP");
    break;
  }
  lab_pre_filter_size->set_text(convert_float2str(params_if_->pre_filter_size(), 0));
  lab_pre_filter_cap->set_text(convert_float2str(params_if_->pre_filter_cap(), 0));
  lab_sad_window_size->set_text(convert_float2str(params_if_->sad_window_size(), 0));
  lab_min_disparity->set_text(convert_float2str(params_if_->min_disparity(), 0));
  lab_num_disparities->set_text(convert_float2str(params_if_->num_disparities(), 0));
  lab_texture_threshold->set_text(convert_float2str(params_if_->texture_threshold(), 0));
  lab_uniqueness_ratio->set_text(convert_float2str(params_if_->uniqueness_ratio(), 0));
  lab_speckle_window_size->set_text(convert_float2str(params_if_->speckle_window_size(), 0));
  lab_speckle_range->set_text(convert_float2str(params_if_->speckle_range(), 0));
  lab_try_smaller_windows->set_text(params_if_->is_try_smaller_windows() ? "true" : "false");
}


void
Bumblebee2CalibGtkWindow::dont_destroy(const guint8 *data)
{
}

bool
Bumblebee2CalibGtkWindow::update_images()
{
  if (bb_ && params_if_ && params_if_->has_writer())
  {
    if (img_writer->get_icon_name() != Gtk::Stock::YES.id) {
      img_writer->set_from_icon_name(Gtk::Stock::YES.id,
				     Gtk::IconSize(Gtk::ICON_SIZE_SMALL_TOOLBAR));
      img_writer->set_tooltip_text("Writer for blackboard interface exists");
    }

    cam_left_rectified_->capture();
    unsigned int rlwidth  = cam_left_rectified_->pixel_width();
    unsigned int rlheight = cam_left_rectified_->pixel_height();
    convert(cam_left_rectified_->colorspace(), RGB,
	    cam_left_rectified_->buffer(), buffer_rgb_rect_left_,
	    rlwidth, rlheight);
    cam_left_rectified_->dispose_buffer();

    Glib::RefPtr<Gdk::Pixbuf> image =
      Gdk::Pixbuf::create_from_data(buffer_rgb_rect_left_, Gdk::COLORSPACE_RGB,
				    /* has alpha */ false, /* bits per color */ 8,
				    rlwidth, rlheight, /* row stride */ 3 * rlwidth,
				    sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::dont_destroy));

    image = image->scale_simple(320, 240, Gdk::INTERP_BILINEAR);

    img_left_rectified->set(image);


    // size must be the same as the rectified image
    cam_disparity_->capture();
    convert(cam_disparity_->colorspace(), RGB,
	  cam_disparity_->buffer(), buffer_rgb_disparity_,
	    rlwidth, rlheight);
    cam_disparity_->dispose_buffer();

    Glib::RefPtr<Gdk::Pixbuf> dimage =
      Gdk::Pixbuf::create_from_data(buffer_rgb_disparity_, Gdk::COLORSPACE_RGB,
				    /* has alpha */ false, /* bits per color */ 8,
				    rlwidth, rlheight, /* row stride */ 3 * rlwidth,
				    sigc::mem_fun(*this, &Bumblebee2CalibGtkWindow::dont_destroy));
    img_disparity->set(dimage);
  } else {
    if (img_writer->get_icon_name() != Gtk::Stock::NO.id) {
      img_writer->set_from_icon_name(Gtk::Stock::NO.id,
				     Gtk::IconSize(Gtk::ICON_SIZE_SMALL_TOOLBAR));
      img_writer->set_tooltip_text("There is no blackboard writer for the interface");
    }
  }

  return true;
}

void
Bumblebee2CalibGtkWindow::on_exit_clicked()
{
  Gtk::Main::quit();
}

/**
 * Converts a float value to a Glib::ustring (locale dependent)
 * @param f The float value
 * @param width The precision width
 * @return the formatted string
 */
Glib::ustring
Bumblebee2CalibGtkWindow::convert_float2str(float f, unsigned int width)
{
#if GLIBMM_MAJOR_VERSION > 2 || ( GLIBMM_MAJOR_VERSION == 2 && GLIBMM_MINOR_VERSION >= 16 )
  return Glib::ustring::format(std::fixed, std::setprecision(width), f);
#else
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(width);
  ss << f;

  return Glib::locale_to_utf8(ss.str());
#endif
}



bool
Bumblebee2CalibGtkWindow::convert_str2float(Glib::ustring sn, float *f)
{
  char *endptr = NULL;
  *f = strtof(sn.c_str(), &endptr);
  if ( endptr[0] != 0 ) {
    Glib::ustring s("Could not convert string to valid number: ");
    s.append(sn, 0, sn.length() - strlen(endptr));
    s += "   &gt;&gt;&gt;<b>";
    s += endptr[0];
    s += "</b>&lt;&lt;&lt;   ";
    s.append(endptr + 1, strlen(endptr) - 1);

    Gtk::MessageDialog md(*this, s,
			  /* use markup */ true,
			  Gtk::MESSAGE_ERROR);
    md.set_title("Invalid value");
    md.run();
    md.hide();
    return false;
  } else {
    return true;
  }
}

