
/***************************************************************************
 *  bb2calib.h - Bumblebee2 calibration GUI
 *
 *  Created: Thu Jul 18 20:36:04 2013
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

#ifndef __PLUGINS_BUMBLEBEE2_CALIB_BB2CALIB_H_
#define __PLUGINS_BUMBLEBEE2_CALIB_BB2CALIB_H_

#include <gui_utils/connection_dispatcher.h>

#include <gtkmm.h>

namespace fawkes {
  class BlackBoard;
  class Interface;
  class InterfaceDispatcher;
  class OpenCVStereoParamsInterface;
}

namespace firevision {
  class NetworkCamera;
}

class Bumblebee2CalibGtkWindow : public Gtk::Window
{
 public:
  Bumblebee2CalibGtkWindow(BaseObjectType* cobject,
			   const Glib::RefPtr<Gtk::Builder> &builder);
  ~Bumblebee2CalibGtkWindow();

 private:
  void on_connection_clicked();
  void on_connect();
  void on_disconnect();
  void on_exit_clicked();

  void on_pre_filter_type_changed();
  void on_pre_filter_size_changed();
  void on_pre_filter_cap_changed();
  void on_sad_window_size_changed();
  void on_min_disparity_changed();
  void on_num_disparities_changed();
  void on_texture_threshold_changed();
  void on_uniqueness_ratio_changed();
  void on_speckle_window_size_changed();
  void on_speckle_range_changed();
  void on_try_smaller_windows_toggled();

  void update_param_values();
  bool update_images();
  void dont_destroy(const guint8 *data);

  bool convert_str2float(Glib::ustring sn, float *f);
  Glib::ustring convert_float2str(float f, unsigned int width = 2);

  void init();


 private:
  fawkes::BlackBoard *bb_;
  fawkes::InterfaceDispatcher *ifd_params_;
  fawkes::OpenCVStereoParamsInterface *params_if_;
  fawkes::ConnectionDispatcher connection_dispatcher;

  firevision::NetworkCamera *cam_left_rectified_;
  firevision::NetworkCamera *cam_disparity_;
  unsigned char *buffer_rgb_disparity_;
  unsigned char *buffer_rgb_rect_left_;
  sigc::connection sconn_update_images_;

  Gtk::ComboBox *cmb_pre_filter_type;
  Gtk::Label  *lab_pre_filter_type;
  Gtk::HScale *hsc_pre_filter_size;
  Gtk::Label  *lab_pre_filter_size;
  Gtk::HScale *hsc_pre_filter_cap;
  Gtk::Label  *lab_pre_filter_cap;
  Gtk::HScale *hsc_sad_window_size;
  Gtk::Label  *lab_sad_window_size;
  Gtk::HScale *hsc_min_disparity;
  Gtk::Label  *lab_min_disparity;
  Gtk::HScale *hsc_num_disparities;
  Gtk::Label  *lab_num_disparities;
  Gtk::HScale *hsc_texture_threshold;
  Gtk::Label  *lab_texture_threshold;
  Gtk::HScale *hsc_uniqueness_ratio;
  Gtk::Label  *lab_uniqueness_ratio;
  Gtk::HScale *hsc_speckle_window_size;
  Gtk::Label  *lab_speckle_window_size;
  Gtk::HScale *hsc_speckle_range;
  Gtk::Label  *lab_speckle_range;
  Gtk::CheckButton *cb_try_smaller_windows;
  Gtk::Label  *lab_try_smaller_windows;
  Gtk::ToolButton *tb_connection;
  Gtk::ToolButton *tb_exit;
  Gtk::Image  *img_left_rectified;
  Gtk::Image  *img_disparity;
  Gtk::Image  *img_writer;
};

#endif
