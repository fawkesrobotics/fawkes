
/***************************************************************************
 *  color_train_widget.h - Color training widget
 *
 *  Created: Thu Mar 20 20:53:35 2008
 *  Copyright  2006  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_
#define __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_

#include <fvutils/base/roi.h>
#include <fvutils/color/colorspaces.h>

#include <gtkmm.h>

class ColormapViewerWidget;
namespace firevision {
  class BayesColormapGenerator;
  class Zauberstab;
  class YuvColormap;
}

class ColorTrainWidget
{
 public:
	static const unsigned int MOUSE_BUTTON_LEFT = 1; /**< constant for left mouse button id */
	static const unsigned int MOUSE_BUTTON_RIGHT = 3; /**< constant for right mouse button id */

  ColorTrainWidget(Gtk::Window* parent);
  virtual ~ColorTrainWidget();

  void set_fg_object(firevision::hint_t fg_object);

  void set_src_buffer(unsigned char* buffer,
		      unsigned int img_width, unsigned int img_height);
  void set_draw_buffer(unsigned char* buffer);


  void click(unsigned int x, unsigned int y, unsigned int button = MOUSE_BUTTON_LEFT);
  void reset_selection();

  void load_histograms();
  void save_histograms();

  void add_to_colormap();
  void reset_colormap();
  void load_colormap();
  void save_colormap();
  firevision::YuvColormap* get_colormap() const;

  void draw_segmentation_result();

  void set_reset_selection_btn(Gtk::Button* btn);
  void set_add_to_colormap_btn(Gtk::Button* btn);
  void set_reset_colormap_btn(Gtk::Button* btn);
  void set_load_histos_btn(Gtk::Button* btn);
  void set_save_histos_btn(Gtk::Button* btn);
  void set_load_colormap_btn(Gtk::Button* btn);
  void set_save_colormap_btn(Gtk::Button* btn);
  void set_colormap_img(Gtk::Image* img);
  void set_segmentation_img(Gtk::Image* img);
  void set_threshold_scl(Gtk::Scale* scl);
  void set_min_prob_scl(Gtk::Scale* scl);
  void set_filechooser_dlg(Gtk::FileChooserDialog* dlg);
  void set_cm_layer_selector(Gtk::Scale* scl);
  void set_cm_selector(Gtk::SpinButton* depth, Gtk::SpinButton* width = 0, Gtk::SpinButton* height = 0);

  Glib::Dispatcher& update_image();
  Glib::Dispatcher& colormap_updated();

 private:
  void resize_seg_image(Gtk::Allocation& allocation);
  bool set_threshold(Gtk::ScrollType scroll, double value);
  bool set_min_prob(Gtk::ScrollType scroll, double value);
  static void free_rgb_buffer(const guint8* rgb_buffer);

  void reset_gui();

  firevision::BayesColormapGenerator* m_generator;
  firevision::Zauberstab* m_zauberstab;
  ColormapViewerWidget* m_cvw;

  firevision::hint_t m_fg_object;

  unsigned char* m_src_buffer;
  unsigned char* m_draw_buffer;
  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_img_size;
  firevision::colorspace_t m_img_cs;
  unsigned int m_seg_img_max_width;
  unsigned int m_seg_img_max_height;

  Gtk::Window* m_wnd_parent;
  Gtk::Button* m_btn_reset_selection;
  Gtk::Button* m_btn_add_to_colormap;
  Gtk::Button* m_btn_reset_colormap;
  Gtk::Button* m_btn_load_histos;
  Gtk::Button* m_btn_save_histos;
  Gtk::Button* m_btn_load_colormap;
  Gtk::Button* m_btn_save_colormap;
  Gtk::SpinButton* m_spbtn_cm_depth;
  Gtk::SpinButton* m_spbtn_cm_width;
  Gtk::SpinButton* m_spbtn_cm_height;
  Gtk::Image* m_img_segmentation;
  Gtk::Scale* m_scl_threshold;
  Gtk::Scale* m_scl_min_prob;
  Gtk::FileChooserDialog* m_fcd_filechooser;

  Glib::Dispatcher m_signal_update_image;
  Glib::Dispatcher m_signal_colormap_updated;
};

#endif /* __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_ */
