
/***************************************************************************
 *  color_train_widget.h - Color training widget
 *
 *  Created: Thu Mar 20 20:53:35 2008
 *  Copyright  2006  Daniel Beck
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_
#define __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_

#include <fvutils/base/roi.h>
#include <fvutils/color/colorspaces.h>

#include <gtkmm.h>

class LutViewerWidget;
class BayesColormapGenerator;
class Zauberstab;
class YuvColormap;

class ColorTrainWidget
{
 public:
  ColorTrainWidget(Gtk::Window* parent);
  virtual ~ColorTrainWidget();

  void set_fg_object(hint_t fg_object);

  void set_src_buffer(unsigned char* buffer, 
		      unsigned int img_width, unsigned int img_height);
  void set_draw_buffer(unsigned char* buffer);


  void click(unsigned int x, unsigned int y);
  void reset_selection();
  
  void load_histograms();
  void save_histograms();

  void add_to_lut();
  void reset_lut();
  void load_lut();
  void save_lut();
  YuvColormap* get_colormap() const;

  void draw_segmentation_result();

  void set_reset_selection_btn(Gtk::Button* btn);
  void set_add_to_lut_btn(Gtk::Button* btn);
  void set_reset_lut_btn(Gtk::Button* btn);
  void set_load_histos_btn(Gtk::Button* btn);
  void set_save_histos_btn(Gtk::Button* btn);
  void set_load_lut_btn(Gtk::Button* btn);
  void set_save_lut_btn(Gtk::Button* btn);
  void set_lut_img(Gtk::Image* img);
  void set_segmentation_img(Gtk::Image* img);
  void set_threshold_scl(Gtk::Scale* scl);
  void set_min_prob_scl(Gtk::Scale* scl);
  void set_filechooser_dlg(Gtk::FileChooserDialog* dlg);
  void set_cm_layer_selector(Gtk::Scale* scl);
  void set_cm_depth_selector(Gtk::SpinButton* spbtn);
  
  void set_update_img_signal(Glib::Dispatcher* update_img);

 private:
  bool set_threshold(Gtk::ScrollType scroll, double value);
  bool set_min_prob(Gtk::ScrollType scroll, double value);
  bool set_cm_layer(Gtk::ScrollType scroll, double value);
  void set_cm_depth();

  void reset_gui();

  BayesColormapGenerator* m_generator;
  Zauberstab* m_zauberstab;
  LutViewerWidget* m_lvw;
  
  hint_t m_fg_object;

  unsigned char* m_src_buffer;
  unsigned char* m_draw_buffer;
  unsigned int m_img_width;
  unsigned int m_img_height;
  unsigned int m_img_size;
  colorspace_t m_img_cs;
  float m_img_ratio;

  Gtk::Window* m_wnd_parent;
  Gtk::Button* m_btn_reset_selection;
  Gtk::Button* m_btn_add_to_lut;
  Gtk::Button* m_btn_reset_lut;
  Gtk::Button* m_btn_load_histos;
  Gtk::Button* m_btn_save_histos;
  Gtk::Button* m_btn_load_lut;
  Gtk::Button* m_btn_save_lut;
  Gtk::SpinButton* m_spbtn_cm_depth;
  Gtk::Image* m_img_segmentation;
  Gtk::Scale* m_scl_threshold;
  Gtk::Scale* m_scl_min_prob;
  Gtk::Scale* m_scl_cm_layer_selector;
  Gtk::FileChooserDialog* m_fcd_filechooser;

  Glib::Dispatcher* m_update_img;
};

#endif /* __FIREVISION_TOOLS_FIRESTATION_COLOR_TRAIN_WIDGET_H_ */
