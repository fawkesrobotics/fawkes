
/***************************************************************************
 *  laser_drawing_area.h - Graph drawing area derived from Gtk::DrawingArea
 *
 *  Created: Wed Mar 18 10:38:07 2009
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _TOOLS_SKILLGUI_GRAPH_DRAWING_AREA_H_
#define _TOOLS_SKILLGUI_GRAPH_DRAWING_AREA_H_

#include <gtkmm.h>

#include <gvc.h>
#include <gvcjob.h>

#include "gvplugin_skillgui_cairo.h"

class SkillGuiGraphDrawingArea
: public Gtk::DrawingArea,
  public SkillGuiCairoRenderInstructor
{
 public:

  SkillGuiGraphDrawingArea();
  ~SkillGuiGraphDrawingArea();

  void save();
  void open();
  bool set_recording(bool recording);

  void zoom_in();
  void zoom_out();
  void zoom_fit();
  void zoom_reset();

  void set_graph_fsm(std::string fsm_name);
  void set_graph(std::string graph);

  void   set_bb(double bbw, double bbh);
  void   set_pad(double pad_x, double pad_y);
  void   set_translation(double tx, double ty);
  void   set_scale(double scale);
  bool   scale_override();
  double get_scale();
  void   get_translation(double &tx, double &ty);
  void   get_dimensions(double &width, double &height);
  void   get_pad(double &pad_x, double &pad_y);
  Cairo::RefPtr<Cairo::Context> get_cairo();

  bool get_update_graph();
  void set_update_graph(bool update);

  sigc::signal<void> signal_update_disabled();

 protected:
#if GTK_VERSION_GE(3,0)
  virtual bool on_draw(const Cairo::RefPtr<Cairo::Context> &cr);
#else
  virtual bool on_expose_event(GdkEventExpose* event);
#endif
  virtual bool on_scroll_event(GdkEventScroll *event);
  virtual bool on_button_press_event(GdkEventButton *event);
  virtual bool on_motion_notify_event(GdkEventMotion *event);

 private:
  void save_dotfile(const char *filename);

 private:
  Cairo::RefPtr<Cairo::Context> cairo_;
  Gtk::FileChooserDialog *fcd_save_;
  Gtk::FileChooserDialog *fcd_open_;
  Gtk::FileChooserDialog *fcd_recording_;
#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> filter_pdf_;
  Glib::RefPtr<Gtk::FileFilter> filter_svg_;
  Glib::RefPtr<Gtk::FileFilter> filter_png_;
  Glib::RefPtr<Gtk::FileFilter> filter_dot_;
#else
  Gtk::FileFilter *filter_pdf_;
  Gtk::FileFilter *filter_svg_;
  Gtk::FileFilter *filter_png_;
  Gtk::FileFilter *filter_dot_;
#endif

  sigc::signal<void> signal_update_disabled_;

  GVC_t *gvc_;

  std::string graph_fsm_;
  std::string graph_;
  std::string nonupd_graph_;
  std::string nonupd_graph_fsm_;

  double bbw_;
  double bbh_;
  double pad_x_;
  double pad_y_;
  double translation_x_;
  double translation_y_;
  double scale_;

  double last_mouse_x_;
  double last_mouse_y_;

  bool scale_override_;
  bool update_graph_;


  bool recording_;
  std::string record_directory_;
};

#endif
