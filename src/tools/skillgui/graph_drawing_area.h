
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

#ifndef __TOOLS_SKILLGUI_GRAPH_DRAWING_AREA_H_
#define __TOOLS_SKILLGUI_GRAPH_DRAWING_AREA_H_

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
  Cairo::RefPtr<Cairo::Context> __cairo;
  Gtk::FileChooserDialog *__fcd_save;
  Gtk::FileChooserDialog *__fcd_open;
  Gtk::FileChooserDialog *__fcd_recording;
#if GTK_VERSION_GE(3,0)
  Glib::RefPtr<Gtk::FileFilter> __filter_pdf;
  Glib::RefPtr<Gtk::FileFilter> __filter_svg;
  Glib::RefPtr<Gtk::FileFilter> __filter_png;
  Glib::RefPtr<Gtk::FileFilter> __filter_dot;
#else
  Gtk::FileFilter *__filter_pdf;
  Gtk::FileFilter *__filter_svg;
  Gtk::FileFilter *__filter_png;
  Gtk::FileFilter *__filter_dot;
#endif

  sigc::signal<void> __signal_update_disabled;

  GVC_t *__gvc;

  std::string __graph_fsm;
  std::string __graph;
  std::string __nonupd_graph;
  std::string __nonupd_graph_fsm;

  double __bbw;
  double __bbh;
  double __pad_x;
  double __pad_y;
  double __translation_x;
  double __translation_y;
  double __scale;

  double __last_mouse_x;
  double __last_mouse_y;

  bool __scale_override;
  bool __update_graph;


  bool __recording;
  std::string __record_directory;
};

#endif
