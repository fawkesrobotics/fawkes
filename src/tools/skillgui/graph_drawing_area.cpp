
/***************************************************************************
 *  graph_drawing_area.cpp - Graph drawing area derived from Gtk::DrawingArea
 *
 *  Created: Wed Mar 18 10:40:00 2009
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "graph_drawing_area.h"
#include "gvplugin_skillgui_cairo.h"

#include <cmath>

/** @class SkillGuiGraphDrawingArea "graph_drawing_area.h"
 * Graph drawing area.
 * Derived version of Gtk::DrawingArea that renders a graph via Graphviz.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillGuiGraphDrawingArea::SkillGuiGraphDrawingArea()
{
  add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK);

  __gvc = gvContext();
  __gvjob = NULL;

  __graph_fsm = "";
  __graph = "";

  __bbw = __bbh = __pad_x = __pad_y = 0.0;
  __translation_x = __translation_y = 0.0;
  __scale = 1.0;
  __scale_override = false;
  __update_graph = true;

  gvplugin_skillgui_cairo_setup(__gvc, this);

  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  if (w) {
    __fcd = new Gtk::FileChooserDialog(*w, "Save Graph",
				       Gtk::FILE_CHOOSER_ACTION_SAVE);
    __fcd->set_transient_for(*w);
  } else {
    __fcd = new Gtk::FileChooserDialog("Save Graph",
				       Gtk::FILE_CHOOSER_ACTION_SAVE);
  }
  //Add response buttons the the dialog:
  __fcd->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  __fcd->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);

  Gtk::FileFilter *filter_pdf = Gtk::manage(new Gtk::FileFilter());
  filter_pdf->set_name("Portable Document Format (PDF)");
  filter_pdf->add_pattern("*.pdf");
  Gtk::FileFilter *filter_svg = Gtk::manage(new Gtk::FileFilter());;
  filter_svg->set_name("Scalable Vector Graphic (SVG)");
  filter_svg->add_pattern("*.svg");
  Gtk::FileFilter *filter_png = Gtk::manage(new Gtk::FileFilter());;
  filter_png->set_name("Portable Network Graphic (PNG)");
  filter_png->add_pattern("*.png");
  __fcd->add_filter(*filter_pdf);
  __fcd->add_filter(*filter_svg);
  __fcd->add_filter(*filter_png);
  __fcd->set_filter(*filter_pdf);

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  signal_expose_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_expose_event));
  signal_button_press_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_button_press_event));
  signal_motion_notify_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_motion_notify_event));
#endif
}

SkillGuiGraphDrawingArea::~SkillGuiGraphDrawingArea()
{
  gvFreeContext(__gvc);
  //delete __fcd;
}

/** Set current Graphviz job.
 * @param job current Graphviz job
 */
void
SkillGuiGraphDrawingArea::set_gvjob(GVJ_t *job)
{
  __gvjob = job;
}


/** Set graph's FSM name.
 * @param fsm_name name of FSM the graph belongs to
 */
void
SkillGuiGraphDrawingArea::set_graph_fsm(std::string fsm_name)
{
  /*
  if ( __graph_fsm != fsm_name ) {
    __translator->set_translate(0, 0);
  }
  */
  if ( __update_graph ) {
    if ( __graph_fsm != fsm_name ) {
      __scale_override = false;
    }
    __graph_fsm = fsm_name;
  } else {
    __nonupd_graph_fsm = fsm_name;
  }
}


/** Set graph.
 * @param graph string representation of the current graph in the dot language.
 */
void
SkillGuiGraphDrawingArea::set_graph(std::string graph)
{
  if ( __update_graph ) {
    __graph = graph;
    queue_draw();
  } else {
    __nonupd_graph = graph;
  }
}

/** Set bounding box.
 * To be called only by the Graphviz plugin.
 * @param bbw bounding box width
 * @param bbh bounding box height
 */
void
SkillGuiGraphDrawingArea::set_bb(double bbw, double bbh)
{
  __bbw = bbw;
  __bbh = bbh;
}


/** Set padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x padding in x
 * @param pad_y padding in y
 */
void
SkillGuiGraphDrawingArea::set_pad(double pad_x, double pad_y)
{
  __pad_x = pad_x;
  __pad_y = pad_y;
}


/** Set translation.
 * To be called only by the Graphviz plugin.
 * @param tx translation in x
 * @param ty translation in y
 */
void
SkillGuiGraphDrawingArea::set_translation(double tx, double ty)
{
  __translation_x = tx;
  __translation_y = ty;
}


/** Set scale.
 * To be called only by the Graphviz plugin.
 * @param scale scale value
 */
void
SkillGuiGraphDrawingArea::set_scale(double scale)
{
  __scale = scale;
}

/** Get scale.
 * To be called only by the Graphviz plugin.
 * @return scale value
 */
double
SkillGuiGraphDrawingArea::get_scale()
{
  return __scale;
}

/** Get translation.
 * @param tx upon return contains translation value
 * @param ty upon return contains translation value
 */
void
SkillGuiGraphDrawingArea::get_translation(double &tx, double &ty)
{
  tx = __translation_x;
  ty = __translation_y;
}


/** Zoom in.
 * Increases zoom factor by 20, no upper limit.
 */
void
SkillGuiGraphDrawingArea::zoom_in()
{
  Gtk::Allocation alloc = get_allocation();
  __scale += 0.1;
  __scale_override = true;
  __translation_x = (alloc.get_width()  - __bbw * __scale) / 2.0;
  __translation_y = (alloc.get_height() - __bbh * __scale) / 2.0 + __bbh * __scale;
  queue_draw();
}

/** Zoom out.
 * Decreases zoom factor by 20 with a minimum of 1.
 */
void
SkillGuiGraphDrawingArea::zoom_out()
{
  __scale_override = true;
  if ( __scale > 0.1 ) {
    Gtk::Allocation alloc = get_allocation();
    __scale -= 0.1;
    __translation_x = (alloc.get_width()  - __bbw * __scale) / 2.0;
    __translation_y = (alloc.get_height() - __bbh * __scale) / 2.0 + __bbh * __scale;
    queue_draw();
  }
}


/** Zoom to fit.
 * Disables scale override and draws with values suggested by Graphviz plugin.
 */
void
SkillGuiGraphDrawingArea::zoom_fit()
{
  __scale_override = false;
  queue_draw();
}


/** Zoom reset.
 * Reset zoom to 1. Enables scale override.
 */
void
SkillGuiGraphDrawingArea::zoom_reset()
{
  Gtk::Allocation alloc = get_allocation();
  __scale = 1.0;
  __scale_override = true;
  __translation_x = (alloc.get_width()  - __bbw) / 2.0;
  __translation_y = (alloc.get_height() - __bbh) / 2.0 + __bbh;
  queue_draw();
}


/** Check if scale override is enabled.
 * @return true if scale override is enabled, false otherwise
 */
bool
SkillGuiGraphDrawingArea::scale_override()
{
  return __scale_override;
}


/** Get Cairo context.
 * This is only valid during the expose event and is only meant for the
 * Graphviz plugin.
 * @return Cairo context
 */
Cairo::RefPtr<Cairo::Context>
SkillGuiGraphDrawingArea::get_cairo()
{
  return __cairo;
}



/** Check if graph is being updated.
 * @return true if the graph will be update if new data is received, false otherwise
 */
bool
SkillGuiGraphDrawingArea::get_update_graph()
{
  return __update_graph;
}


/** Set if the graph should be updated on new data.
 * @param update true to update on new data, false to disable update
 */
void
SkillGuiGraphDrawingArea::set_update_graph(bool update)
{
  if (update && ! __update_graph) {
    if ( __graph_fsm != __nonupd_graph_fsm ) {
      __scale_override = false;
    }
    __graph     = __nonupd_graph;
    __graph_fsm = __nonupd_graph_fsm;
    queue_draw();
  }
  __update_graph = update;
}


/** save current graph. */
void
SkillGuiGraphDrawingArea::save()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());

  int result = __fcd->run();
  if (result == Gtk::RESPONSE_OK) {
    Cairo::RefPtr<Cairo::Surface> surface;

    std::string filename = __fcd->get_filename();
    bool write_to_png = false;
    if (filename != "") {
      Gtk::FileFilter *f = __fcd->get_filter();
      if (f->get_name().find("PDF") != Glib::ustring::npos) {
	surface = Cairo::PdfSurface::create(filename, __bbw, __bbh);
      } else if (f->get_name().find("SVG") != Glib::ustring::npos) {
	surface = Cairo::SvgSurface::create(filename, __bbw, __bbh);
      } else if (f->get_name().find("PNG") != Glib::ustring::npos) {
	surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32,
					      (int)ceilf(__bbw),
					      (int)ceilf(__bbh));
	write_to_png = true;
      }

      if (surface) {
	__cairo = Cairo::Context::create(surface);

	bool old_scale_override = __scale_override;
	double old_tx = __translation_x;
	double old_ty = __translation_y;
	double old_scale = __scale;
	__translation_x = __pad_x;
	__translation_y = __bbh - __pad_y;
	__scale = 1.0;
	__scale_override = true;

	Agraph_t *g = agmemread((char *)__graph.c_str());
	if (g) {
	  gvLayout(__gvc, g, (char *)"dot");
	  gvRender(__gvc, g, (char *)"skillguicairo", NULL);
	  gvFreeLayout(__gvc, g);
	  agclose(g);
	}

	if (write_to_png) {
	  surface->write_to_png(filename);
	}

	__cairo.clear();

	__translation_x = old_tx;
	__translation_y = old_ty;
	__scale = old_scale;
	__scale_override = old_scale_override;
      }

    } else {
      Gtk::MessageDialog md(*w, "Invalid filename",
			    /* markup */ false, Gtk::MESSAGE_ERROR,
			    Gtk::BUTTONS_OK, /* modal */ true);
      md.set_title("Invalid File Name");
      md.run();
    }
  }

  __fcd->hide();
}


/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
SkillGuiGraphDrawingArea::on_expose_event(GdkEventExpose* event)
{
  // This is where we draw on the window
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window) {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    // coordinates for the center of the window
    int xc, yc;
    xc = width / 2;
    yc = height / 2;
    
    __cairo = window->create_cairo_context();
    __cairo->set_source_rgb(1, 1, 1);
    __cairo->paint();

    Agraph_t *g = agmemread((char *)__graph.c_str());
    if (g) {
      gvLayout(__gvc, g, (char *)"dot");
      gvRender(__gvc, g, (char *)"skillguicairo", NULL);
      gvFreeLayout(__gvc, g);
      agclose(g);
    }

    __cairo.clear();
  }    

  return true;
}

/** Scroll event handler.
 * @param event event structure
 * @return signal return value
 */
bool
SkillGuiGraphDrawingArea::on_scroll_event(GdkEventScroll *event)
{
  if (event->direction == GDK_SCROLL_UP) {
    zoom_in();
  } else if (event->direction == GDK_SCROLL_DOWN) {
    zoom_out();
  }
  return true;
}


/** Button press event handler.
 * @param event event data
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_button_press_event(GdkEventButton *event)
{
  __last_mouse_x = event->x;
  __last_mouse_y = event->y;
  return true;
}


/** Mouse motion notify event handler.
 * @param event event data
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_motion_notify_event(GdkEventMotion *event)
{
  __scale_override = true;
  __translation_x -= __last_mouse_x - event->x;
  __translation_y -= __last_mouse_y - event->y;
  __last_mouse_x = event->x;
  __last_mouse_y = event->y;
  queue_draw();
  return true;
}

