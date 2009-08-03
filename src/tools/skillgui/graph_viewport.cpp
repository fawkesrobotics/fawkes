
/***************************************************************************
 *  graph_viewport.cpp - FSM Graph Viewport for Skill GUI
 *
 *  Created: Mon Dec 15 15:40:36 2008
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "graph_viewport.h"
#include "gvplugin_skillgui_papyrus.h"

#include <gtk/gtk.h>

/** @class SkillGuiGraphViewport "graph_viewport.h"
 * Skill FSM Graph Viewport.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillGuiGraphViewport::SkillGuiGraphViewport()
{
  Cairo::RefPtr<Cairo::SolidPattern> bp = Cairo::SolidPattern::create_rgb(1, 1, 1);
  Papyrus::Paint::pointer pp = Papyrus::Paint::create(bp);

  Papyrus::Canvas::pointer c = canvas();
  c->set_scroll_anchor(Papyrus::SCROLL_ANCHOR_TOP_LEFT);
  c->set_background(pp);

  __affine = Papyrus::AffineController::create();
  __affine->insert(c);
  __translator = Papyrus::Translator::create();
  add_controller(__translator);

  __gvc = gvContext();
  __gvjob = NULL;

  __graph_fsm = "";
  __graph = "";

  __bbw = __bbh = __pad_x = __pad_y = 0.0;
  __translation_x = __translation_y = 0.0;
  __scale = 1.0;
  __update_graph = true;

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

  gvplugin_skillgui_setup(__gvc, this);

  signal_size_allocate().connect_notify(sigc::hide(sigc::mem_fun(*this, &SkillGuiGraphViewport::render)));
  signal_expose_event().connect_notify(sigc::mem_fun(*this, &SkillGuiGraphViewport::on_expose));
}


/** Destructor. */
SkillGuiGraphViewport::~SkillGuiGraphViewport()
{
  gvFreeContext(__gvc);
  delete __fcd;
}


/** Set current Graphviz job.
 * @param job current Graphviz job
 */
void
SkillGuiGraphViewport::set_gvjob(GVJ_t *job)
{
  __gvjob = job;
}


/** Set graph's FSM name.
 * @param fsm_name name of FSM the graph belongs to
 */
void
SkillGuiGraphViewport::set_graph_fsm(std::string fsm_name)
{
  if ( __graph_fsm != fsm_name ) {
    __translator->set_translate(0, 0);
  }
  __graph_fsm = fsm_name;
}


/** Set graph.
 * @param graph string representation of the current graph in the dot language.
 */
void
SkillGuiGraphViewport::set_graph(std::string graph)
{
  __graph = graph;
}


/** Add a drawable.
 * To be called only by the Graphviz plugin.
 * @param d drawable to add
 */
void
SkillGuiGraphViewport::add_drawable(Papyrus::Drawable::pointer d)
{
  canvas()->add(d);
  __translator->insert(d);
}


/** Clear all drawables.
 * To be called only by the Graphviz plugin.
 */
void
SkillGuiGraphViewport::clear()
{
  Papyrus::Gtk::Viewport::clear();
  __translator->clear();
}


/** Set bounding box.
 * To be called only by the Graphviz plugin.
 * @param bbw bounding box width
 * @param bbh bounding box height
 */
void
SkillGuiGraphViewport::set_bb(double bbw, double bbh)
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
SkillGuiGraphViewport::set_pad(double pad_x, double pad_y)
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
SkillGuiGraphViewport::set_translation(double tx, double ty)
{
  __translation_x = tx;
  __translation_y = ty;
}


/** Set scale.
 * To be called only by the Graphviz plugin.
 * @param scale scale value
 */
void
SkillGuiGraphViewport::set_scale(double scale)
{
  __scale = scale;
}

/** Check if graph is being updated.
 * @return true if the graph will be update if new data is received, false otherwise
 */
bool
SkillGuiGraphViewport::get_update_graph()
{
  return __update_graph;
}


/** Set if the graph should be updated on new data.
 * @param update true to update on new data, false to disable update
 */
void
SkillGuiGraphViewport::set_update_graph(bool update)
{
  __update_graph = update;
}


/** Zoom in.
 * Sets scale override and increases the scale by 0.1.
 */
void
SkillGuiGraphViewport::zoom_in()
{
  double sx, sy;
  Gtk::Allocation alloc = get_allocation();

  __affine->get_scale(sx, sy);
  sx += 0.1; sy += 0.1;
  __affine->set_scale(sx, sy);
  __affine->set_translate((alloc.get_width()  - __bbw * sx) / 2.0,
			  (alloc.get_height() - __bbh * sy) / 2.0);

  __scale_override = true;
}


/** Zoom out.
 * Sets scale override and decreases the scale by 0.1.
 */
void
SkillGuiGraphViewport::zoom_out()
{
  double sx, sy;
  __affine->get_scale(sx, sy);
  if ( (sx > 0.1) && (sy > 0.1) ) {
    Gtk::Allocation alloc = get_allocation();
    sx -= 0.1; sy -= 0.1;
    __affine->set_scale(sx, sy);
    __affine->set_translate((alloc.get_width()  - __bbw * sx) / 2.0,
			    (alloc.get_height() - __bbh * sy) / 2.0);
  }
  __scale_override = true;
}


/** Zoom to fit.
 * Disables scale override and draws with values suggested by Graphviz plugin.
 */
void
SkillGuiGraphViewport::zoom_fit()
{
  __affine->set_scale(__scale);
  __affine->set_translate(__pad_x + __translation_x, __pad_y + __translation_y);
  __translator->set_translate(0, 0);
  __scale_override = false;
}


/** Zoom reset.
 * Reset zoom to 1. Enables scale override.
 */
void
SkillGuiGraphViewport::zoom_reset()
{
  __affine->set_scale(1.0);
  if ( __scale == 1.0 ) {
    __affine->set_translate(__pad_x + __translation_x, __pad_y + __translation_y);
  } else {
    __affine->set_translate(__pad_x, __pad_y);
  }
  __scale_override = true;
}


/** Check if scale override is enabled.
 * @return true if scale override is enabled, false otherwise
 */
bool
SkillGuiGraphViewport::scale_override()
{
  return __scale_override;
}


/** Get scaler.
 * @return scaler controller
 */
Papyrus::AffineController::pointer
SkillGuiGraphViewport::get_affine()
{
  return __affine;
}

/** Render current graph. */
void
SkillGuiGraphViewport::save()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());

  int result = __fcd->run();
  if (result == Gtk::RESPONSE_OK) {

    double old_scale_x, old_scale_y, old_translate_x, old_translate_y;
    __affine->get_scale(old_scale_x, old_scale_y);
    __affine->get_translate(old_translate_x, old_translate_y);
    __affine->set_scale(1);
    __affine->set_translate(__pad_x, __pad_y);

    Papyrus::Canvas::pointer c = canvas();

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
	surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, __bbw, __bbh);
	write_to_png = true;
      }

      if (surface) {
	Cairo::RefPtr<Cairo::Context> context = Cairo::Context::create(surface);
	c->render(context);
	if (write_to_png) {
	  surface->write_to_png(filename);
	}
      }

    } else {
      Gtk::MessageDialog md(*w, "Invalid filename",
			    /* markup */ false, Gtk::MESSAGE_ERROR,
			    Gtk::BUTTONS_OK, /* modal */ true);
      md.set_title("Invalid File Name");
      md.run();
    }

    __affine->set_scale(old_scale_x, old_scale_y);
    __affine->set_translate(old_translate_x, old_translate_y);
  }

  __fcd->hide();
}


/** Render current graph. */
void
SkillGuiGraphViewport::render()
{
  if (!  __update_graph)  return;

  Papyrus::Canvas::pointer c = canvas();
#ifdef HAVE_TIMS_PAPYRUS_PATCHES
  c->set_redraw_enabled(false);
#endif
  Agraph_t *g = agmemread((char *)__graph.c_str());
  if (g) {
    gvLayout(__gvc, g, (char *)"dot");
    gvRender(__gvc, g, (char *)"skillgui", NULL);
    gvFreeLayout(__gvc, g);
    agclose(g);
  } else {
    clear();
  }
#ifdef HAVE_TIMS_PAPYRUS_PATCHES
  c->set_redraw_enabled(true);
#endif
}


/** Called on explose.
 * @param event Gdk event structure
 */
void
SkillGuiGraphViewport::on_expose(GdkEventExpose *event)
{
  if (__scale_override) {
    Gtk::Allocation alloc = get_allocation();

    double sx, sy;
    __affine->get_scale(sx, sy);
    __affine->set_translate(((alloc.get_width()  - __bbw * sx) / 2.0) + __pad_x,
			    ((alloc.get_height() - __bbh * sy) / 2.0) + __pad_y);
    
  }
}
