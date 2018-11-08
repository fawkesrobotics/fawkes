
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

  affine_ = Papyrus::AffineController::create();
  affine_->insert(c);
  translator_ = Papyrus::Translator::create();
  add_controller(translator_);

  gvc_ = gvContext();
  gvjob_ = NULL;

  graph_fsm_ = "";
  graph_ = "";

  bbw_ = bbh_ = pad_x_ = pad_y_ = 0.0;
  translation_x_ = translation_y_ = 0.0;
  scale_ = 1.0;
  update_graph_ = true;

  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  if (w) {
    fcd_ = new Gtk::FileChooserDialog(*w, "Save Graph",
				       Gtk::FILE_CHOOSER_ACTION_SAVE);
    fcd_->set_transient_for(*w);
  } else {
    fcd_ = new Gtk::FileChooserDialog("Save Graph",
				       Gtk::FILE_CHOOSER_ACTION_SAVE);
  }

  //Add response buttons the the dialog:
  fcd_->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  fcd_->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);

  Gtk::FileFilter *filter_pdf = Gtk::manage(new Gtk::FileFilter());
  filter_pdf->set_name("Portable Document Format (PDF)");
  filter_pdf->add_pattern("*.pdf");
  Gtk::FileFilter *filter_svg = Gtk::manage(new Gtk::FileFilter());;
  filter_svg->set_name("Scalable Vector Graphic (SVG)");
  filter_svg->add_pattern("*.svg");
  Gtk::FileFilter *filter_png = Gtk::manage(new Gtk::FileFilter());;
  filter_png->set_name("Portable Network Graphic (PNG)");
  filter_png->add_pattern("*.png");
  fcd_->add_filter(*filter_pdf);
  fcd_->add_filter(*filter_svg);
  fcd_->add_filter(*filter_png);
  fcd_->set_filter(*filter_pdf);

  gvplugin_skillgui_setup(gvc_, this);

  signal_size_allocate().connect_notify(sigc::hide(sigc::mem_fun(*this, &SkillGuiGraphViewport::render)));
  signal_expose_event().connect_notify(sigc::mem_fun(*this, &SkillGuiGraphViewport::on_expose));
}


/** Destructor. */
SkillGuiGraphViewport::~SkillGuiGraphViewport()
{
  gvFreeContext(gvc_);
  delete fcd_;
}


/** Set current Graphviz job.
 * @param job current Graphviz job
 */
void
SkillGuiGraphViewport::set_gvjob(GVJ_t *job)
{
  gvjob_ = job;
}


/** Set graph's FSM name.
 * @param fsm_name name of FSM the graph belongs to
 */
void
SkillGuiGraphViewport::set_graph_fsm(std::string fsm_name)
{
  if ( graph_fsm_ != fsm_name ) {
    translator_->set_translate(0, 0);
  }
  graph_fsm_ = fsm_name;
}


/** Set graph.
 * @param graph string representation of the current graph in the dot language.
 */
void
SkillGuiGraphViewport::set_graph(std::string graph)
{
  graph_ = graph;
}


/** Add a drawable.
 * To be called only by the Graphviz plugin.
 * @param d drawable to add
 */
void
SkillGuiGraphViewport::add_drawable(Papyrus::Drawable::pointer d)
{
  canvas()->add(d);
  translator_->insert(d);
}


/** Clear all drawables.
 * To be called only by the Graphviz plugin.
 */
void
SkillGuiGraphViewport::clear()
{
  Papyrus::Gtk::Viewport::clear();
  translator_->clear();
}


/** Set bounding box.
 * To be called only by the Graphviz plugin.
 * @param bbw bounding box width
 * @param bbh bounding box height
 */
void
SkillGuiGraphViewport::set_bb(double bbw, double bbh)
{
  bbw_ = bbw;
  bbh_ = bbh;
}


/** Set padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x padding in x
 * @param pad_y padding in y
 */
void
SkillGuiGraphViewport::set_pad(double pad_x, double pad_y)
{
  pad_x_ = pad_x;
  pad_y_ = pad_y;
}


/** Set translation.
 * To be called only by the Graphviz plugin.
 * @param tx translation in x
 * @param ty translation in y
 */
void
SkillGuiGraphViewport::set_translation(double tx, double ty)
{
  translation_x_ = tx;
  translation_y_ = ty;
}


/** Set scale.
 * To be called only by the Graphviz plugin.
 * @param scale scale value
 */
void
SkillGuiGraphViewport::set_scale(double scale)
{
  scale_ = scale;
}

/** Check if graph is being updated.
 * @return true if the graph will be update if new data is received, false otherwise
 */
bool
SkillGuiGraphViewport::get_update_graph()
{
  return update_graph_;
}


/** Set if the graph should be updated on new data.
 * @param update true to update on new data, false to disable update
 */
void
SkillGuiGraphViewport::set_update_graph(bool update)
{
  update_graph_ = update;
}


/** Zoom in.
 * Sets scale override and increases the scale by 0.1.
 */
void
SkillGuiGraphViewport::zoom_in()
{
  double sx, sy;
  Gtk::Allocation alloc = get_allocation();

  affine_->get_scale(sx, sy);
  sx += 0.1; sy += 0.1;
  affine_->set_scale(sx, sy);
  affine_->set_translate((alloc.get_width()  - bbw_ * sx) / 2.0,
			  (alloc.get_height() - bbh_ * sy) / 2.0);

  scale_override_ = true;
}


/** Zoom out.
 * Sets scale override and decreases the scale by 0.1.
 */
void
SkillGuiGraphViewport::zoom_out()
{
  double sx, sy;
  affine_->get_scale(sx, sy);
  if ( (sx > 0.1) && (sy > 0.1) ) {
    Gtk::Allocation alloc = get_allocation();
    sx -= 0.1; sy -= 0.1;
    affine_->set_scale(sx, sy);
    affine_->set_translate((alloc.get_width()  - bbw_ * sx) / 2.0,
			    (alloc.get_height() - bbh_ * sy) / 2.0);
  }
  scale_override_ = true;
}


/** Zoom to fit.
 * Disables scale override and draws with values suggested by Graphviz plugin.
 */
void
SkillGuiGraphViewport::zoom_fit()
{
  affine_->set_scale(scale_);
  affine_->set_translate(pad_x_ + translation_x_, pad_y_ + translation_y_);
  translator_->set_translate(0, 0);
  scale_override_ = false;
}


/** Zoom reset.
 * Reset zoom to 1. Enables scale override.
 */
void
SkillGuiGraphViewport::zoom_reset()
{
  affine_->set_scale(1.0);
  if ( scale_ == 1.0 ) {
    affine_->set_translate(pad_x_ + translation_x_, pad_y_ + translation_y_);
  } else {
    affine_->set_translate(pad_x_, pad_y_);
  }
  scale_override_ = true;
}


/** Check if scale override is enabled.
 * @return true if scale override is enabled, false otherwise
 */
bool
SkillGuiGraphViewport::scale_override()
{
  return scale_override_;
}


/** Get scaler.
 * @return scaler controller
 */
Papyrus::AffineController::pointer
SkillGuiGraphViewport::get_affine()
{
  return affine_;
}

/** Render current graph. */
void
SkillGuiGraphViewport::save()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());

  int result = fcd_->run();
  if (result == Gtk::RESPONSE_OK) {

    double old_scale_x, old_scale_y, old_translate_x, old_translate_y;
    affine_->get_scale(old_scale_x, old_scale_y);
    affine_->get_translate(old_translate_x, old_translate_y);
    affine_->set_scale(1);
    affine_->set_translate(pad_x_, pad_y_);

    Papyrus::Canvas::pointer c = canvas();

    Cairo::RefPtr<Cairo::Surface> surface;

    std::string filename = fcd_->get_filename();
    bool write_to_png = false;
    if (filename != "") {
      Gtk::FileFilter *f = fcd_->get_filter();
      if (f->get_name().find("PDF") != Glib::ustring::npos) {
	surface = Cairo::PdfSurface::create(filename, bbw_, bbh_);
      } else if (f->get_name().find("SVG") != Glib::ustring::npos) {
	surface = Cairo::SvgSurface::create(filename, bbw_, bbh_);
      } else if (f->get_name().find("PNG") != Glib::ustring::npos) {
	surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32, bbw_, bbh_);
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

    affine_->set_scale(old_scale_x, old_scale_y);
    affine_->set_translate(old_translate_x, old_translate_y);
  }

  fcd_->hide();
}


/** Render current graph. */
void
SkillGuiGraphViewport::render()
{
  if (!  update_graph_)  return;

  Papyrus::Canvas::pointer c = canvas();
#ifdef HAVE_TIMS_PAPYRUS_PATCHES
  c->set_redraw_enabled(false);
#endif
  Agraph_t *g = agmemread((char *)graph_.c_str());
  if (g) {
    gvLayout(gvc_, g, (char *)"dot");
    gvRender(gvc_, g, (char *)"skillgui", NULL);
    gvFreeLayout(gvc_, g);
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
  if (scale_override_) {
    Gtk::Allocation alloc = get_allocation();

    double sx, sy;
    affine_->get_scale(sx, sy);
    affine_->set_translate(((alloc.get_width()  - bbw_ * sx) / 2.0) + pad_x_,
			    ((alloc.get_height() - bbh_ * sy) / 2.0) + pad_y_);
    
  }
}
