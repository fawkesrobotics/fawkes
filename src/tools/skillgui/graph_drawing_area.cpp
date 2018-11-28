
/***************************************************************************
 *  graph_drawing_area.cpp - Graph drawing area derived from Gtk::DrawingArea
 *
 *  Created: Wed Mar 18 10:40:00 2009
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

#include "graph_drawing_area.h"
#include "gvplugin_skillgui_cairo.h"

#include <cmath>
#include <libgen.h>
#include <sys/time.h>

/** @class SkillGuiGraphDrawingArea "graph_drawing_area.h"
 * Graph drawing area.
 * Derived version of Gtk::DrawingArea that renders a graph via Graphviz.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillGuiGraphDrawingArea::SkillGuiGraphDrawingArea()
{
  add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK);

  gvc_ = gvContext();

  graph_fsm_ = "";
  graph_ = "";

  bbw_ = bbh_ = pad_x_ = pad_y_ = 0.0;
  translation_x_ = translation_y_ = 0.0;
  scale_ = 1.0;
  scale_override_ = false;
  update_graph_ = true;
  recording_ = false;

  gvplugin_skillgui_cairo_setup(gvc_, this);

  fcd_save_ = new Gtk::FileChooserDialog("Save Graph",
					  Gtk::FILE_CHOOSER_ACTION_SAVE);
  fcd_open_ = new Gtk::FileChooserDialog("Load Graph",
					  Gtk::FILE_CHOOSER_ACTION_OPEN);
  fcd_recording_ = new Gtk::FileChooserDialog("Recording Directory",
						 Gtk::FILE_CHOOSER_ACTION_CREATE_FOLDER);

  //Add response buttons the the dialog:
  fcd_save_->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  fcd_save_->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);
  fcd_open_->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  fcd_open_->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);
  fcd_recording_->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  fcd_recording_->add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);

#if GTK_VERSION_GE(3,0)
  filter_pdf_ = Gtk::FileFilter::create();
  filter_svg_ = Gtk::FileFilter::create();
  filter_png_ = Gtk::FileFilter::create();
  filter_dot_ = Gtk::FileFilter::create();
#else
  filter_pdf_ = new Gtk::FileFilter();
  filter_svg_ = new Gtk::FileFilter();
  filter_png_ = new Gtk::FileFilter();
  filter_dot_ = new Gtk::FileFilter();
#endif
  filter_pdf_->set_name("Portable Document Format (PDF)");
  filter_pdf_->add_pattern("*.pdf");
  filter_svg_->set_name("Scalable Vector Graphic (SVG)");
  filter_svg_->add_pattern("*.svg");
  filter_png_->set_name("Portable Network Graphic (PNG)");
  filter_png_->add_pattern("*.png");
  filter_dot_->set_name("DOT Graph");
  filter_dot_->add_pattern("*.dot");
#if GTK_VERSION_GE(3,0)
  fcd_save_->add_filter(filter_pdf_);
  fcd_save_->add_filter(filter_svg_);
  fcd_save_->add_filter(filter_png_);
  fcd_save_->add_filter(filter_dot_);
  fcd_save_->set_filter(filter_pdf_);

  fcd_open_->add_filter(filter_dot_);
  fcd_open_->set_filter(filter_dot_);
#else
  fcd_save_->add_filter(*filter_pdf_);
  fcd_save_->add_filter(*filter_svg_);
  fcd_save_->add_filter(*filter_png_);
  fcd_save_->add_filter(*filter_dot_);
  fcd_save_->set_filter(*filter_pdf_);

  fcd_open_->add_filter(*filter_dot_);
  fcd_open_->set_filter(*filter_dot_);
#endif

  add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK |
	     Gdk::BUTTON_PRESS_MASK );

#if GTK_VERSION_LT(3,0)
  signal_expose_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_expose_event));
#endif
  signal_button_press_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_button_press_event));
  signal_motion_notify_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_motion_notify_event));
}

SkillGuiGraphDrawingArea::~SkillGuiGraphDrawingArea()
{
  gvFreeContext(gvc_);
  //delete fcd_;
  delete fcd_save_;
  delete fcd_open_;
  delete fcd_recording_;
#if GTK_VERSION_GE(3,0)
  filter_pdf_.reset();
  filter_svg_.reset();
  filter_png_.reset();
  filter_dot_.reset();
#else
  delete filter_pdf_;
  delete filter_svg_;
  delete filter_png_;
  delete filter_dot_;
#endif
}


/** Get "update disabled" signal.
 * @return "update disabled" signal
 */
sigc::signal<void>
SkillGuiGraphDrawingArea::signal_update_disabled()
{
  return signal_update_disabled_;
}


/** Set graph's FSM name.
 * @param fsm_name name of FSM the graph belongs to
 */
void
SkillGuiGraphDrawingArea::set_graph_fsm(const std::string& fsm_name)
{
  if ( update_graph_ ) {
    if ( graph_fsm_ != fsm_name ) {
      scale_override_ = false;
    }
    graph_fsm_ = fsm_name;
  } else {
    nonupd_graph_fsm_ = fsm_name;
  }
}


/** Set graph.
 * @param graph string representation of the current graph in the dot language.
 */
void
SkillGuiGraphDrawingArea::set_graph(const std::string& graph)
{
  if ( update_graph_ ) {
    graph_ = graph;
    queue_draw();
  } else {
    nonupd_graph_ = graph;
  }

  if ( recording_ ) {
    char *tmp;
#if defined(__MACH__) && defined(__APPLE__)
    struct timeval t;
    if (gettimeofday(&t, NULL) == 0) {
      long int nsec = t.tv_usec * 1000;
#else
    timespec t;
    if (clock_gettime(CLOCK_REALTIME, &t) == 0) {
      long int nsec = t.tv_nsec;
#endif
      struct tm tms;
      localtime_r(&t.tv_sec, &tms);

      if ( asprintf(&tmp, "%s/%s_%04i%02i%02i-%02i%02i%02i.%09li.dot",
		    record_directory_.c_str(), graph_fsm_.c_str(),
		    tms.tm_year + 1900, tms.tm_mon + 1, tms.tm_mday,
		    tms.tm_hour, tms.tm_min, tms.tm_sec, nsec) != -1)
      {

	//printf("Would record to filename %s\n", tmp);
	save_dotfile(tmp);
	free(tmp);
      } else {
	printf("Warning: Could not create file name for recording, skipping graph\n");
      }
    } else {
      printf("Warning: Could not time recording, skipping graph\n");
    }
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
  bbw_ = bbw;
  bbh_ = bbh;
}


/** Set padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x padding in x
 * @param pad_y padding in y
 */
void
SkillGuiGraphDrawingArea::set_pad(double pad_x, double pad_y)
{
  pad_x_ = pad_x;
  pad_y_ = pad_y;
}


/** Get padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x upon return contains padding in x
 * @param pad_y upon return contains padding in y
 */
void
SkillGuiGraphDrawingArea::get_pad(double &pad_x, double &pad_y)
{
  if (scale_override_) {
    pad_x = pad_y = 0;
  } else {
    pad_x = pad_x_;
    pad_y = pad_y_;
  }
}


/** Set translation.
 * To be called only by the Graphviz plugin.
 * @param tx translation in x
 * @param ty translation in y
 */
void
SkillGuiGraphDrawingArea::set_translation(double tx, double ty)
{
  translation_x_ = tx;
  translation_y_ = ty;
}


/** Set scale.
 * To be called only by the Graphviz plugin.
 * @param scale scale value
 */
void
SkillGuiGraphDrawingArea::set_scale(double scale)
{
  scale_ = scale;
}

/** Get scale.
 * To be called only by the Graphviz plugin.
 * @return scale value
 */
double
SkillGuiGraphDrawingArea::get_scale()
{
  return scale_;
}

/** Get translation.
 * @param tx upon return contains translation value
 * @param ty upon return contains translation value
 */
void
SkillGuiGraphDrawingArea::get_translation(double &tx, double &ty)
{
  tx = translation_x_;
  ty = translation_y_;
}


/** Get dimensions
 * @param width upon return contains width
 * @param height upon return contains height
 */
void
SkillGuiGraphDrawingArea::get_dimensions(double &width, double &height)
{
  Gtk::Allocation alloc = get_allocation();
  width  = alloc.get_width();
  height = alloc.get_height();
}


/** Zoom in.
 * Increases zoom factor by 20, no upper limit.
 */
void
SkillGuiGraphDrawingArea::zoom_in()
{
  Gtk::Allocation alloc = get_allocation();
  scale_ += 0.1;
  scale_override_ = true;
  translation_x_ = (alloc.get_width()  - bbw_ * scale_) / 2.0;
  translation_y_ = (alloc.get_height() - bbh_ * scale_) / 2.0 + bbh_ * scale_;
  queue_draw();
}

/** Zoom out.
 * Decreases zoom factor by 20 with a minimum of 1.
 */
void
SkillGuiGraphDrawingArea::zoom_out()
{
  scale_override_ = true;
  if ( scale_ > 0.1 ) {
    Gtk::Allocation alloc = get_allocation();
    scale_ -= 0.1;
    translation_x_ = (alloc.get_width()  - bbw_ * scale_) / 2.0;
    translation_y_ = (alloc.get_height() - bbh_ * scale_) / 2.0 + bbh_ * scale_;
    queue_draw();
  }
}


/** Zoom to fit.
 * Disables scale override and draws with values suggested by Graphviz plugin.
 */
void
SkillGuiGraphDrawingArea::zoom_fit()
{
  scale_override_ = false;
  queue_draw();
}


/** Zoom reset.
 * Reset zoom to 1. Enables scale override.
 */
void
SkillGuiGraphDrawingArea::zoom_reset()
{
  Gtk::Allocation alloc = get_allocation();
  scale_ = 1.0;
  scale_override_ = true;
  translation_x_ = (alloc.get_width()  - bbw_) / 2.0 + pad_x_;
  translation_y_ = (alloc.get_height() - bbh_) / 2.0 + bbh_ - pad_y_;
  queue_draw();
}


/** Check if scale override is enabled.
 * @return true if scale override is enabled, false otherwise
 */
bool
SkillGuiGraphDrawingArea::scale_override()
{
  return scale_override_;
}


/** Get Cairo context.
 * This is only valid during the expose event and is only meant for the
 * Graphviz plugin.
 * @return Cairo context
 */
Cairo::RefPtr<Cairo::Context>
SkillGuiGraphDrawingArea::get_cairo()
{
  return cairo_;
}



/** Check if graph is being updated.
 * @return true if the graph will be update if new data is received, false otherwise
 */
bool
SkillGuiGraphDrawingArea::get_update_graph()
{
  return update_graph_;
}


/** Set if the graph should be updated on new data.
 * @param update true to update on new data, false to disable update
 */
void
SkillGuiGraphDrawingArea::set_update_graph(bool update)
{
  if (update && ! update_graph_) {
    if ( graph_fsm_ != nonupd_graph_fsm_ ) {
      scale_override_ = false;
    }
    graph_     = nonupd_graph_;
    graph_fsm_ = nonupd_graph_fsm_;
    queue_draw();
  }
  update_graph_ = update;
}


void
SkillGuiGraphDrawingArea::save_dotfile(const char *filename)
{
  FILE *f = fopen(filename, "w");
  if (f) {
    if (fwrite(graph_.c_str(), graph_.length(), 1, f) != 1) {
      // bang, ignored
      printf("Failed to write dot file '%s'\n", filename);
    }
    fclose(f);
  }
}


/** Enable/disable recording.
 * @param recording true to enable recording, false otherwise
 * @return true if recording is enabled now, false if it is disabled.
 * Enabling the recording may fail for example if the user chose to abort
 * the directory creation process.
 */
bool
SkillGuiGraphDrawingArea::set_recording(bool recording)
{
  if (recording) {
    Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
    fcd_recording_->set_transient_for(*w);
    int result = fcd_recording_->run();
    if (result == Gtk::RESPONSE_OK) {
      record_directory_ = fcd_recording_->get_filename();
      recording_ = true;
    }
    fcd_recording_->hide();
  } else {
    recording_ = false;
  }
  return recording_;
}


/** save current graph. */
void
SkillGuiGraphDrawingArea::save()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  fcd_save_->set_transient_for(*w);

  int result = fcd_save_->run();
  if (result == Gtk::RESPONSE_OK) {

#if GTK_VERSION_GE(3,0)
    Glib::RefPtr<Gtk::FileFilter> f = fcd_save_->get_filter();
#else
    Gtk::FileFilter *f = fcd_save_->get_filter();
#endif
    std::string filename = fcd_save_->get_filename();
    if (filename != "") {
      if (f == filter_dot_) {
	save_dotfile(filename.c_str());
      } else {
	Cairo::RefPtr<Cairo::Surface> surface;

	bool write_to_png = false;
	if (f == filter_pdf_) {
	  surface = Cairo::PdfSurface::create(filename, bbw_, bbh_);
	} else if (f == filter_svg_) {
	  surface = Cairo::SvgSurface::create(filename, bbw_, bbh_);
	} else if (f == filter_png_) {
	  surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32,
						(int)ceilf(bbw_),
						(int)ceilf(bbh_));
	  write_to_png = true;
	}

	if (surface) {
	  cairo_ = Cairo::Context::create(surface);
	  
	  bool old_scale_override = scale_override_;
	  double old_tx = translation_x_;
	  double old_ty = translation_y_;
	  double old_scale = scale_;
	  translation_x_ = pad_x_;
	  translation_y_ = bbh_ - pad_y_;
	  scale_ = 1.0;
	  scale_override_ = true;

	  Agraph_t *g = agmemread((char *)graph_.c_str());
	  if (g) {
	    gvLayout(gvc_, g, (char *)"dot");
	    gvRender(gvc_, g, (char *)"skillguicairo", NULL);
	    gvFreeLayout(gvc_, g);
	    agclose(g);
	  }

	  if (write_to_png) {
	    surface->write_to_png(filename);
	  }

	  cairo_.clear();

	  translation_x_ = old_tx;
	  translation_y_ = old_ty;
	  scale_ = old_scale;
	  scale_override_ = old_scale_override;
	}
      }

    } else {
      Gtk::MessageDialog md(*w, "Invalid filename",
			    /* markup */ false, Gtk::MESSAGE_ERROR,
			    Gtk::BUTTONS_OK, /* modal */ true);
      md.set_title("Invalid File Name");
      md.run();
    }
  }

  fcd_save_->hide();
}


/** Open a dot graph and display it. */
void
SkillGuiGraphDrawingArea::open()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  fcd_open_->set_transient_for(*w);

  int result = fcd_open_->run();
  if (result == Gtk::RESPONSE_OK) {
    update_graph_ = false;
    graph_ = "";
    char *basec = strdup(fcd_open_->get_filename().c_str());
    char *basen = basename(basec);
    graph_fsm_ = basen;
    free(basec);

    FILE *f = fopen(fcd_open_->get_filename().c_str(), "r");
    while (! feof(f)) {
      char tmp[4096];
      size_t s;
      if ((s = fread(tmp, 1, 4096, f)) > 0) {
	graph_.append(tmp, s);
      }
    }
    fclose(f);
    signal_update_disabled_.emit();
    queue_draw();
  }

  fcd_open_->hide();
}


#if GTK_VERSION_GE(3,0)
/** Draw event handler.
 * @param cr cairo context
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_draw(const Cairo::RefPtr<Cairo::Context> &cr)
#else
/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
SkillGuiGraphDrawingArea::on_expose_event(GdkEventExpose* event)
#endif
{
  // This is where we draw on the window
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window) {
    //Gtk::Allocation allocation = get_allocation();
    //const int width = allocation.get_width();
    //const int height = allocation.get_height();
    
    // coordinates for the center of the window
    //int xc, yc;
    //xc = width / 2;
    //yc = height / 2;
#if GTK_VERSION_LT(3,0)
    cairo_ = window->create_cairo_context();
#else
    cairo_ = cr;
#endif
    cairo_->set_source_rgb(1, 1, 1);
    cairo_->paint();

    Agraph_t *g = agmemread((char *)graph_.c_str());
    if (g) {
      gvLayout(gvc_, g, (char *)"dot");
      gvRender(gvc_, g, (char *)"skillguicairo", NULL);
      gvFreeLayout(gvc_, g);
      agclose(g);
    }

    cairo_.clear();
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
  last_mouse_x_ = event->x;
  last_mouse_y_ = event->y;
  return true;
}


/** Mouse motion notify event handler.
 * @param event event data
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_motion_notify_event(GdkEventMotion *event)
{
  scale_override_ = true;
  translation_x_ -= last_mouse_x_ - event->x;
  translation_y_ -= last_mouse_y_ - event->y;
  last_mouse_x_ = event->x;
  last_mouse_y_ = event->y;
  queue_draw();
  return true;
}

