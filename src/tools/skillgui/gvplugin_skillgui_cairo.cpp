
/***************************************************************************
 *  gvplugin_skillgui_cairo.cpp - Graphviz plugin for Skill GUI using Cairo
 *
 *  Created: Fri Dec 19 12:01:39 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "gvplugin_skillgui_cairo.h"

#include <gvplugin_device.h>
#include <gvplugin_render.h>

#include <cstdio>

static Gtk::DrawingArea *__drwarea;


static void
skillgui_cairo_device_init(GVJ_t *firstjob)
{
  Glib::RefPtr<const Gdk::Screen> s = __drwarea->get_screen();
  firstjob->device_dpi.x = s->get_resolution();
  firstjob->device_dpi.y = s->get_resolution();
  firstjob->device_sets_dpi = TRUE;
}

static void
skillgui_cairo_device_finalize(GVJ_t *firstjob)
{
  Glib::RefPtr<Gdk::Window> window = __drwarea->get_window();
  if ( window ) {
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
    Gtk::Allocation alloc = __drwarea->get_allocation();

    firstjob->context = (void *)cr->cobj();
    firstjob->external_context = TRUE;

    printf("Alloc: %i x %i\n", alloc.get_width(), alloc.get_height());
    firstjob->width  = alloc.get_width();
    firstjob->height = alloc.get_height();

    // Render!
    (firstjob->callbacks->refresh)(firstjob);
  }
}


static gvdevice_engine_t skillgui_device_engine = {
    skillgui_cairo_device_init,
    NULL,			/* skillgui_device_format */
    skillgui_cairo_device_finalize,
};


#ifdef __cplusplus
extern "C" {
#endif


static gvdevice_features_t skillgui_device_features = {
  GVDEVICE_DOES_TRUECOLOR | GVDEVICE_EVENTS,	/* flags */
  {0.,0.},                    			/* default margin - points */
  {0.,0.},					/* default page width, height - points */
  {96.,96.},					/* dpi */
};

gvplugin_installed_t gvdevice_types_skillgui[] = {
  {0, ( char *)"skillguicairo:cairo", 0, &skillgui_device_engine, &skillgui_device_features},
  {0, NULL, 0, NULL, NULL}
};

static gvplugin_api_t apis[] = {
  {API_device, gvdevice_types_skillgui},
  {(api_t)0, 0},
};

gvplugin_library_t gvplugin_skillguicairo_LTX_library = { (char *)"skillguicairo", apis };

#ifdef __cplusplus
}
#endif


void
gvplugin_skillgui_cairo_setup(GVC_t *gvc, Gtk::DrawingArea *drwarea)
{
  __drwarea = drwarea;
  gvAddLibrary(gvc, &gvplugin_skillguicairo_LTX_library);
}
