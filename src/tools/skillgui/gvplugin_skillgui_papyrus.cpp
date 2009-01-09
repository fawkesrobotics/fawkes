
/***************************************************************************
 *  fvplugin_skillgui.cpp - Graphviz plugin for Skill GUI using libpapyrus
 *
 *  Created: Tue Dec 16 14:36:51 2008
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

#include "gvplugin_skillgui_papyrus.h"

#include <utils/math/angle.h>

#include <gvplugin_device.h>
#include <gvplugin_render.h>

#include <cstdio>

#define NOEXPORT __attribute__ ((visibility("hidden")))

NOEXPORT SkillGuiGraphViewport *__sggvp = NULL;

NOEXPORT std::valarray<double> __skillgui_render_dashed(6., 1);
NOEXPORT std::valarray<double> __skillgui_render_dotted((double[]){2., 6.}, 2);


static void
skillgui_device_init(GVJ_t *firstjob)
{
  Glib::RefPtr<const Gdk::Screen> s = __sggvp->get_screen();
  firstjob->device_dpi.x = s->get_resolution();
  firstjob->device_dpi.y = s->get_resolution();
  firstjob->device_sets_dpi = true;

  Gtk::Allocation alloc      = __sggvp->get_allocation();
  firstjob->width            = alloc.get_width();
  firstjob->height           = alloc.get_height();

  firstjob->fit_mode = TRUE;
}

static void
skillgui_device_finalize(GVJ_t *firstjob)
{
  __sggvp->set_gvjob(firstjob);

  firstjob->context = (void *)__sggvp;
  firstjob->external_context = TRUE;

  double zoom_to_fit;

  Gtk::Allocation alloc      = __sggvp->get_allocation();

  if (! firstjob->has_been_rendered) {
    zoom_to_fit = MIN((double) alloc.get_width()  / (double) firstjob->width,
		      (double) alloc.get_height() / (double) firstjob->height);

    if (zoom_to_fit < 1.0) {
      firstjob->zoom *= zoom_to_fit;
    }
  } else if (firstjob->fit_mode) {
    zoom_to_fit = MIN((double) alloc.get_width()  / (double) firstjob->width,
		      (double) alloc.get_height() / (double) firstjob->height);
    firstjob->zoom *= zoom_to_fit;
  }

  if (((unsigned int)alloc.get_width() > firstjob->width) ||
      ((unsigned int)alloc.get_height() > firstjob->height) ) {
    firstjob->has_grown = TRUE;
  }
  firstjob->width = alloc.get_width();
  firstjob->height = alloc.get_height();
  firstjob->needs_refresh = TRUE;


  // Render!
  (firstjob->callbacks->refresh)(firstjob);
}


static inline float
offset_x(GVJ_t *job)
{
  return 0; //job->pad.x * job->scale.x;
}

static inline float
offset_y(GVJ_t *job)
{
  return 0; //job->pad.y * job->scale.y;
}


static inline Cairo::RefPtr< Cairo::SolidPattern >
skillgui_render_solidpattern(gvcolor_t *color)
{
  return Cairo::SolidPattern::create_rgba(color->u.RGBA[0],
					  color->u.RGBA[1],
					  color->u.RGBA[2],
					  color->u.RGBA[3]);
}


static inline Papyrus::LineStyle
skillgui_render_linestyle(obj_state_t *obj)
{
  Papyrus::LineStyle ls(skillgui_render_solidpattern(&(obj->pencolor)), obj->penwidth);

  if (obj->pen == PEN_DASHED) {
    ls.set_dash(__skillgui_render_dashed);
  } else if (obj->pen == PEN_DOTTED) {
    ls.set_dash(__skillgui_render_dotted);
  }

  return ls;
}

static void
skillgui_render_begin_page(GVJ_t *job)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  gvp->clear();
}

static void
skillgui_render_end_page(GVJ_t * job)
{
  //SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  //gvp->queue_draw();  
}

static void
skillgui_render_textpara(GVJ_t *job, pointf p, textpara_t *para)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  obj_state_t *obj = job->obj;

  switch (para->just) {
  case 'r':
    p.x -= para->width;
    break;
  case 'l':
    p.x -= 0.0;
    break;
  case 'n':
  default:
    p.x -= para->width / 2.0;
    break;
  }

  //p.y -= para->height / 2.0;// - para->yoffset_centerline;
  //p.y -= para->yoffset_centerline;
  p.y += para->yoffset_centerline; // + para->yoffset_layout;

  Glib::RefPtr<Pango::Layout> pl = Glib::wrap((PangoLayout *)para->layout,
					      /* copy */ true);
  Pango::FontDescription fd = pl->get_font_description();
  Cairo::FontSlant slant = Cairo::FONT_SLANT_NORMAL;
  if (fd.get_style() == Pango::STYLE_OBLIQUE ) {
    slant = Cairo::FONT_SLANT_OBLIQUE;
  } else if (fd.get_style() == Pango::STYLE_ITALIC ) {
    slant = Cairo::FONT_SLANT_ITALIC;
  }
  Cairo::FontWeight weight = Cairo::FONT_WEIGHT_NORMAL;
  if ( fd.get_weight() == Pango::WEIGHT_BOLD ) {
    weight = Cairo::FONT_WEIGHT_BOLD;
  }

  double offsetx = 0.0;
  double offsety = 0.0;
  double rotate  = 0.0;

  if ( (obj->type == EDGE_OBJTYPE) && (strcmp(para->str, obj->headlabel) == 0) ) {
    char *labelrotate = agget(obj->u.e, (char *)"labelrotate");
    if (labelrotate && (strlen(labelrotate) > 0)) {
      rotate = fawkes::deg2rad(atof(labelrotate));
    }
    char *labeloffsetx = agget(obj->u.e, (char *)"labeloffsetx");
    if (labeloffsetx && (strlen(labeloffsetx) > 0)) {
      offsetx = atof(labeloffsetx) * job->scale.x;
    }
    char *labeloffsety = agget(obj->u.e, (char *)"labeloffsety");
    if (labeloffsety && (strlen(labeloffsety) > 0)) {
      offsety = atof(labeloffsety) * job->scale.y;
    }
  }


  Papyrus::Text::pointer t = Papyrus::Text::create(para->str, para->fontsize,
						   fd.get_family(), slant, weight);
  //t->set_outline(skillgui_render_solidpattern(&(obj->pencolor)));
  t->set_fill(skillgui_render_solidpattern(&(obj->pencolor)));
  t->set_translate(p.x + offset_x(job) + offsetx, p.y + offset_y(job) + offsety);
  t->set_rotate(rotate);
  gvp->canvas()->add(t);  

}

static void
skillgui_render_ellipse(GVJ_t *job, pointf * A, int filled)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  obj_state_t *obj = job->obj;

  double rx = fabs(A[1].x - A[0].x);
  double ry = fabs(A[1].y - A[0].y);

  Papyrus::Circle::pointer e = Papyrus::Circle::create(rx);
  Papyrus::LineStyle ls = skillgui_render_linestyle(obj);
  e->set_outline(ls);
  if ( filled )  e->set_fill(skillgui_render_solidpattern(&(obj->fillcolor)));
  e->set_translate(A[0].x + offset_x(job), A[0].y + offset_y(job));
  e->set_scale_y(ry / rx);

  gvp->canvas()->add(e);
}

static void
skillgui_render_polygon(GVJ_t *job, pointf *A, int n, int filled)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  obj_state_t *obj = job->obj;

  Papyrus::Vertices v;
  for (int i = 0; i < n; ++i) {
    v.push_back(Papyrus::Vertex(A[i].x + offset_x(job), A[i].y + offset_y(job)));
  }

  Papyrus::Polygon::pointer p = Papyrus::Polygon::create(v);
  p->set_outline(skillgui_render_linestyle(obj));
  if ( filled )  p->set_fill(skillgui_render_solidpattern(&(obj->fillcolor)));
  gvp->canvas()->add(p);
}

static void
skillgui_render_bezier(GVJ_t * job, pointf * A, int n, int arrow_at_start,
		int arrow_at_end, int filled)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  obj_state_t *obj = job->obj;

  Papyrus::BezierVertices v;
  v.push_back(Papyrus::BezierVertex(A[0].x + offset_x(job), A[0].y + offset_y(job),
				    A[0].x + offset_x(job), A[0].y + offset_y(job),
				    A[1].x + offset_x(job), A[1].y + offset_y(job)));
  for (int i = 1; i < n; i += 3) {
    if ( i < (n - 4) ) {
      v.push_back(Papyrus::BezierVertex(A[i+2].x + offset_x(job), A[i+2].y + offset_y(job),
					A[i+1].x + offset_x(job), A[i+1].y + offset_y(job),
					A[i+3].x + offset_x(job), A[i+3].y + offset_y(job)));
    } else {
      v.push_back(Papyrus::BezierVertex(A[i+2].x + offset_x(job), A[i+2].y + offset_y(job),
					A[i+1].x + offset_x(job), A[i+1].y + offset_y(job),
					A[i+2].x + offset_x(job), A[i+2].y + offset_y(job)));
    }
  }

  Papyrus::Bezierline::pointer p = Papyrus::Bezierline::create(v);
  p->set_outline(skillgui_render_linestyle(obj));
  if ( filled )  p->set_fill(skillgui_render_solidpattern(&(obj->fillcolor)));
  gvp->canvas()->add(p);
}

static void
skillgui_render_polyline(GVJ_t * job, pointf * A, int n)
{
  SkillGuiGraphViewport *gvp = (SkillGuiGraphViewport *)job->context;
  obj_state_t *obj = job->obj;

  Papyrus::Vertices v;
  for (int i = 0; i < n; ++i) {
    v.push_back(Papyrus::Vertex(A[i].x + offset_x(job), A[i].y + offset_y(job)));
  }

  Papyrus::Polyline::pointer p = Papyrus::Polyline::create(v);
  p->set_outline(skillgui_render_linestyle(obj));
  gvp->canvas()->add(p);
}


static gvrender_engine_t skillgui_render_engine = {
    0,				/* skillgui_render_begin_job */
    0,				/* skillgui_render_end_job */
    0,				/* skillgui_render_begin_graph */
    0,				/* skillgui_render_end_graph */
    0,				/* skillgui_render_begin_layer */
    0,				/* skillgui_render_end_layer */
    skillgui_render_begin_page,
    skillgui_render_end_page,
    0,				/* skillgui_render_begin_cluster */
    0,				/* skillgui_render_end_cluster */
    0,				/* skillgui_render_begin_nodes */
    0,				/* skillgui_render_end_nodes */
    0,				/* skillgui_render_begin_edges */
    0,				/* skillgui_render_end_edges */
    0,				/* skillgui_render_begin_node */
    0,				/* skillgui_render_end_node */
    0,				/* skillgui_render_begin_edge */
    0,				/* skillgui_render_end_edge */
    0,				/* skillgui_render_begin_anchor */
    0,				/* skillgui_render_end_anchor */
    skillgui_render_textpara,
    0,				/* skillgui_render_resolve_color */
    skillgui_render_ellipse,
    skillgui_render_polygon,
    skillgui_render_bezier,
    skillgui_render_polyline,
    0,				/* skillgui_render_comment */
    0,				/* skillgui_render_library_shape */
};

static gvdevice_engine_t skillgui_device_engine = {
    skillgui_device_init,
    NULL,			/* skillgui_device_format */
    skillgui_device_finalize,
};


#ifdef __cplusplus
extern "C" {
#endif


static gvrender_features_t skillgui_render_features = {
  GVRENDER_Y_GOES_DOWN | GVRENDER_DOES_LABELS, 	/* flags, for Cairo: GVRENDER_DOES_TRANSFORM */
  8,                         			/* default pad - graph units */
  0,						/* knowncolors */
  0,						/* sizeof knowncolors */
  RGBA_DOUBLE,					/* color_type */
};


static gvdevice_features_t skillgui_device_features = {
  GVDEVICE_DOES_TRUECOLOR | GVDEVICE_EVENTS,	/* flags */
  {0.,0.},                    			/* default margin - points */
  {0.,0.},					/* default page width, height - points */
  {96.,96.},					/* dpi */
};

gvplugin_installed_t gvdevice_types_skillgui[] = {
  {0, ( char *)"skillgui:skillgui", 0, &skillgui_device_engine, &skillgui_device_features},
  {0, NULL, 0, NULL, NULL}
};

gvplugin_installed_t gvrender_types_skillgui[] = {
  {0, (char *)"skillgui", 10, &skillgui_render_engine, &skillgui_render_features},
  {0, NULL, 0, NULL, NULL}
};

static gvplugin_api_t apis[] = {
  {API_device, gvdevice_types_skillgui},
  {API_render, gvrender_types_skillgui},
  {(api_t)0, 0},
};

gvplugin_library_t gvplugin_skillgui_LTX_library = { (char *)"skillgui", apis };

#ifdef __cplusplus
}
#endif


void
gvplugin_skillgui_setup(GVC_t *gvc, SkillGuiGraphViewport *sggvp)
{
  __sggvp = sggvp;
  gvAddLibrary(gvc, &gvplugin_skillgui_LTX_library);
}
