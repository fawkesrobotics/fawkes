
/***************************************************************************
 *  gvplugin_skillgui_cairo.cpp - Graphviz plugin for Skill GUI using cairo
 *
 *  Created: Fri Dec 19 12:01:39 2008
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

#include "gvplugin_skillgui_cairo.h"

#include <utils/math/angle.h>
#include <utils/time/tracker.h>

#include <gvplugin_device.h>
#include <gvplugin_render.h>

#include <algorithm>
#include <cstdio>

#define NOEXPORT __attribute__ ((visibility("hidden")))

NOEXPORT SkillGuiCairoRenderInstructor *__sgcri = NULL;

#if CAIROMM_MAJOR_VERSION > 1 || (CAIROMM_MAJOR_VERSION == 1 && CAIROMM_MINO_VERSION > 8)
NOEXPORT std::vector<double> __skillgui_cairo_render_dashed;
NOEXPORT std::vector<double> __skillgui_cairo_render_dotted;
#else
NOEXPORT std::valarray<double> __skillgui_cairo_render_dashed(1);
NOEXPORT std::valarray<double> __skillgui_cairo_render_dotted(2);
#endif

#ifdef USE_GVPLUGIN_TIMETRACKER
NOEXPORT fawkes::TimeTracker __tt;
NOEXPORT unsigned int __ttc_page = __tt.add_class("Page");
NOEXPORT unsigned int __ttc_beginpage = __tt.add_class("Begin Page");
NOEXPORT unsigned int __ttc_ellipse = __tt.add_class("Ellipse");
NOEXPORT unsigned int __ttc_bezier = __tt.add_class("Bezier");
NOEXPORT unsigned int __ttc_polygon = __tt.add_class("Polygon");
NOEXPORT unsigned int __ttc_polyline = __tt.add_class("Polyline");
NOEXPORT unsigned int __ttc_text = __tt.add_class("Text");
NOEXPORT unsigned int __ttc_text_1 = __tt.add_class("Text 1");
NOEXPORT unsigned int __ttc_text_2 = __tt.add_class("Text 2");
NOEXPORT unsigned int __ttc_text_3 = __tt.add_class("Text 3");
NOEXPORT unsigned int __ttc_text_4 = __tt.add_class("Text 4");
NOEXPORT unsigned int __ttc_text_5 = __tt.add_class("Text 5");
NOEXPORT unsigned int __tt_count = 0;
NOEXPORT unsigned int __num_ellipse = 0;
NOEXPORT unsigned int __num_bezier = 0;
NOEXPORT unsigned int __num_polygon = 0;
NOEXPORT unsigned int __num_polyline = 0;
NOEXPORT unsigned int __num_text = 0;
#endif


/** @class SkillGuiCairoRenderInstructor
 * Graphviz Cairo render plugin instructor.
 * @author Tim Niemueller
 *
 * @fn Cairo::RefPtr<Cairo::Context> SkillGuiCairoRenderInstructor::get_cairo()
 * Get Cairo context.
 * @return cairo context to use for drawing
 *
 * @fn bool   SkillGuiCairoRenderInstructor::scale_override()
 * Check if scale override is enabled.
 * @return true if the instructor determines the scaling, false to have the
 * plugin do this.
 *
 * @fn void   SkillGuiCairoRenderInstructor::get_dimensions(double &width, double &height)
 * Get available space dimensions.
 * @param width upon return contains the available width
 * @param height upon return contains the available height
 *
 * @fn double SkillGuiCairoRenderInstructor::get_scale()
 * Get scale factor.
 * If scale_override() returns true, shall return the requested scale value.
 * @return scale factor
 *
 * @fn void   SkillGuiCairoRenderInstructor::set_scale(double scale)
 * Set scale.
 * Set the scale value that the plugin determined.
 * @param scale scale determined by plugin
 *
 * @fn void   SkillGuiCairoRenderInstructor::get_translation(double &tx, double &ty)
 * Get translation values.
 * If scale_override() returns true, shall return the requested translation values.
 * @param tx upon return contains translation in x
 * @param ty upon return contains translation in y
 *
 * @fn void   SkillGuiCairoRenderInstructor::set_translation(double tx, double ty)
 * Set translation.
 * Set the translation values the plugin determined.
 * @param tx translation in x
 * @param ty translation in y
 *
 * @fn void   SkillGuiCairoRenderInstructor::set_bb(double bbw, double bbh)
 * Set the bounding box.
 * Set by the plugin before calling any other function.
 * @param bbw bounding box width
 * @param bbh bounding box height
 *
 * @fn void   SkillGuiCairoRenderInstructor::set_pad(double pad_x, double pad_y)
 * Set padding.
 * Set by the plugin immediately after set_bb() is called.
 * @param pad_x padding in x
 * @param pad_y padding in y
 *
 * @fn void   SkillGuiCairoRenderInstructor::get_pad(double &pad_x, double &pad_y)
 * Get padding.
 * If scale_override() returns true, shall return the requested padding values.
 * @param pad_x upon return contains padding in x
 * @param pad_y upon return contains padding in y
 */

static void
skillgui_cairo_device_init(GVJ_t *firstjob)
{
}

static void
skillgui_cairo_device_finalize(GVJ_t *firstjob)
{
  firstjob->context = (void *)__sgcri;
  firstjob->external_context = TRUE;

  // Render!
  (firstjob->callbacks->refresh)(firstjob);
}

static inline void
skillgui_cairo_set_color(Cairo::RefPtr<Cairo::Context> cairo, gvcolor_t * color)
{
  cairo->set_source_rgba(color->u.RGBA[0], color->u.RGBA[1],
			 color->u.RGBA[2], color->u.RGBA[3]);
}

static inline void
skillgui_cairo_set_penstyle(Cairo::RefPtr<Cairo::Context> cairo, GVJ_t *job)
{
  obj_state_t *obj = job->obj;

  if (obj->pen == PEN_DASHED) {
    cairo->set_dash(__skillgui_cairo_render_dashed, 0.0);
  } else if (obj->pen == PEN_DOTTED) {
    cairo->set_dash(__skillgui_cairo_render_dotted, 0.0);
  } else {
#if CAIROMM_MAJOR_VERSION > 1 || (CAIROMM_MAJOR_VERSION == 1 && CAIROMM_MINO_VERSION > 8)
    std::vector<double> empty;
#else
    std::valarray<double> empty;
#endif
    cairo->set_dash(empty, 0.0);
  }
  cairo->set_line_width(obj->penwidth);
}


static void
skillgui_cairo_render_begin_page(GVJ_t *job)
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_page);
  __tt.ping_start(__ttc_beginpage);
#endif
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;

  float bbwidth  = job->bb.UR.x - job->bb.LL.x;
  float bbheight = job->bb.UR.y - job->bb.LL.y;

  cri->set_bb(bbwidth, bbheight);
  cri->set_pad(job->pad.x, job->pad.y);
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();

  double pad_x, pad_y;
  cri->get_pad(pad_x, pad_y);

  // For internal calculations we need to care about the padding
  //bbwidth  += 2 * pad_x;
  //bbheight += 2 * pad_y;

  double avwidth, avheight;
  cri->get_dimensions(avwidth, avheight);
  float translate_x = 0;
  float translate_y = 0;

  if ( cri->scale_override() ) {
    float zoom = cri->get_scale();
    float zwidth  = bbwidth * zoom;
    float zheight = bbheight * zoom;
    translate_x += (avwidth  - zwidth ) / 2.;
    translate_y += (avheight - zheight) / 2.;

    double translate_x, translate_y;
    cri->get_translation(translate_x, translate_y);

    cairo->translate(translate_x, translate_y);
    cairo->scale(zoom, zoom);

  } else {
    float zoom_w = avwidth  / bbwidth;
    float zoom_h = avheight / bbheight;
    float zoom   = std::min(zoom_w, zoom_h);

    if (bbwidth > avwidth || bbheight > avheight) {
      float zwidth  = bbwidth * zoom;
      float zheight = bbheight * zoom;
      translate_x += (avwidth  - zwidth ) / 2.;
      translate_y += (avheight - zheight) / 2. + zheight;
    } else {
      zoom = 1.0;
      translate_x += (avwidth  - bbwidth)  / 2.;
      translate_y += (avheight - bbheight) / 2. + bbheight;
    }

    cri->set_scale(zoom);
    cri->set_translation(translate_x, translate_y);

    cairo->translate(translate_x + pad_x * zoom, translate_y - pad_y * zoom);
    cairo->scale(zoom, zoom);
  }



#ifdef USE_GVPLUGIN_TIMETRACKER
  __num_ellipse = 0;
  __num_bezier = 0;
  __num_polygon = 0;
  __num_polyline = 0;
  __num_text = 0;

  __tt.ping_end(__ttc_beginpage);
#endif
}

static void
skillgui_cairo_render_end_page(GVJ_t * job)
{
  //SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  //cri->queue_draw();  
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_page);
  if ( ++__tt_count >= 10 ) {
    __tt_count = 0;
    __tt.print_to_stdout();

    printf("Num Ellipse:   %u\n"
	   "Num Bezier:    %u\n"
	   "Num Polygon:   %u\n"
	   "Num Polyline:  %u\n"
	   "Num Text:      %u\n", __num_ellipse, __num_bezier,
	   __num_polygon, __num_polyline, __num_text);
  }
#endif
}

static void
#if GRAPHVIZ_VERSION >= 23600
skillgui_cairo_render_textpara(GVJ_t *job, pointf p, textspan_t *para)
#else
skillgui_cairo_render_textpara(GVJ_t *job, pointf p, textpara_t *para)
#endif
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_text);
  ++__num_text;
#endif
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();
  obj_state_t *obj = job->obj;

  Cairo::FontWeight weight = Cairo::FONT_WEIGHT_NORMAL;
  Cairo::FontSlant slant   = Cairo::FONT_SLANT_NORMAL;
  char *fontweight = NULL;
  if (obj->type == CLUSTER_OBJTYPE) {
    fontweight = agget(obj->u.sg, (char *)"fontweight");
  } else if (obj->type == ROOTGRAPH_OBJTYPE) {
    fontweight = agget(obj->u.g, (char *)"fontweight");
  } else if (obj->type == NODE_OBJTYPE) {
    fontweight = agget(obj->u.n, (char *)"fontweight");
  } else if (obj->type == EDGE_OBJTYPE) {
    fontweight = agget(obj->u.e, (char *)"fontweight");
  }
  if (fontweight && (strcmp(fontweight, "bold") == 0)) {
    weight = Cairo::FONT_WEIGHT_BOLD;
    p.x -= 8;
  }
  char *fontslant = NULL;
  if (obj->type == CLUSTER_OBJTYPE) {
    fontslant = agget(obj->u.sg, (char *)"fontslant");
  } else if (obj->type == ROOTGRAPH_OBJTYPE) {
    fontslant = agget(obj->u.g, (char *)"fontslant");
  } else if (obj->type == NODE_OBJTYPE) {
    fontslant = agget(obj->u.n, (char *)"fontslant");
  } else if (obj->type == EDGE_OBJTYPE) {
    fontslant = agget(obj->u.e, (char *)"fontslant");
  }
  if (fontslant && (strcmp(fontslant, "italic") == 0)) {
    slant = Cairo::FONT_SLANT_ITALIC;
  }

  double offsetx = 0.0;
  double offsety = 0.0;
  double rotate  = 0.0;

  if ((obj->type == EDGE_OBJTYPE) && obj->headlabel && (strcmp(para->str, obj->headlabel) == 0) ) {
    char *labelrotate = agget(obj->u.e, (char *)"labelrotate");
    if (labelrotate && (strlen(labelrotate) > 0)) {
      rotate = fawkes::deg2rad(atof(labelrotate));
    }
    char *labeloffsetx = agget(obj->u.e, (char *)"labeloffsetx");
    if (labeloffsetx && (strlen(labeloffsetx) > 0)) {
      offsetx = atof(labeloffsetx);
    }
    char *labeloffsety = agget(obj->u.e, (char *)"labeloffsety");
    if (labeloffsety && (strlen(labeloffsety) > 0)) {
      offsety = atof(labeloffsety);
    }
  }
  //__tt.ping_start(__ttc_text_1);

  Cairo::Matrix old_matrix;
  cairo->get_matrix(old_matrix);

#if GRAPHVIZ_VERSION >= 23600
  cairo->select_font_face(para->font->name, slant, weight);
  cairo->set_font_size(para->font->size);
#else
  cairo->select_font_face(para->fontname, slant, weight);
  cairo->set_font_size(para->fontsize);
#endif

  //cairo->set_font_options ( Cairo::FontOptions() );
  //cairo->set_line_width(1.0);

  Cairo::TextExtents extents;
  cairo->get_text_extents(para->str, extents);

  if (para->just == 'r') {
    p.x -= extents.width;
  } else if (para->just != 'l') {
    p.x -= extents.width / 2.0;
  }

  cairo->move_to(p.x + offsetx, -p.y + offsety);
  cairo->rotate(rotate);
  skillgui_cairo_set_color(cairo, &(obj->pencolor));
  cairo->text_path( para->str );
  cairo->fill();

  cairo->set_matrix(old_matrix);

  //__tt.ping_end(__ttc_text_5);
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_text);
#endif
}

static void
skillgui_cairo_render_ellipse(GVJ_t *job, pointf *A, int filled)
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_ellipse);
  ++__num_ellipse;
#endif
  //printf("Render ellipse\n");
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();
  obj_state_t *obj = job->obj;

  Cairo::Matrix old_matrix;
  cairo->get_matrix(old_matrix);

  skillgui_cairo_set_penstyle(cairo, job);

  cairo->translate(A[0].x, -A[0].y);

  double rx = A[1].x - A[0].x;
  double ry = A[1].y - A[0].y;
  cairo->scale(1, ry / rx);
  cairo->move_to(rx, 0);
  cairo->arc(0, 0, rx, 0, 2 * M_PI);
  cairo->close_path();

  cairo->set_matrix(old_matrix);

  if (filled) {
    skillgui_cairo_set_color(cairo, &(obj->fillcolor));
    cairo->fill_preserve();
  }
  skillgui_cairo_set_color(cairo, &(obj->pencolor));
  cairo->stroke();

#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_ellipse);
#endif
}

static void
skillgui_cairo_render_polygon(GVJ_t *job, pointf *A, int n, int filled)
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_polygon);
  ++__num_polygon;
#endif
  //printf("Polygon\n");
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();
  obj_state_t *obj = job->obj;

  skillgui_cairo_set_penstyle(cairo, job);

  cairo->move_to(A[0].x, -A[0].y);
  for (int i = 1; i < n; ++i) {
    cairo->line_to(A[i].x, -A[i].y);
  }
  cairo->close_path();

  if (filled) {
    skillgui_cairo_set_color(cairo, &(obj->fillcolor));
    cairo->fill_preserve();
  }

  // HACK to workaround graphviz bug any get the Tim style...
  if ( obj->type == CLUSTER_OBJTYPE ) {
    obj->pencolor.u.RGBA[0] = 0.666;
    obj->pencolor.u.RGBA[1] = 0.666;
    obj->pencolor.u.RGBA[2] = 1.0;
    obj->pencolor.u.RGBA[3] = 1.0;
  }
  skillgui_cairo_set_color(cairo, &(obj->pencolor));
  cairo->stroke();

#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_polygon);
#endif
}

static void
skillgui_cairo_render_bezier(GVJ_t * job, pointf * A, int n, int arrow_at_start,
		int arrow_at_end, int filled)
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_bezier);
  ++__num_bezier;
#endif
  //printf("Bezier\n");
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();
  obj_state_t *obj = job->obj;

  skillgui_cairo_set_penstyle(cairo, job);

  cairo->move_to(A[0].x, -A[0].y);
  for (int i = 1; i < n; i += 3)
    cairo->curve_to(A[i].x, -A[i].y, A[i + 1].x, -A[i + 1].y,
		    A[i + 2].x, -A[i + 2].y);
  if (filled) {
    skillgui_cairo_set_color(cairo, &(obj->fillcolor));
    cairo->fill_preserve();
  }
  skillgui_cairo_set_color(cairo, &(obj->pencolor));
  cairo->stroke();

#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_bezier);
#endif
}

static void
skillgui_cairo_render_polyline(GVJ_t * job, pointf * A, int n)
{
#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_start(__ttc_polyline);
  ++__num_polyline;
#endif
  //printf("Polyline\n");
  SkillGuiCairoRenderInstructor *cri = (SkillGuiCairoRenderInstructor *)job->context;
  Cairo::RefPtr<Cairo::Context> cairo = cri->get_cairo();
  obj_state_t *obj = job->obj;

  skillgui_cairo_set_penstyle(cairo, job);

  //cairo->set_line_width(obj->penwidth * job->scale.x);
  cairo->move_to(A[0].x, -A[0].y);
  for (int i = 1; i < n; i++) {
    cairo->line_to(A[i].x, -A[i].y);
  }
  skillgui_cairo_set_color(cairo, &(obj->pencolor));
  cairo->stroke();

#ifdef USE_GVPLUGIN_TIMETRACKER
  __tt.ping_end(__ttc_polyline);
#endif
}


static gvrender_engine_t skillgui_cairo_render_engine = {
    0,				/* skillgui_cairo_render_begin_job */
    0,				/* skillgui_cairo_render_end_job */
    0,				/* skillgui_cairo_render_begin_graph */
    0,				/* skillgui_cairo_render_end_graph */
    0,				/* skillgui_cairo_render_begin_layer */
    0,				/* skillgui_cairo_render_end_layer */
    skillgui_cairo_render_begin_page,
    skillgui_cairo_render_end_page,
    0,				/* skillgui_cairo_render_begin_cluster */
    0,				/* skillgui_cairo_render_end_cluster */
    0,				/* skillgui_cairo_render_begin_nodes */
    0,				/* skillgui_cairo_render_end_nodes */
    0,				/* skillgui_cairo_render_begin_edges */
    0,				/* skillgui_cairo_render_end_edges */
    0,				/* skillgui_cairo_render_begin_node */
    0,				/* skillgui_cairo_render_end_node */
    0,				/* skillgui_cairo_render_begin_edge */
    0,				/* skillgui_cairo_render_end_edge */
    0,				/* skillgui_cairo_render_begin_anchor */
    0,				/* skillgui_cairo_render_end_anchor */
    0,				/* skillgui_cairo_begin_label */
    0,				/* skillgui_cairo_end_label */
    skillgui_cairo_render_textpara,
    0,				/* skillgui_cairo_render_resolve_color */
    skillgui_cairo_render_ellipse,
    skillgui_cairo_render_polygon,
    skillgui_cairo_render_bezier,
    skillgui_cairo_render_polyline,
    0,				/* skillgui_cairo_render_comment */
    0,				/* skillgui_cairo_render_library_shape */
};

static gvdevice_engine_t skillgui_cairo_device_engine = {
    skillgui_cairo_device_init,
    NULL,			/* skillgui_cairo_device_format */
    skillgui_cairo_device_finalize,
};


#ifdef __cplusplus
extern "C" {
#endif


static gvrender_features_t skillgui_cairo_render_features = {
  GVRENDER_Y_GOES_DOWN |
  GVRENDER_DOES_LABELS |
  GVRENDER_DOES_TRANSFORM |
  GVRENDER_NO_WHITE_BG, 			/* flags */
  8,                         			/* default pad - graph units */
  0,						/* knowncolors */
  0,						/* sizeof knowncolors */
  RGBA_DOUBLE,					/* color_type */
};


static gvdevice_features_t skillgui_cairo_device_features = {
  GVDEVICE_DOES_TRUECOLOR | GVDEVICE_EVENTS,	/* flags */
  {0.,0.},                    			/* default margin - points */
  {0.,0.},					/* default page width, height - points */
  {96.,96.},					/* dpi */
};

gvplugin_installed_t gvdevice_types_skillgui_cairo[] = {
  {0, ( char *)"skillguicairo:skillguicairo", 0, &skillgui_cairo_device_engine, &skillgui_cairo_device_features},
  {0, NULL, 0, NULL, NULL}
};

gvplugin_installed_t gvrender_types_skillgui_cairo[] = {
  {0, (char *)"skillguicairo", 10, &skillgui_cairo_render_engine, &skillgui_cairo_render_features},
  {0, NULL, 0, NULL, NULL}
};

static gvplugin_api_t apis[] = {
  {API_device, gvdevice_types_skillgui_cairo},
  {API_render, gvrender_types_skillgui_cairo},
  {(api_t)0, 0},
};

gvplugin_library_t gvplugin_skillgui_cairo_LTX_library = { (char *)"skillguicairo", apis };

#ifdef __cplusplus
}
#endif


void
gvplugin_skillgui_cairo_setup(GVC_t *gvc, SkillGuiCairoRenderInstructor *sgcri)
{
  __sgcri = sgcri;
  gvAddLibrary(gvc, &gvplugin_skillgui_cairo_LTX_library);

#if CAIROMM_MAJOR_VERSION > 1 || (CAIROMM_MAJOR_VERSION == 1 && CAIROMM_MINO_VERSION > 8)
  __skillgui_cairo_render_dashed.clear();
  __skillgui_cairo_render_dashed.push_back(6.0);
  __skillgui_cairo_render_dotted.clear();
  __skillgui_cairo_render_dotted.push_back(2.0);
  __skillgui_cairo_render_dotted.push_back(6.0);
#else
  __skillgui_cairo_render_dashed[0] = 6.0;
  __skillgui_cairo_render_dotted[0] = 2.0;
  __skillgui_cairo_render_dotted[1] = 6.0;
#endif
}
