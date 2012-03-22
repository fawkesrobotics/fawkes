
/***************************************************************************
 *  gvplugin_skillgui_cairo.h - Graphviz plugin for Skill GUI
 *
 *  Created: Fri Dec 19 12:01:03 2008
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

#ifndef __TOOLS_SKILLGUI_GVPLUGIN_SKILLGUI_CAIRO_H_
#define __TOOLS_SKILLGUI_GVPLUGIN_SKILLGUI_CAIRO_H_

#include <cairomm/cairomm.h>
#include <gvc.h>

class SkillGuiCairoRenderInstructor
{
 public:
  /** Empty virtual destructor. */
  virtual ~SkillGuiCairoRenderInstructor() {}

  virtual Cairo::RefPtr<Cairo::Context> get_cairo() = 0;

  virtual bool   scale_override() = 0;
  virtual void   get_dimensions(double &width, double &height) = 0;
  virtual double get_scale() = 0;
  virtual void   set_scale(double scale) = 0;
  virtual void   get_translation(double &tx, double &ty) = 0;
  virtual void   set_translation(double tx, double ty) = 0;
  virtual void   set_bb(double bbw, double bbh) = 0;
  virtual void   set_pad(double pad_x, double pad_y) = 0;
  virtual void   get_pad(double &pad_x, double &pad_y) = 0;

};

extern void gvplugin_skillgui_cairo_setup(GVC_t *gvc,
					  SkillGuiCairoRenderInstructor *sgcri);

#endif
