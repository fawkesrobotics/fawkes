
/***************************************************************************
 *  graph_viewport.h - FSM Graph Viewport for Skill GUI
 *
 *  Created: Mon Dec 15 15:38:02 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __TOOLS_SKILLGUI_GRAPH_VIEWPORT_H_
#define __TOOLS_SKILLGUI_GRAPH_VIEWPORT_H_

#include <papyrus.h>
#include <papyrus-gtkmm/viewport.h>

#include <gvc.h>
#include <gvcjob.h>

#include <string>

class SkillGuiGraphViewport : public Papyrus::Gtk::Viewport
{
 public:
  SkillGuiGraphViewport();
  ~SkillGuiGraphViewport();

  void set_gvjob(GVJ_t *job);
  void set_graph_fsm(std::string fsm_name);
  void set_graph(std::string graph);

  bool get_update_graph();
  void set_update_graph(bool update);

  void save();
  void render();

  void zoom_in();
  void zoom_out();
  void zoom_fit();
  void zoom_reset();

  Papyrus::AffineController::pointer get_affine();

  void add_drawable(Papyrus::Drawable::pointer d);
  virtual void clear();
  void set_bb(double bbw, double bbh);
  void set_pad(double pad_x, double pad_y);
  void set_translation(double tx, double ty);
  void set_scale(double scale);
  bool scale_override();

 protected:
  void on_expose(GdkEventExpose *event);

 private:
  GVC_t *__gvc;
  GVJ_t *__gvjob;

  std::string __graph_fsm;
  std::string __graph;

  double __bbw;
  double __bbh;
  double __pad_x;
  double __pad_y;
  double __translation_x;
  double __translation_y;
  double __scale;
  bool   __update_graph;

  bool __scale_override;

  Gtk::FileChooserDialog *__fcd;
  Papyrus::AffineController::pointer __affine;
  Papyrus::Translator::pointer __translator;
};


#endif
