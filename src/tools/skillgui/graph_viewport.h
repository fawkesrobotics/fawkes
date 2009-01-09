
/***************************************************************************
 *  graph_viewport.h - FSM Graph Viewport for Skill GUI
 *
 *  Created: Mon Dec 15 15:38:02 2008
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

#ifndef __TOOLS_SKILLGUI_GRAPH_VIEWPORT_H_
#define __TOOLS_SKILLGUI_GRAPH_VIEWPORT_H_

#include <papyrus.h>
#include <papyrusmm/viewport.h>

#include <gvc.h>
#include <gvcjob.h>

#include <string>

class SkillGuiGraphViewport : public PapyrusGtk::Viewport
{
 public:
  SkillGuiGraphViewport();
  ~SkillGuiGraphViewport();

  void set_gvjob(GVJ_t *job);
  void set_graph_fsm(std::string fsm_name);
  void set_graph(std::string graph);

  void render();

 private:
  GVC_t *__gvc;
  GVJ_t *__gvjob;

  std::string __graph_fsm;
  std::string __graph;
};


#endif
