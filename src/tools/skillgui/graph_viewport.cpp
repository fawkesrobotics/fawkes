
/***************************************************************************
 *  graph_viewport.cpp - FSM Graph Viewport for Skill GUI
 *
 *  Created: Mon Dec 15 15:40:36 2008
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

#include "graph_viewport.h"
#include "gvplugin_skillgui_papyrus.h"

/** @class SkillGuiGraphViewport "graph_viewport.h"
 * Skill FSM Graph Viewport.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillGuiGraphViewport::SkillGuiGraphViewport()
{
  m_scroll_to = PapyrusGtk::SCROLL_TO_TOP_LEFT;

  Cairo::RefPtr<Cairo::SolidPattern> bp = Cairo::SolidPattern::create_rgb(1, 1, 1);

  Papyrus::Canvas::pointer c = canvas();
  c->set_background(bp);

  __gvc = gvContext();
  __gvjob = NULL;

  __graph_fsm = "";
  __graph = "";

  gvplugin_skillgui_setup(__gvc, this);

  signal_size_allocate().connect_notify(sigc::hide(sigc::mem_fun(*this, &SkillGuiGraphViewport::render)));
}


/** Destructor. */
SkillGuiGraphViewport::~SkillGuiGraphViewport()
{
  gvFreeContext(__gvc);
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


/** Render current graph. */
void
SkillGuiGraphViewport::render()
{
  Agraph_t *g = agmemread((char *)__graph.c_str());
  if (g) {
    gvLayout(__gvc, g, (char *)"dot");
    gvRender(__gvc, g, (char *)"skillgui", NULL);
    gvFreeLayout(__gvc, g);
    agclose(g);
  } else {
    clear();
  }
}
