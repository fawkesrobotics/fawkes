
/***************************************************************************
 *  yaml_navgraph.h - Nav graph stored in a YAML file
 *
 *  Created: Thu Sep 20 18:31:06 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __UTILS_GRAPH_YAML_NAVGRAPH_H_
#define __UTILS_GRAPH_YAML_NAVGRAPH_H_

#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class TopologicalMapGraph;

extern TopologicalMapGraph *  load_yaml_navgraph(std::string filename);

} // end of namespace fawkes

#endif
