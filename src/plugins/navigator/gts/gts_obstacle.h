
/***************************************************************************
 *  gts_obstacle.h - GTS Obstacle Wrapper
 *
 *  Created: Thu Jul 19 2007 16:01:37
 *  Copyright  2007-2008  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _PLUGINS_NAVIGATOR_GTS_GTS_OBSTACLE_H_
#define _PLUGINS_NAVIGATOR_GTS_GTS_OBSTACLE_H_

#include <gts.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GTS_IS_OBSTACLE(obj) (gts_object_is_from_class (obj,\
                                                     gts_obstacle_class ()))
#define GTS_OBSTACLE(obj)              GTS_OBJECT_CAST (obj,\
                                                     GtsObstacle,\
                                                     gts_vertex_class ())
#define GTS_OBSTACLE_CLASS(klass)      GTS_OBJECT_CLASS_CAST (klass,\
                                                           GtsObstacleClass,\
                                                           gts_Obstacle_class ())
                                                           
  typedef struct {
    GtsVertexClass parent_class;
  } GtsObstacleClass;


  typedef struct {
    GtsVertex vertex;

    gdouble width; 
  } GtsObstacle;


  GtsObstacleClass* gts_obstacle_class(void);

  GtsObstacle*   gts_obstacle_new(GtsObstacleClass *klass,
                                  gdouble x,
                                  gdouble y,
                                  gdouble z,
                                  gdouble width);

#ifdef __cplusplus
}
#endif
                                             
#endif
