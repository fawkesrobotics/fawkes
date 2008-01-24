#include <plugins/navigator/gts/gts_obstacle.h>

/** @file gts_obstacle.c
 * Implementation an obstacle in GTS.
 * 
 * @author Martin Liebenberg
 */
/** @struct GtsObstacle
 *  The structure representing a GtsObstacle object. 
 */
 /**  @var GtsObstacle::point
 *  	The point defining the position of this obstacle.
 */
 /**  @var GtsObstacle::width
 * 	The width of this obstacle.
 */
/** @struct GtsObstacleClass
 *  The structure representing the GtsObstacle class.
 */
 /**  @var GtsObstacleClass::parent_class
 * 	The parent class of this class.
 */
 
/** Returns a GtsObstacleClass.
 *
 * @return  GtsObstacleClass
 */
GtsObstacleClass* gts_obstacle_class(void)
{
  static GtsObstacleClass * klass = NULL;

  if (klass == NULL) {
    GtsObjectClassInfo obstacle_info = {
      "GtsObstacle",
      sizeof (GtsObstacle),
      sizeof (GtsObstacleClass),
      (GtsObjectClassInitFunc) NULL,
      (GtsObjectInitFunc) NULL,
      (GtsArgSetFunc) NULL,
      (GtsArgGetFunc) NULL
    };
    klass = (GtsObstacleClass *)gts_object_class_new (gts_object_class (), 
				  &obstacle_info);
  }

  return klass;
}


 /**  Returns a new GtsObstacle.
 * @param klass a GtsObstacleClass.
 * @param x x-coordinate of this GtsObstacle
 * @param y y-coordinate of this GtsObstacle
 * @param z z-coordinate of this GtsObstacle
 * @param width the width of this GtsObstacle
 * @return a new GtsObstacle.
 */
GtsObstacle*   gts_obstacle_new(GtsObstacleClass *klass,
                                             gdouble x,
                                             gdouble y,
                                             gdouble z,
                                             gdouble width)
{
  GtsObstacle * o;
  
  o = GTS_OBSTACLE (gts_object_new (GTS_OBJECT_CLASS (klass)));
  o->vertex.p.x = x;
  o->vertex.p.y = y;
  o->vertex.p.z = z;
  o->width = width;

  return o;
}
