#ifndef _OBSTACLE_POINT_H_
#define _OBSTACLE_POINT_H_

#include <gts.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GTS_IS_OBSTACLE(obj) (gts_object_is_from_class (obj,\
                                                     gts_obstacle_class ()))
#define GTS_OBSTACLE(obj)              GTS_OBJECT_CAST (obj,\
                                                     GtsObstacle,\
                                                     gts_point_class ())
#define GTS_OBSTACLE_CLASS(klass)      GTS_OBJECT_CLASS_CAST (klass,\
                                                           GtsObstacleClass,\
                                                           gts_Obstacle_class ())
                                                           
  typedef struct {
    GtsPointClass parent_class;
  } GtsObstacleClass;


  typedef struct {
    GtsPoint point;

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
                                             
#endif //_OBSTACLE_POINT_H_
