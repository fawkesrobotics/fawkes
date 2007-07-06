
/***************************************************************************
 *  pipeline.h - This header defines a image processing pipeline for
 *               geegaw
 *
 *  Generated: Tue Apr 10 13:41:19 2007 (based on suricate's pipeline)
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_APPS_GEEGAW_PIPELINE_H_
#define __FIREVISION_APPS_GEEGAW_PIPELINE_H_

/// @cond RCSoftX

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <utils/system/signal.h>

#include <sys/time.h>
#include <list>
#include <string>
#include <map>
#include <vector>

class Camera;
class CameraControl;
class ColorModelLookupTable;
class RelativePositionModel;
class GlobalPositionModel;
class ScanlineModel;
class Classifier;
class ArgumentParser;
class SharedMemoryImageBuffer;
class GeegawConfig;

class GeegawPipeline : SignalHandler {

 public:
  GeegawPipeline(ArgumentParser *argp, GeegawConfig *config, bool object_mode = false);
  ~GeegawPipeline();

  void init();
  void finalize();
  void loop();
  void run(unsigned int delay = 0);
  void run(unsigned int delay, unsigned int times);

  /*
  GlobalPositionModel *   getGlobalBoxPosModel();
  */
  ScanlineModel *         getScanlineModel();
  CameraControl *         getCameraControl();
  void                    getDataTakenTime(long int *sec, long int *usec);

  bool                    obstacles_found();
  std::list< polar_coord_t >  & getObstacles();

  RelativePositionModel * object_relpos();
  float                   object_bearing();
  float                   object_distance();

  void                    pan_tilt(float *pan, float *tilt);

  // keep congruent to Geegaw interface constants
  typedef enum {
    MODE_OBSTACLES      = 0,
    MODE_ADD_OBJECT     = 1,
    MODE_LOSTNFOUND     = 2,
    MODE_RESET_COLORMAP = 3
  } GeegawOperationMode;

  // keep congruent to Geegaw interface constants
  typedef enum {
    ADDSTATUS_NOTRUNNING   = 0,
    ADDSTATUS_INPROGRESS   = 1,
    ADDSTATUS_SUCCESS      = 2,
    ADDSTATUS_FAILURE      = 3
  } GeegawAddStatus;

  void setMode(GeegawOperationMode mode);
  GeegawOperationMode getMode();

  bool addStatusChanged();
  GeegawAddStatus addStatus();

  void setColormap(std::string colormap_filename_without_path);

 private:
  /* private methods */
  void handle_signal(int signum);

  void detect_obstacles();
  void detect_object();
  void add_object();

  ArgumentParser  *argp;
  GeegawConfig  *config;
  std::string      msg_prefix;

  Camera          *cam;
  CameraControl   *camctrl;


  colorspace_t     cspace_from;
  colorspace_t     cspace_to;
  char            *file;
  int              param_width;
  int              param_height;
  bool             quit;

  GeegawOperationMode mode;
  GeegawAddStatus     add_status;
  GeegawAddStatus     last_add_status;

  float x, y, z;

  struct timeval   data_taken_time;

  float _object_bearing;
  float _object_distance;

  float pan;
  float tilt;

  unsigned int              width;
  unsigned int              height;

  unsigned int              buffer_size;
  unsigned char            *buffer;
  unsigned char            *buffer1;
  unsigned char            *buffer2;
  unsigned char            *buffer3;
  unsigned char            *buffer_src;
  SharedMemoryImageBuffer  *shm_buffer;
  SharedMemoryImageBuffer  *shm_buffer_src;

  ScanlineModel            *scanlines;
  ColorModelLookupTable    *cm;
  RelativePositionModel    *rel_pos;
  RelativePositionModel    *object_relposmod;
  //BallGlobal             *box_glob;
  Classifier               *classifier;

  bool                      already_fetched_pantilt;
  cart_coord_t              mass_point;

  // Classifier results and iterators
  std::list< ROI > *rois;
  std::list< ROI >::iterator r;

  std::list< polar_coord_t > obstacles;

  bool             generate_output;

  // Color detection stuff
  unsigned int     determine_cycle_num;
  unsigned int     determined_valid_frames;
  ColorModelLookupTable *deter_cm;
  Classifier            *deter_classifier;
  std::vector<const char *>  deter_colormaps;
  unsigned int               deter_nextcm;
  

};

/// @endcond

#endif
