
/***************************************************************************
 *  pipeline.h - This header defines a image processing pipeline for
 *               cannikin
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

#ifndef __FIREVISION_APPS_CANNIKIN_PIPELINE_H_
#define __FIREVISION_APPS_CANNIKIN_PIPELINE_H_

/// @cond RCSoftX

#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <utils/system/signal.h>

#include <cams/camera.h>
#include <cams/cameracontrol.h>

#include <sys/time.h>
#include <list>
#include <string>
#include <map>

class Camera;
class CameraControl;
class ColorModelLookupTable;
class RelativePositionModel;
class GlobalPositionModel;
class ScanlineModel;
class Classifier;
class ArgumentParser;
class SharedMemoryImageBuffer;
class CannikinConfig;
class ScanlineRadial;
class TriclopsStereoProcessor;

class CannikinPipeline : SignalHandler {

 public:
  typedef enum {
    CC_YELLOW = 0,
    CC_GREEN  = 1,
    CC_BLUE   = 2,
    CC_RED    = 3,
    CC_ORANGE = 4
  } cup_color_t;

  typedef enum{
    DETECT_CUP = 0,
    DETERMINE_CUP_COLOR = 1,
  } cannikin_mode_t;

  CannikinPipeline(ArgumentParser *argp, CannikinConfig *config);
  ~CannikinPipeline();

  void init();
  void finalize();
  void loop();
  void run(unsigned int delay = 0);
  void run(unsigned int delay, unsigned int times);

  /*
  RelativePositionModel * getRelativeBoxPosModel();
  GlobalPositionModel *   getGlobalBoxPosModel();
  */
  ScanlineModel *         getScanlineModel();
  CameraControl *         getCameraControl();
  void                    getDataTakenTime(long int *sec, long int *usec);

  void                    set_mode(cannikin_mode_t m);
  cannikin_mode_t         mode();

  cup_color_t             cup_color();
  void                    set_cup_color(cup_color_t c);
  bool                    is_cup_visible();

  bool                    done_determining_cup_color();
  cup_color_t             determined_cup_color();

  void                    set_colormap(cup_color_t c, const char *file);
  bool                    get_xyz(float *x, float *y, float *z);
  bool                    get_world_xyz(float *x, float *y, float *z);

 private:

  void detect_cup();
  void determine_cup_color();
  void reinitialize_colormap();

  typedef enum {
    CANNIKIN_STATE_UNINITIALIZED,
    CANNIKIN_STATE_REINITIALIZE_COLORMAP,
    CANNIKIN_STATE_DETECTION,
    CANNIKIN_STATE_DETERMINE_CUP_COLOR
  } cannikin_state_t;

  ArgumentParser  *argp;
  CannikinConfig  *config;
  std::string      msg_prefix;

  Camera          *cam;
  CameraControl   *camctrl;
  TriclopsStereoProcessor *triclops;

  colorspace_t     cspace_from;
  colorspace_t     cspace_to;
  char            *file;
  int              param_width;
  int              param_height;
  bool             use_fileloader;
  bool             old_file_format;
  bool             quit;

  cannikin_mode_t  _mode;
  cannikin_state_t state;
  cannikin_state_t last_state;

  // Color detection stuff
  unsigned int     determine_cycle_num;
  unsigned int     determined_valid_frames;
  cup_color_t      _cup_color;
  cup_color_t      _determined_cup_color;

  bool             cup_visible;
  bool             cup_color_determination_done;

  float x, y, z, wx, wy, wz;

  struct timeval   data_taken_time;

  std::string               colormap_filestem;
  std::string::size_type    colormap_filestem_cindex;

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
  //Ball                   *box_rel;
  //BallGlobal             *box_glob;
  Classifier               *classifier;

  ScanlineRadial           *disparity_scanlines;

  bool                      already_fetched_pantilt;
  cart_coord_t              mass_point;

  // Classifier results and iterators
  std::list< ROI > *rois;
  std::list< ROI >::iterator r;

  std::map<cup_color_t, char *> colormaps;

  bool             generate_output;

  /* private methods */

  void handle_signal(int signum);

};

/// @endcond

#endif
