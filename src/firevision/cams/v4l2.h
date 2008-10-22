
/***************************************************************************
 *  v4l2.h - Video4Linux 2 camera access
 *
 *  Generated: Sat Jul  5 20:40:20 2008
 *  Copyright  2008 Tobias Kellner
 *
 *  $Id$
 *
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

#ifndef __FIREVISION_CAMS_V4L2_H_
#define __FIREVISION_CAMS_V4L2_H_

#include <cams/camera.h>

#include <linux/types.h>
#include <linux/videodev2.h>

class CameraArgumentParser;
class V4L2CameraData;

class V4L2Camera: public Camera 
{
  friend class V4LCamera;

 public:
  V4L2Camera(const char *device_name = "/dev/video0");
  V4L2Camera(const CameraArgumentParser *cap);
  virtual ~V4L2Camera();

  virtual void open();
  virtual void start();
  virtual void stop();
  virtual void close();
  virtual void flush();
  virtual void capture();
  virtual void print_info();
  virtual bool ready();

  virtual unsigned char *buffer();
  virtual unsigned int   buffer_size();
  virtual void           dispose_buffer();

  virtual unsigned int    pixel_width();
  virtual unsigned int    pixel_height();
  virtual colorspace_t    colorspace();

  virtual void           set_image_number(unsigned int n);

 protected:
  V4L2Camera(const char *device_name, int dev);

 private:
  virtual void post_open();
  virtual void select_read_method();
  virtual void select_format();
  virtual void set_fps();
  virtual void set_controls();
  virtual void set_one_control(const char *ctrl, unsigned int id, int value);
  virtual void create_buffer();
  virtual void reset_cropping();

 private:
  enum ReadMethod
  {
    READ, //< read() input
    MMAP, //< Memory mapping input
    UPTR  //< User pointer input
  };

  enum TriState
  {
    NOT_SET, //< parameter not set
    TRUE,    //< parameter set to true
    FALSE    //< parameter set to false
  };

  struct ControlParameterInt
  {
    bool set;
    int value;
  };

  char *_device_name;           //< Device name
  int _dev;                     //< Device file descriptor

  V4L2CameraData *_data;

  ReadMethod _read_method;      //< Used read method
  bool _opened;                 //< Device has been open()ed
  bool _started;                //< Device has been start()ed
  char _format[5];              //< FourCC of the image format
  colorspace_t _colorspace;     //< Used colorspace_t

  unsigned int _width;          //< Image width
  unsigned int _height;         //< Image height
  unsigned int _bytes_per_line; //< Image bytes per line
  unsigned char *_frame_buffer; //< Image buffer
  unsigned int _buffer_size;    //< Image buffer size

  bool _switch_u_v;             //< Switch U and V channels
  unsigned int _fps;            //< Capture FPS

  TriState _aec;                //< Auto Exposition enabled
  TriState _awb;                //< Auto White Balance enabled
  TriState _agc;                //< Auto Gain enabled
  TriState _h_flip;             //< Horizontal mirror
  TriState _v_flip;             //< Vertical mirror
  ControlParameterInt _brightness;              //< Brightness [0-255] (def. 128)
  ControlParameterInt _contrast;                //< Contrast [0-127] (def. 64)
  ControlParameterInt _saturation;              //< Saturation [0-256] (def. 128)
  ControlParameterInt _hue;                     //< Hue [-180-180] (def. 0)
  ControlParameterInt _red_balance;             //< Red Balance [0-255] (def. 128)
  ControlParameterInt _blue_balance;            //< Blue Balance [0-255] (def. 128)
  ControlParameterInt _exposure;                //< Exposure [0-65535] (def. 60)
  ControlParameterInt _gain;                    //< Gain [0-255] (def. 0)
  ControlParameterInt _lens_x;                  //< Lens Correction X [0-255] (def. 0)
  ControlParameterInt _lens_y;                  //< Lens Correction Y [0-255] (def. 0)

  bool _nao_hacks;              //< Nao-specific hacks (bad driver)

};

#endif //__FIREVISION_CAMS_V4L2_H_
