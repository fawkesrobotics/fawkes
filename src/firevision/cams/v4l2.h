
/***************************************************************************
 *  v4l2.h - Video4Linux 2 camera access
 *
 *  Generated: Sat Jul  5 20:40:20 2008
 *  Copyright  2008 Tobias Kellner
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

#include <cams/control/color.h>
#include <cams/control/image.h>

/* Number of buffers to use for memory mapped IO */
#define MMAP_NUM_BUFFERS 4;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;
class V4L2CameraData;

class V4L2Camera:
  public Camera,
  public CameraControlColor,
  public CameraControlImage
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

  virtual unsigned int   pixel_width();
  virtual unsigned int   pixel_height();
  virtual colorspace_t   colorspace();
  virtual fawkes::Time * capture_time();

  virtual void           set_image_number(unsigned int n);

  virtual bool         auto_gain();
  virtual void         set_auto_gain(bool enabled);
  virtual bool         auto_white_balance();
  virtual void         set_auto_white_balance(bool enabled);
  virtual bool         auto_exposure();
  virtual void         set_auto_exposure(bool enabled);
  virtual int          red_balance();
  virtual void         set_red_balance(int red_balance);
  virtual int          blue_balance();
  virtual void         set_blue_balance(int blue_balance);
  virtual int          u_balance();
  virtual void         set_u_balance(int u_balance);
  virtual int          v_balance();
  virtual void         set_v_balance(int v_balance);
  virtual unsigned int brightness();
  virtual void         set_brightness(unsigned int brightness);
  virtual unsigned int contrast();
  virtual void         set_contrast(unsigned int contrast);
  virtual unsigned int saturation();
  virtual void         set_saturation(unsigned int saturation);
  virtual int          hue();
  virtual void         set_hue(int hue);
  virtual unsigned int exposure();
  virtual void         set_exposure(unsigned int exposure);
  virtual unsigned int gain();
  virtual void         set_gain(unsigned int gain);

  virtual const char * format();
  virtual void         set_format(const char *format);
  virtual unsigned int width();
  virtual unsigned int height();
  virtual void         set_size(unsigned int width,
                                unsigned int height);
  virtual bool         horiz_mirror();
  virtual bool         vert_mirror();
  virtual void         set_horiz_mirror(bool enabled);
  virtual void         set_vert_mirror(bool enabled);
  virtual unsigned int fps();
  virtual void         set_fps(unsigned int fps);
  virtual unsigned int lens_x_corr();
  virtual unsigned int lens_y_corr();
  virtual void         set_lens_x_corr(unsigned int x_corr);
  virtual void         set_lens_y_corr(unsigned int y_corr);


 protected:
  V4L2Camera(const char *device_name, int dev);
  virtual void set_one_control(const char *ctrl, unsigned int id, int value);
  virtual int get_one_control(const char *ctrl, unsigned int id);

 private:
  virtual void post_open();
  virtual void select_read_method();
  virtual void select_format();
  virtual void set_fps();
  virtual void set_controls();
  virtual void create_buffer();
  virtual void reset_cropping();

 protected:
  char *_device_name; ///< Device name

 private:
  enum ReadMethod
  {
    READ, ///< read() input
    MMAP, ///< Memory mapping input
    UPTR  ///< User pointer input
  };

  enum TriState
  {
    NOT_SET, ///< parameter not set
    TRUE,    ///< parameter set to true
    FALSE    ///< parameter set to false
  };

  struct FrameBuffer
  {
    unsigned char *buffer;
    unsigned int size;
  };

  struct ControlParameterInt
  {
    bool set;
    int value;
  };

  int _dev;                          ///< Device file descriptor

  V4L2CameraData *_data;

  ReadMethod _read_method;           ///< Used read method
  bool _opened;                      ///< Device has been open()ed
  bool _started;                     ///< Device has been start()ed
  char _format[5];                   ///< FourCC of the image format
  colorspace_t _colorspace;          ///< Used colorspace_t

  unsigned int _width;               ///< Image width
  unsigned int _height;              ///< Image height
  unsigned int _bytes_per_line;      ///< Image bytes per line
  FrameBuffer *_frame_buffers;       ///< Image buffers
  unsigned int _buffers_length;      ///< Image buffer size
  int _current_buffer;               ///< Current Image buffer (-1 if not set)
  fawkes::Time *_capture_time;       ///< Time when last picture was captured

  bool _switch_u_v;                  ///< Switch U and V channels
  unsigned int _fps;                 ///< Capture FPS

  TriState _aec;                     ///< Auto Exposition enabled
  TriState _awb;                     ///< Auto White Balance enabled
  TriState _agc;                     ///< Auto Gain enabled
  TriState _h_flip;                  ///< Horizontal mirror
  TriState _v_flip;                  ///< Vertical mirror
  ControlParameterInt _brightness;   ///< Brightness [0-255] (def. 128)
  ControlParameterInt _contrast;     ///< Contrast [0-127] (def. 64)
  ControlParameterInt _saturation;   ///< Saturation [0-256] (def. 128)
  ControlParameterInt _hue;          ///< Hue [-180-180] (def. 0)
  ControlParameterInt _red_balance;  ///< Red Balance [0-255] (def. 128)
  ControlParameterInt _blue_balance; ///< Blue Balance [0-255] (def. 128)
  ControlParameterInt _exposure;     ///< Exposure [0-65535] (def. 60)
  ControlParameterInt _gain;         ///< Gain [0-255] (def. 0)
  ControlParameterInt _lens_x;       ///< Lens Correction X [0-255] (def. 0)
  ControlParameterInt _lens_y;       ///< Lens Correction Y [0-255] (def. 0)

  bool _nao_hacks;                   ///< Nao-specific hacks (bad driver)

};

} // end namespace firevision

#endif //__FIREVISION_CAMS_V4L2_H_
