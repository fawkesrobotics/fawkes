
/***************************************************************************
 *  v4l1.cpp - Implementation to access V4L cam
 *
 *  Generated: Fri Mar 11 17:48:27 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <core/exceptions/software.h>

#include <fvcams/v4l1.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/rgb.h>
#include <fvutils/system/camargp.h>

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>	/* gettimeofday() */
#include <fcntl.h>
#include <unistd.h>
#include <linux/types.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include <cassert>
#include <sys/types.h>
#include <linux/videodev.h>


using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS

class V4L1CameraData
{
 public:
  V4L1CameraData(const char *device_name)
  {
    this->device_name = strdup(device_name);
  }

  ~V4L1CameraData()
  {
    free(device_name);
  }

 public:
  char *device_name;

  /* V4L1 stuff */
  struct video_capability  capabilities;          // Device Capabilities: Can overlay, Number of channels, etc
  struct video_buffer      vbuffer;               // information about buffer
  struct video_window      window;                // Window Information: Size, Depth, etc
  struct video_channel    *channel;               // Channels information: Channel[0] holds information for channel 0 and so on...
  struct video_picture     picture;               // Picture information: Palette, contrast, hue, etc
  struct video_tuner      *tuner;                 // Tuner Information: if the card has tuners...
  struct video_audio       audio;                 // If the card has audio
  struct video_mbuf        captured_frame_buffer; // Information for the frame to be captured: norm, palette, etc
  struct video_mmap       *buf_v4l;               // mmap() buffer VIDIOCMCAPTURE
};

/// @endcond

/** @class V4L1Camera <fvcams/v4l1.h>
 * Video4Linux 1 camera implementation.
 */

/** Constructor.
 * @param device_name device file name (e.g. /dev/video0)
 */
V4L1Camera::V4L1Camera(const char *device_name)
{
  started = opened = false;
  __data = new V4L1CameraData(device_name);
}


/** Constructor.
 * Initialize camera with parameters from camera argument parser.
 * Supported arguments:
 * - device=DEV, device file, for example /dev/video0
 * @param cap camera argument parser
 */
V4L1Camera::V4L1Camera(const CameraArgumentParser *cap)
{
  started = opened = false;
  if ( cap->has("device") ) {
    __data = new V4L1CameraData(cap->get("device").c_str());
  } else {
    throw MissingParameterException("Missing device for V4L1Camera");
  }
}

/** Protected Constructor.
 * Gets called from V4LCamera, when the device has already been opened
 * and determined to be a V4L1 device.
 * @param device_name device file name (e.g. /dev/video0)
 * @param dev file descriptor of the opened device
 */
V4L1Camera::V4L1Camera(const char *device_name, int dev)
{
  started = opened = false;
  __data = new V4L1CameraData(device_name);
  this->dev = dev;

  // getting grabber info in capabilities struct
  if ( (ioctl(dev, VIDIOCGCAP, &(__data->capabilities))) == -1 ) {
    throw Exception("V4L1Cam: Could not get capabilities");
  }

  post_open();
}


/** Destructor. */
V4L1Camera::~V4L1Camera()
{
  delete __data;
}


void
V4L1Camera::open()
{
  opened = false;

  dev = ::open(__data->device_name, O_RDWR);
  if (dev < 0) {
    throw Exception("V4L1Cam: Could not open device");
  }

  // getting grabber info in capabilities struct
  if ( (ioctl(dev, VIDIOCGCAP, &(__data->capabilities))) == -1 ) {
    throw Exception("V4L1Cam: Could not get capabilities");
  }

  post_open();
}

/**
 * Post-open() operations
 * @param dev file descriptor of the opened device
 */
void
V4L1Camera::post_open()
{
  // Capture window information
  if ( (ioctl(dev, VIDIOCGWIN, &__data->window)) == -1) {
    throw Exception("V4L1Cam: Could not get window information");
  }

  // Picture information
  if ( (ioctl(dev, VIDIOCGPICT, &__data->picture)) == -1) {
    throw Exception("V4L1Cam: Could not get window information");
  }

  ///Video Channel Information or Video Sources
  ///Allocate space for each channel
  __data->channel = (struct video_channel*)malloc(sizeof(struct video_channel)*(__data->capabilities.channels+1));
  for(int ch = 0; ch < __data->capabilities.channels; ch++) {
    __data->channel[ch].norm = 0;
    if ( (ioctl(dev, VIDIOCSCHAN, &__data->channel[ch])) == -1) {
      printf("V4L1Cam: Could not get channel information for channel %i: %s", ch, strerror(errno));
    }
  }

  ///Trying to capture through read()
  if (ioctl (dev, VIDIOCGMBUF, __data->captured_frame_buffer) == -1) {
    capture_method = READ;
    frame_buffer = (unsigned char *)malloc(__data->window.width * __data->window.height * RGB_PIXEL_SIZE);
  } else {
    capture_method = MMAP;
    frame_buffer = (unsigned char*)mmap (0, __data->captured_frame_buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED, dev, 0);
    if ((unsigned char *) -1 == (unsigned char *)frame_buffer) {
      throw Exception("V4L1Cam: Cannot initialize mmap region");
    }
  }

  __data->buf_v4l = NULL;

  opened = true;
}


void
V4L1Camera::start()
{

  started = false;
  if (!opened) {
    throw Exception("V4L1Cam: Trying to start closed cam!");
  }

  started = true;
}


void
V4L1Camera::stop()
{
  started = false;
}


void
V4L1Camera::print_info()
{

  if (! opened) return;

  cout << endl << "CAPABILITIES" << endl
       << "===========================================================================" << endl;

  if(__data->capabilities.type & VID_TYPE_CAPTURE)
    cout << " + Can capture to memory" << endl;
  if(__data->capabilities.type & VID_TYPE_TUNER)
    cout << " + Has a tuner of some form" << endl;
  if(__data->capabilities.type & VID_TYPE_TELETEXT)
    cout << " + Has teletext capability" << endl;
  if(__data->capabilities.type & VID_TYPE_OVERLAY)
    cout << " + Can overlay its image onto the frame buffer" << endl;
  if(__data->capabilities.type & VID_TYPE_CHROMAKEY)
    cout << " + Overlay is Chromakeyed" << endl;
  if(__data->capabilities.type & VID_TYPE_CLIPPING)
    cout << " + Overlay clipping is supported" << endl;
  if(__data->capabilities.type & VID_TYPE_FRAMERAM)
    cout << " + Overlay overwrites frame buffer memory" << endl;
  if(__data->capabilities.type & VID_TYPE_SCALES)
    cout << " + The hardware supports image scaling" << endl;
  if(__data->capabilities.type & VID_TYPE_MONOCHROME)
    cout << " + Image capture is grey scale only" << endl;
  if(__data->capabilities.type & VID_TYPE_SUBCAPTURE)
    cout << " + Can subcapture" << endl;

  cout << endl;
  cout << " Number of Channels ='" << __data->capabilities.channels << "'" << endl;
  cout << " Number of Audio Devices ='" << __data->capabilities.audios << "'" << endl;
  cout << " Maximum Capture Width ='" << __data->capabilities.maxwidth << "'" << endl;
  cout << " Maximum Capture Height ='" << __data->capabilities.maxheight << "'" << endl;
  cout << " Minimum Capture Width ='" << __data->capabilities.minwidth << "'" << endl;
  cout << " Minimum Capture Height ='" << __data->capabilities.minheight << "'" << endl;




  cout << endl << "CAPTURE WINDOW INFO" << endl
       << "===========================================================================" << endl;

  cout << " X Coord in X window Format:  " << __data->window.x << endl;
  cout << " Y Coord in X window Format:  " << __data->window.y << endl;
  cout << " Width of the Image Capture:  " << __data->window.width << endl;
  cout << " Height of the Image Capture: " << __data->window.height << endl;
  cout << " ChromaKey:                   " << __data->window.chromakey  << endl;




  cout << endl << "DEVICE PICTURE INFO" << endl
       << "===========================================================================" << endl;

  cout << " Picture Brightness: " << __data->picture.brightness << endl;
  cout << " Picture        Hue: " << __data->picture.hue << endl;
  cout << " Picture     Colour: " << __data->picture.colour << endl;
  cout << " Picture   Contrast: " << __data->picture.contrast << endl;
  cout << " Picture  Whiteness: " << __data->picture.whiteness << endl;
  cout << " Picture      Depth: " << __data->picture.depth << endl;
  cout << " Picture    Palette: " << __data->picture.palette << " (";

  if(__data->picture.palette == VIDEO_PALETTE_GREY)
    cout << "VIDEO_PALETTE_GRAY";
  if(__data->picture.palette == VIDEO_PALETTE_HI240)
    cout << "VIDEO_PALETTE_HI240";
  if(__data->picture.palette == VIDEO_PALETTE_RGB565)
    cout << "VIDEO_PALETTE_RGB565";
  if(__data->picture.palette == VIDEO_PALETTE_RGB555)
    cout << "VIDEO_PALETTE_RGB555";
  if(__data->picture.palette == VIDEO_PALETTE_RGB24)
    cout << "VIDEO_PALETTE_RGB24";
  if(__data->picture.palette == VIDEO_PALETTE_RGB32)
    cout << "VIDEO_PALETTE_RGB32";
  if(__data->picture.palette == VIDEO_PALETTE_YUV422)
    cout << "VIDEO_PALETTE_YUV422";
  if(__data->picture.palette == VIDEO_PALETTE_YUYV)
    cout << "VIDEO_PALETTE_YUYV";
  if(__data->picture.palette == VIDEO_PALETTE_UYVY)
    cout << "VIDEO_PALETTE_UYVY";
  if(__data->picture.palette == VIDEO_PALETTE_YUV420)
    cout << "VIDEO_PALETTE_YUV420";
  if(__data->picture.palette == VIDEO_PALETTE_YUV411)
    cout << "VIDEO_PALETTE_YUV411";
  if(__data->picture.palette == VIDEO_PALETTE_RAW)
    cout << "VIDEO_PALETTE_RAW";
  if(__data->picture.palette == VIDEO_PALETTE_YUV422P)
    cout << "VIDEO_PALETTE_YUV422P";
  if(__data->picture.palette == VIDEO_PALETTE_YUV411P)
    cout << "VIDEO_PALETTE_YUV411P";

  cout << ")" << endl;



  cout << endl << "VIDEO SOURCE INFO" << endl
       << "===========================================================================" << endl;

  cout << " Channel Number or Video Source Number: " << __data->channel->channel << endl;
  cout << " Channel Name:                          " << __data->channel->name << endl;
  cout << " Number of Tuners for this source:      " << __data->channel->tuners << endl;
  cout << " Channel Norm:                          " << __data->channel->norm << endl;
  if(__data->channel->flags & VIDEO_VC_TUNER)
    cout << " + This channel source has tuners" << endl;
  if(__data->channel->flags & VIDEO_VC_AUDIO)
    cout << " + This channel source has audio" << endl;
  if(__data->channel->type & VIDEO_TYPE_TV)
    cout << " + This channel source is a TV input" << endl;
  if(__data->channel->type & VIDEO_TYPE_CAMERA)
    cout << " + This channel source is a Camera input" << endl;




  cout << endl << "FRAME BUFFER INFO" << endl
       << "===========================================================================" << endl;

  cout << " Base Physical Address:  " << __data->vbuffer.base << endl;
  cout << " Height of Frame Buffer: " << __data->vbuffer.height << endl;
  cout << " Width of Frame Buffer:  " << __data->vbuffer.width << endl;
  cout << " Depth of Frame Buffer:  " << __data->vbuffer.depth << endl;
  cout << " Bytes Per Line:         " << __data->vbuffer.bytesperline << endl;



  /* Which channel!?
  cout << endl << "CHANNEL INFO" << endl
       << "===========================================================================" << endl;

  cout << " Channel:          " << ch << " - " << channel[ch].name << endl;
  cout << " Number of Tuners: " << channel[0].tuners << endl;
  cout << " Input Type:       " << channel[ch].type << endl;
  cout << " Flags: " << endl;
  if(channel[0].flags & VIDEO_VC_TUNER)
    cout << " + This Channel Source has Tuners" << endl;
  if(channel[0].flags & VIDEO_VC_AUDIO)
    cout << " + This Channel Source has Audio" << endl;
  //  if(channel[0].flags & VIDEO_VC_NORM)
  //cout << " \tThis Channel Source has Norm\n");
  cout << " Norm for Channel: '" << channel[0].norm << "'" << endl;
  */

}


void
V4L1Camera::capture()
{

  if (capture_method == READ) {
    int len = read(dev, frame_buffer, __data->window.width * __data->window.height * RGB_PIXEL_SIZE);
    if (len < 0) {
      throw Exception("V4L1Cam: Could not capture frame");
    }
  } else {

    __data->buf_v4l = (struct video_mmap*)malloc(__data->captured_frame_buffer.frames * sizeof(struct video_mmap));

    ///Setting up the palette, size of frame and which frame to capture
    __data->buf_v4l[0].format = __data->picture.palette;
    __data->buf_v4l[0].frame  = 0;
    __data->buf_v4l[0].width  = __data->window.width;
    __data->buf_v4l[0].height = __data->window.height;

    if (ioctl (dev, VIDIOCMCAPTURE, &(__data->buf_v4l[0])) == -1) {
      throw Exception("V4L1Cam: Could not capture frame (VIDIOCMCAPTURE)");
    }
    ///Waiting for the frame to finish
    int Frame = 0;
    if (ioctl (dev, VIDIOCSYNC, &Frame) == -1) {
      throw Exception("V4L1Cam: Could not capture frame (VIDIOCSYNC)");
    }
  }
}


void
V4L1Camera::dispose_buffer()
{
  if (capture_method == MMAP) {
    if (__data->buf_v4l != NULL) {
      free(__data->buf_v4l);
      __data->buf_v4l = NULL;
    }
    munmap(frame_buffer, __data->captured_frame_buffer.size);
  }
}


unsigned char*
V4L1Camera::buffer()
{
  return frame_buffer;
}

unsigned int
V4L1Camera::buffer_size()
{
  return colorspace_buffer_size(RGB, __data->window.width, __data->window.height);
}

void
V4L1Camera::close()
{
  if (opened) {
    ::close(dev);
  }
}

unsigned int
V4L1Camera::pixel_width()
{
  if (opened) {
    return __data->window.width;
  } else {
    throw Exception("V4L1Cam::pixel_width(): Camera not opened");
  }
}

unsigned int
V4L1Camera::pixel_height()
{
  if (opened) {
    return __data->window.height;
  } else {
    throw Exception("V4L1Cam::pixel_height(): Camera not opened");
  }
}


colorspace_t
V4L1Camera::colorspace()
{
  return BGR;
}


void
V4L1Camera::flush()
{
}


bool
V4L1Camera::ready()
{
  return started;
}


void
V4L1Camera::set_image_number(unsigned int n)
{
}

} // end namespace firevision
