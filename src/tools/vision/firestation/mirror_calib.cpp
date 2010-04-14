
/***************************************************************************
 *  mirror_calib.cpp - Mirror calibration tool
 *
 *  Created: Fri Dec 07 18:35:40 2007
 *  Copyright  2007  Daniel Beck
 *  Copyright  2009  Christoph Schwering
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

#include "mirror_calib.h"

#include <core/exception.h>
#include <utils/math/angle.h>

#include <fvutils/color/yuv.h>
#include <fvutils/readers/pnm.h>

#include <filters/sobel.h>
#include <filters/sharpen.h>
#include <filters/median.h>
#include <filters/or.h>
#include <filters/laplace.h>
#include <filters/min.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cassert>

//#define FILTER_HOLES
#define FILTER_MINI_HOLES

using namespace std;
using namespace fawkes;
#ifdef HAVE_BULB_CREATOR
using namespace firevision;
#endif

/** @class MirrorCalibTool mirror_calib.h
 * This class encapsulates the routines necessary for interactive mirror
 * calibration.
 */

namespace {
  const unsigned int  ORIENTATION_COUNT         = (ORI_DEG_360 - ORI_DEG_0);
  const unsigned int  MARK_COUNT                = 7;
  const float         MARK_DISTANCE             = 29.7;
  const unsigned char DARK                      = 0;
  const unsigned char BRIGHT                    = 255;
  const unsigned char MARK_LUMA                 = 128;
  const unsigned char MARK_CHROMINANCE          = 255;
  const int           MIN_WIDTH_OF_BIGGEST_LINE = 200;
  const float         APPROX_LINE_WIDTH_LOSS    = 0.05f;
  const float         MIN_BRIGHT_FRACTION       = 0.20f;
}


class MirrorCalibTool::StepResult {
 private:
  unsigned char* buffer_;
  size_t buflen_;
  int width_;
  int height_;
  unsigned int* refcount_;

 public:
  StepResult(size_t buflen, int width, int height)
  : buffer_(new unsigned char[buflen]),
    buflen_(buflen),
    width_(width),
    height_(height),
    refcount_(new unsigned int)
  {
    *refcount_ = 1;
  }

  StepResult(const StepResult& copy)
  : buffer_(copy.buffer_),
    buflen_(copy.buflen_),
    width_(copy.width_),
    height_(copy.height_),
    refcount_(copy.refcount_)
  {
    ++*refcount_;
  }

  StepResult& operator=(const StepResult& copy)
  {
    if (this != &copy) {
      if (--*refcount_ == 0) {
        delete[] buffer_;
        delete refcount_;
      }
      buffer_ = copy.buffer_;
      refcount_ = copy.refcount_;
      buflen_ = copy.buflen_;
      width_ = copy.width_;
      height_ = copy.height_;
      ++*refcount_;
    }
    return *this;
  }

  ~StepResult()
  {
    if (--*refcount_ == 0) {
      delete[] buffer_;
      delete refcount_;
    }
  }

  inline unsigned char* yuv_buffer() { return buffer_; }
  inline const unsigned char* yuv_buffer() const { return buffer_; }
  inline size_t buflen() const { return buflen_; }
  inline int width() const { return width_; } 
  inline int height() const { return height_; }
}; // StepResult


class MirrorCalibTool::Point {
 public:
  const int x;
  const int y;

  Point(int x, int y)
  : x(x),
    y(y)
  {
  }

  PolarRadius length() const
  {
    return static_cast<PolarRadius>(sqrt(x*x + y*y));
  }

  inline PolarAngle atan() const
  {
    return normalize_rad(atan2(y, x));
  }

  Point operator=(const Point& p)
  {
    if (&p == this) {
      return *this;
    }
    return Point(p.x, p.y);
  }
}; // Point


class MirrorCalibTool::CartesianPoint
: public Point
{
 public:
  CartesianPoint(PolarAngle phi, PolarRadius length)
  : Point(length * cos(phi),
          length * sin(phi))
  {
  }

  CartesianPoint(int x, int y)
  : Point(x, y)
  {
  }

  CartesianPoint rotate(PolarAngle rotate_phi) const
  {
    const PolarRadius len = length();
    const PolarAngle phi = atan() + rotate_phi;
    const int x = len * cos(phi);
    const int y = len * sin(phi);
    return CartesianPoint(x, y);
  }
}; // CartesianPoint


class MirrorCalibTool::PixelPoint
: public Point
{
 public:
  PixelPoint(int x, int y)
  : Point(x, y)
  {
  }

  PixelPoint rotate(PolarAngle rotate_phi) const
  {
    const PolarRadius len = length();
    const PolarAngle phi = atan() + rotate_phi;
    const int x = len * cos(phi);
    const int y = len * sin(phi);
    return PixelPoint(x, y);
  }
}; // PixelPoint


/** Wraps an image so that access to (0, 0) is mapped to the middle of the
 *  image and so on. The result is a cartesian coordinate system with x and
 *  y axis. */
class MirrorCalibTool::CartesianImage
{
 private:
  unsigned char* buf_;
  const int width_;
  const int height_;
  const PixelPoint center_;
  const PolarAngle phi_;
  const unsigned char* mask_;

 public:
  CartesianImage(const StepResult& res,
                 PolarAngle phi,
                 PixelPoint center,
                 const unsigned char* mask = 0)
  : buf_(const_cast<unsigned char*>(res.yuv_buffer())),
    width_(res.width()),
    height_(res.height()),
    center_(center),
    phi_(phi),
    mask_(mask)
  {
  }

  CartesianImage(const StepResult& res,
                 PolarAngle phi,
                 const unsigned char* mask = 0)
  : buf_(const_cast<unsigned char*>(res.yuv_buffer())),
    width_(res.width()),
    height_(res.height()),
    center_(PixelPoint(res.width()/2, res.height()/2)),
    phi_(phi),
    mask_(mask)
  {
  }

  CartesianImage(const unsigned char* buf,
                 int width,
                 int height,
                 PolarAngle phi,
                 const PixelPoint& center)
  : buf_(const_cast<unsigned char*>(buf)),
    width_(width),
    height_(height),
    center_(center),
    phi_(phi),
    mask_(0)
  {
  }

  CartesianImage(unsigned char* buf,
                 int width,
                 int height,
                 PolarAngle phi,
                 const PixelPoint& center)
  : buf_(buf),
    width_(width),
    height_(height),
    center_(center),
    phi_(phi),
    mask_(0)
  {
  }

  CartesianImage(const unsigned char* buf,
                 int width,
                 int height,
                 PolarAngle phi)
  : buf_(const_cast<unsigned char*>(buf)),
    width_(width),
    height_(height),
    center_(PixelPoint(width/2, height/2)),
    phi_(phi),
    mask_(0)
  {
  }

  CartesianImage(unsigned char* buf,
                 int width,
                 int height,
                 PolarAngle phi)
  : buf_(buf),
    width_(width),
    height_(height),
    center_(PixelPoint(width/2, height/2)),
    phi_(phi),
    mask_(0)
  {
  }

  inline unsigned char* buf() { return buf_; }
  inline const unsigned char* mask() const { return mask_; }
  inline const unsigned char* buf() const { return buf_; }
  inline const PixelPoint& center() const { return center_; }
  inline const int width() const { return width_; }
  inline const int height() const { return height_; }
  inline const PolarAngle phi() const { return phi_; }

  inline PixelPoint
  to_pixel(const CartesianPoint& p) const
  {
    const CartesianPoint rp = p.rotate(-1.0 * phi());
    return PixelPoint(center().x + rp.x, center().y - rp.y);
  }

  inline CartesianPoint
  to_cartesian(const PixelPoint& p) const
  {
    const CartesianPoint cp(p.x - center().x, center().y - p.y);
    return cp.rotate(phi());
  }

  inline bool
  contains(const PixelPoint& p) const
  {
    return 0 <= p.x && p.x <= width()-1 && 0 <= p.y && p.y <= height()-1;
  }

  inline bool
  contains(const CartesianPoint& p) const
  {
    return contains(to_pixel(p));
  }

  inline unsigned char
  get(const CartesianPoint& p) const
  {
    if (!contains(p)) {
      throw fawkes::Exception("Point p is out of image");
    }
    PixelPoint pp = to_pixel(p);
    const firevision::YUV_t ignr(0);
    if (mask() == 0 ||
        (YUV422_PLANAR_Y_AT(mask(), width(), pp.x, pp.y) != ignr.Y &&
         YUV422_PLANAR_U_AT(mask(), width(), height(), pp.x, pp.y) != ignr.U &&
         YUV422_PLANAR_V_AT(mask(), width(), height(), pp.x, pp.y) != ignr.V)) {
      return YUV422_PLANAR_Y_AT(buf(), width(), pp.x, pp.y);
    } else {
      if (mask() != 0) {
        //printf("Ignoring (%lf,%d) = (%d,%d)\n", p.atan(), p.length(), pp.x, pp.y);
      }
      return 0;
    }
  }

  inline int max_x() const { return max(center().x, width() - center().x); }
  inline int max_y() const { return max(center().y, height() - center().y); }
  inline PolarRadius max_radius() const {
    return static_cast<PolarRadius>(sqrt(max_x()*max_x() + max_y()*max_y()));
  }

  void
  set_color(const PixelPoint& p,
            unsigned char luma,
            unsigned char chrominance)
  {
    if (!contains(p)) {
      throw fawkes::Exception("Point p is out of image");
    }
    YUV422_PLANAR_Y_AT(buf(), width(), p.x, p.y) = luma;
    YUV422_PLANAR_U_AT(buf(), width(), height(), p.x, p.y) = chrominance;
  }

  void
  set_color(const CartesianPoint& p,
            unsigned char luma,
            unsigned char chrominance)
  {
    set_color(to_pixel(p), luma, chrominance);
  }

  /** Returns the relative amount of BRIGHT pixels in the rectangle denoted
   * by the bottom-left (x1, y1) and the top-right (x2, y2). */
  float
  bright_fraction(const CartesianPoint& from,
                  const CartesianPoint& to) const
  {
    const int from_x = from.x < to.x ? from.x : to.x;
    const int to_x   = from.x > to.x ? from.x : to.x;
    const int from_y = from.y < to.y ? from.y : to.y;
    const int to_y   = from.y > to.y ? from.y : to.y;
    int pixel_count = 0;
    int bright_count = 0;
    for (int x = from_x; x <= to_x; x++) {
      for (int y = from_y; y <= to_y; y++) {
        const CartesianPoint p(x, y);
        if (contains(p)) {
          if (get(p) == BRIGHT) {
            ++bright_count;
          }
        }
        ++pixel_count;
      }
    }
    return static_cast<float>(static_cast<double>(bright_count)
                            / static_cast<double>(pixel_count));
  }

  bool
  is_line(const CartesianPoint& p,
          int length) const
  {
    for (int y_offset = 0; y_offset <= 1; y_offset++) {
      const CartesianPoint vec(p.atan() + deg2rad(90.0), length / 2 + 1);
      const CartesianPoint from(p.x - vec.x, p.y - vec.y);
      const CartesianPoint to(p.x + vec.x, p.y + vec.y);
      if (bright_fraction(from, to) >= MIN_BRIGHT_FRACTION) {
        return true;
      }
    }
    return false;
  }

  void
  highlight_line(const CartesianPoint& p,
                 int length)
  {
    const CartesianPoint vec(p.atan() + deg2rad(90.0), length / 2);
    const CartesianPoint from(p.x - vec.x, p.y - vec.y);
    const CartesianPoint to(p.x + vec.x, p.y + vec.y);
    draw_line(from, to);
  }

  void
  draw_line(const PixelPoint& p,
            const PixelPoint& q)
  {
    draw_line(to_cartesian(p), to_cartesian(q));
  }

  void
  draw_line(const CartesianPoint& p,
            const CartesianPoint& q)
  {
    const CartesianPoint distVec(q.x - p.x, q.y - p.y);
    for (PolarRadius length = 0; length <= distVec.length(); length++) {
      const CartesianPoint step(distVec.atan(), length);
      const CartesianPoint linePoint(p.x + step.x, p.y + step.y);
      if (contains(linePoint)) {
        set_color(linePoint, MARK_LUMA, MARK_CHROMINANCE);
      }
    }
  }

  void
  highlight_polygon(const ConvexPolygon& poly)
  {
    for (ConvexPolygon::size_type i = 1; i <= poly.size(); i++) {
      const PixelPoint& p = poly[i-1];
      const PixelPoint& q = poly[i % poly.size()];
      draw_line(p, q);
    }
  }

  void
  highlight_pixel(const PixelPoint& p)
  {
    for (int xx = p.x-5; xx <= p.x+5; xx++) {
      for (int yy = p.y-5; yy <= p.y+5; yy++) {
        const PixelPoint pp(xx, yy);
        if (contains(pp)) {
          set_color(pp, MARK_LUMA, MARK_CHROMINANCE);
        }
      }
    }
  }

  void
  highlight_point(const CartesianPoint& p)
  {
    for (int xx = p.x-5; xx <= p.x+5; xx++) {
      for (int yy = p.y-5; yy <= p.y+5; yy++) {
        const CartesianPoint hp(xx, yy);
        if (contains(hp)) {
          set_color(hp, MARK_LUMA, MARK_CHROMINANCE);
        }
      }
    }
  }
}; // CartesianImage


/** @class MirrorCalibTool::ConvexPolygon tools/firestation/mirror_calib.h
 * Represents a convex polygon. It is defined by a sequence of points in
 * clock-wise-order.
 */

/** Returns true if the point is inside the convex polygon.
 * The point r is converted to PixelPoint wrt img. */
bool
MirrorCalibTool::ConvexPolygon::contains(const CartesianImage& img,
                                         const CartesianPoint& r) const
{
  return contains(img.to_pixel(r));
}


/** Returns true if the point is inside the convex polygon.
 * This is the case if for all points p, q in the polygon p_1, ..., p_n
 * where p = p_i, q = p_{i+1} for some i or p = p_n, q = p_1 it holds
 * (p, q, r) does not form a left turn (if they do, they are
 * counter-clock-wise).
 */
bool
MirrorCalibTool::ConvexPolygon::contains(const PixelPoint& r) const
{
  for (std::vector<PixelPoint>::size_type i = 1; i <= size(); i++) {
    const PixelPoint& p = at(i-1);
    const PixelPoint& q = at(i % size());
    double val = (q.x - p.x) * (r.y - p.y) - (r.x - p.x) * (q.y - p.y);
    if (val < 0) { // (p, q, r) forms a left turn
      return false;
    }
  }
  return true;
}


/** A hole is a sequence of pixels between two lines. */
class MirrorCalibTool::Hole {
 public:
  int index;
  PolarRadius from_length;
  PolarRadius to_length;

  Hole(int index,
       PolarRadius from_length,
       PolarRadius to_length)
  : index(index),
    from_length(from_length),
    to_length(to_length)
  {
  }

  inline PolarRadius size() const { return to_length - from_length; }
}; // Hole


/** A container for a YUV-buffer etc. */
class MirrorCalibTool::Image
{
 public:
  Image(const unsigned char* yuv_buffer,
        size_t buflen,
        int width,
        int height,
        PolarAngle ori)
  : yuv_buffer_(new unsigned char[buflen]),
    buflen_(buflen),
    width_(width),
    height_(height),
    ori_(ori),
    refcount_(new unsigned int)
  {
    memcpy(yuv_buffer_, yuv_buffer, buflen);
    *refcount_ = 1;
  }

  Image(const Image& copy)
  : yuv_buffer_(copy.yuv_buffer_),
    buflen_(copy.buflen_),
    width_(copy.width_),
    height_(copy.height_),
    ori_(copy.ori_),
    results_(copy.results_),
    premarks_(copy.premarks_),
    marks_(copy.marks_),
    refcount_(copy.refcount_)
  {
    ++*refcount_;
  }

  Image& operator=(const Image& copy)
  {
    if (this != &copy) {
      if (--*refcount_ == 0) {
        delete[] yuv_buffer_;
        delete refcount_;
      }
      yuv_buffer_ = copy.yuv_buffer_;
      buflen_     = copy.buflen_;
      width_      = copy.width_;
      height_     = copy.height_;
      ori_        = copy.ori_;
      results_    = copy.results_;
      premarks_   = copy.premarks_;
      marks_      = copy.marks_;
      refcount_   = copy.refcount_;
      ++*copy.refcount_;
    }
    return *this;
  }

  ~Image()
  {
    if (--*refcount_ == 0) {
      delete[] yuv_buffer_;
      delete refcount_;
    }
  }

  inline unsigned char* yuv_buffer() { return yuv_buffer_; }
  inline const unsigned char* yuv_buffer() const { return yuv_buffer_; }
  inline size_t buflen() const { return buflen_; }
  inline int width() const { return width_; }
  inline int height() const { return height_; }
  inline PolarAngle ori() const { return ori_; } /* angle of marks wrt X axis */
  inline StepResultList& results() { return results_; }
  inline const StepResultList& results() const { return results_; }
  inline StepResult& result(int i) { return results_[i]; }
  inline const StepResult& result(int i) const { return results_[i]; }
  inline void add_result(const StepResult& r) { results_.push_back(r); }
  inline const MarkList& premarks() { return premarks_; }
  inline void set_premarks(const MarkList& premarks) { premarks_ = premarks; }
  inline MarkList& marks() { return marks_; }
  inline const MarkList& marks() const { return marks_; }
  inline void set_marks(const MarkList& marks) { marks_ = marks; }

 private:
  unsigned char* yuv_buffer_;
  size_t         buflen_;
  int            width_;
  int            height_;
  PolarAngle     ori_;
  StepResultList results_;
  MarkList       premarks_;
  MarkList       marks_;
  unsigned int*  refcount_;
}; // Image


/** Constructor. */
MirrorCalibTool::MirrorCalibTool()
  : img_yuv_buffer_(0),
    img_center_x_(500),
    img_center_y_(500),
    img_yuv_mask_(0),
    state_(CalibrationState())
#ifdef HAVE_BULB_CREATOR
  , bulb_(0),
    m_generator(0)
#endif
{
}


/** Destructor. */
MirrorCalibTool::~MirrorCalibTool()
{
  if (img_yuv_buffer_) {
    delete[] img_yuv_buffer_;
  }
  if (img_yuv_mask_) {
    delete[] img_yuv_mask_;
  }
}


/**
 * Converts an angle relative to the robots view to the needed image rotation
 * so that the things, which lie at angle `ori' from robot's perspective, are
 * on the X axis of the image.
 * For example: if the marks are 120 degrees counter-clock-wise from the robot,
 * the image needs to be rotated 120 degrees clock-wise (then the marks are 
 * in front of the robot, i.e. Y axis) and then 90 degrees clock-wise
 * (to get the marks from the Y axis to the X axis).
 *
 * The reason that we want to have the marks on the X axis is that calculating
 * with them is then a bit easier (because then their polar angle is always
 * 0.0).
 */
MirrorCalibTool::PolarAngle
MirrorCalibTool::robotRelativeOrientationToImageRotation(PolarAngle ori)
{
  return normalize_rad(-1.0 * ori + deg2rad(-90.0));
}


/**
 * Converts the rotation of the image to the orientation relative to the robot's
 * view.
 * Just see the documentation of robotRelativeOrientationToImageRotation()
 * of which this is the inverse.
 */
MirrorCalibTool::PolarAngle
MirrorCalibTool::imageRotationToRobotRelativeOrientation(PolarAngle ori)
{
  return normalize_rad(-1.0 * (ori + deg2rad(90.0)));
}


void
MirrorCalibTool::load_mask(const char* mask_file_name)
{
  if (img_yuv_mask_) {
    delete[] img_yuv_mask_;
  }
#if 0 // delete the enclosed code?
  size_t hack_size = 2 * 1000 * 1000;
  if (!source_images_.empty()) {
    size_t width = static_cast<size_t>(source_images_.front().width());
    size_t height = static_cast<size_t>(source_images_.front().height());
    hack_size = 2 * width * height;
  }
  img_yuv_mask_ = new unsigned char[hack_size];
#endif
  PNMReader reader(mask_file_name);
  size_t size = colorspace_buffer_size(reader.colorspace(),
                                       reader.pixel_width(),
                                       reader.pixel_height());
  img_yuv_mask_ = new unsigned char[size];
#if 0 // delete the enclosed code?
  if (size != hack_size) {
    throw fawkes::Exception("Size hack didn't work. PNM-Mask has unexpected size.");
  }
#endif
  reader.set_buffer(img_yuv_mask_);
  reader.read();
}


/** Store image for calibration process.
 * @param yuv_buffer The image's yuv_buffer. It may be freeed by the caller
 *                   immediately, the MirrorCalibTool works with a copy of it.
 * @param buflen     The length of yuv_buffer.
 * @param width      The width of the image.
 * @param height     The height of the image.
 * @param ori        The polar angle in degrees (!) where the marks are.
 */
void
MirrorCalibTool::push_back(const unsigned char* yuv_buffer,
                           size_t buflen,
                           int width,
                           int height,
                           double ori)
{
  ori = robotRelativeOrientationToImageRotation(ori);
  Image src_img(yuv_buffer, buflen, width, height, ori);
  source_images_.push_back(src_img);
}


/** Aborts the calibration process. */
void
MirrorCalibTool::abort()
{
  img_center_x_       = 500;
  img_center_y_       = 500;
  state_              = CalibrationState();
  source_images_.clear();
  premarks_.clear();
  mark_map_.clear();
}


/** Applies a sobel filter for edge detection in some direction. */
void
MirrorCalibTool::apply_sobel(unsigned char* src,
                             unsigned char* dst,
                             int width,
                             int height,
                             orientation_t ori)
{
  ROI* roi = ROI::full_image(width, height);
  FilterSobel filter;
  filter.set_src_buffer(src, roi, ori, 0);
  filter.set_dst_buffer(dst, roi);
  filter.apply();
}


/** Applies a sharpening filter. */
void
MirrorCalibTool::apply_sharpen(unsigned char* src,
                               unsigned char* dst,
                               int width,
                               int height)
{
  ROI* roi = ROI::full_image(width, height);
  FilterSharpen filter;
  filter.set_src_buffer(src, roi, 0);
  filter.set_dst_buffer(dst, roi);
  filter.apply();
}


/** Applies a median filter. */
void
MirrorCalibTool::apply_median(unsigned char* src,
                              unsigned char* dst,
                              int width,
                              int height,
                              int i)
{
  ROI* roi = ROI::full_image(width, height);
  FilterMedian filter(i);
  filter.set_src_buffer(src, roi, 0);
  filter.set_dst_buffer(dst, roi);
  filter.apply();
}


/** Applies a minimum filter. */
void
MirrorCalibTool::apply_min(unsigned char* src,
                           unsigned char* dst,
                           int width,
                           int height)
{
  ROI* roi = ROI::full_image(width, height);
  FilterMin filter;
  filter.set_src_buffer(src, roi, 0);
  filter.set_dst_buffer(dst, roi);
  filter.apply();
}


/** Applies a disjunction filter. */
void
MirrorCalibTool::apply_or(unsigned char* src1,
                          unsigned char* src2,
                          unsigned char* dst,
                          int width,
                          int height)
{
  ROI* roi = ROI::full_image(width, height);
  FilterOr filter;
  filter.set_src_buffer(src1, roi, 0);
  filter.set_src_buffer(src2, roi, 1);
  filter.set_dst_buffer(dst, roi);
  filter.apply();
}


/** Sets all pixels to black or white (i.e. maximum contrast). */
void
MirrorCalibTool::make_contrast(unsigned char* buf,
                               size_t buflen)
{
  for (unsigned int i = 0; i < buflen/2; i++) {
    buf[i] = (buf[i] >= 75) ? BRIGHT : DARK;
  }
}


/** Sets all chrominance values to 128. */
void
MirrorCalibTool::make_grayscale(unsigned char* buf,
                                size_t buflen)
{
  memset(buf + buflen/2, 128, buflen - buflen/2);
}


/** Returns a string that describes what's done in next_step(). */
const char*
MirrorCalibTool::get_state_description() const
{
  switch (state_.step) {
    case SHARPENING:        return "Sharpen";
    case EDGE_DETECTION:    return "Edge detection";
    case COMBINATION:       return "Combining images";
    case CENTERING:         return "Finding center point";
    case PRE_MARKING:       return "First marking";
    case FINAL_MARKING:     return "Final marking";
    case DONE:              return "Direction done";
    default:                return "Invalid state";
  }
}


/** Finds the first marks. This is the first step in finding marks. */
MirrorCalibTool::MarkList
MirrorCalibTool::premark(const StepResult& prev,
                         const unsigned char* yuv_mask,
                         StepResult& result,
                         PolarAngle phi,
                         const PixelPoint& center)
{
  const ConvexPolygon empty_polygon;
  return premark(empty_polygon, prev, yuv_mask, result, phi, center);
}


/** Finds the first marks. This is the first step in finding marks. */
MirrorCalibTool::MarkList
MirrorCalibTool::premark(const ConvexPolygon& polygon,
                         const StepResult& prev,
                         const unsigned char* yuv_mask,
                         StepResult& result,
                         PolarAngle phi,
                         const PixelPoint& center)
{
  const CartesianImage prev_img(prev, phi, center, yuv_mask);
  CartesianImage res_img(result, phi, center, yuv_mask);
  int width = MIN_WIDTH_OF_BIGGEST_LINE;
  MarkList premarks;
  for (PolarRadius length = 0; length < prev_img.max_radius(); length++)
  {
    const CartesianPoint p(0.0, length);
    if (polygon.contains(prev_img, p) && prev_img.is_line(p, width)) {
      premarks.push_back(length);
      res_img.highlight_line(p, width);
    }
    if (length % 25 == 0) {
      width *= (1.0f - APPROX_LINE_WIDTH_LOSS);
    }
  }
  return premarks;
}


/** Searches for holes between the pre-marks. Helper for mark(). */
MirrorCalibTool::HoleList
MirrorCalibTool::search_holes(const MarkList& premarks)
{
  HoleList holes;
  PolarRadius prev_radius = -1;
  for (MarkList::const_iterator it = premarks.begin();
       it != premarks.end(); it++)
  {
    PolarRadius radius = *it;
    if (prev_radius != -1 && prev_radius + 1 < radius) {
      Hole hole(holes.size(), prev_radius, radius-1);
      holes.push_back(hole);
    }
    prev_radius = radius;
  }
  return holes;
}


/** Removes all but the n biggest holes (size depends on their position).
 * Helper for mark(). */
MirrorCalibTool::HoleList
MirrorCalibTool::filter_biggest_holes(const HoleList& holes,
                                      unsigned int n)
{
#ifdef FILTER_HOLES
  HoleList biggest = holes;
#ifdef FILTER_MINI_HOLES
restart: // XXX ugly :-)
  for (HoleList::iterator it = biggest.begin(); it != biggest.end(); it++)
  {
    if (it->size() <= 5) {
      biggest.erase(it);
      goto restart;
    }
  }
#endif

  HoleList filtered;
  for (unsigned int from = 0; from < biggest.size(); from++)
  {
    unsigned int to;
    for (to = from + 1; to < biggest.size(); to++) {
      if ((to - from + 1) > n) {
        to = from + n;
        break;
      }
      if (biggest[to - 1].size() < biggest[to].size()) {
        break;
      }
    }
    to--; // in all three break cases, to must be decremented
    if (to - from + 1 > filtered.size()) {
      filtered.clear();
      for (unsigned int j = from; j <= to; j++) {
        filtered.push_back(biggest[j]);
      }
    }
  }
  return filtered;
#else
  HoleList biggest;
  for (HoleList::const_iterator it = holes.begin(); it != holes.end(); ++it)
  {
    const Hole& hole = *it;
#ifdef FILTER_MINI_HOLES
    if (hole.size() < 5) {
      // very small holes are usually false-positives
      continue;
    }
#endif
    if (biggest.size() == 1 && hole.size() > biggest.front().size()) {
      // often the first determined hole is a part of the robot
      //biggest.erase(biggest.begin());
    }
    biggest.push_back(hole);
    if (biggest.size() == n) {
      break;
    }
  }
  return biggest;
#endif
}


/** Calculates the position of the marks between holes. Helper for mark(). */
MirrorCalibTool::MarkList
MirrorCalibTool::determine_marks(const HoleList& holes)
{
  HoleList biggest = filter_biggest_holes(holes, MARK_COUNT - 1);
  std::cout << "Filtered Holes: " << biggest.size() << std::endl;
  MarkList marks;
  for (HoleList::const_iterator prev, iter = biggest.begin();
       iter != biggest.end(); iter++) {
    const Hole& hole = *iter;
    if (iter == biggest.begin()) {
      const PolarRadius length = hole.from_length;
      marks.push_back(length);
    } else {
      const PolarRadius length = (hole.from_length + prev->to_length) / 2;
      marks.push_back(length);
    }
    if (iter+1 == biggest.end()) {
      const PolarRadius length = hole.to_length;
      marks.push_back(length);
    }
    prev = iter;
  }
  return marks;
}


/** Sets marks between all holes. The last step of marking. */
MirrorCalibTool::MarkList
MirrorCalibTool::mark(const MarkList& premarks,
                      const unsigned char* yuv_mask,
                      StepResult& result,
                      PolarAngle phi,
                      const PixelPoint& center)
{
  std::cout << "Premarked places: " << premarks.size() << std::endl;
  HoleList holes = search_holes(premarks);
  std::cout << "Found Holes: " << holes.size() << std::endl;
  MarkList marks = determine_marks(holes);
  std::cout << "Found Marks: " << marks.size() << std::endl;

  CartesianImage res_img(result, phi, center, yuv_mask);
  for (MarkList::const_iterator iter = marks.begin();
       iter != marks.end(); iter++)
  {
    const PolarAngle angle = 0.0;
    const PolarRadius radius = *iter;
    res_img.highlight_point(CartesianPoint(angle, radius));
    std::cout << "Highlighting Mark at " << *iter << std::endl;
  }
  return marks;
}


void
MirrorCalibTool::goto_next_state()
{
  const Image& src_img = source_images_[state_.image_index];
  switch (state_.step) {
    case SHARPENING:
      state_.step          = EDGE_DETECTION;
      break;
    case EDGE_DETECTION:
      if (src_img.results().size() <= ORIENTATION_COUNT) {
        state_.step        = EDGE_DETECTION;
      } else {
        state_.step        = COMBINATION;
      }
      break;
    case COMBINATION:
      if (src_img.results().size() < 2 * ORIENTATION_COUNT) {
        state_.step        = COMBINATION;
      } else if (state_.image_index + 1 < source_images_.size()) {
        state_.step = SHARPENING;
        state_.image_index++;
      } else {
        state_.step           = PRE_MARKING;
        state_.centering_done = false;
        state_.image_index    = 0;
      }
      break;
    case CENTERING:
      state_.step           = PRE_MARKING;
      state_.centering_done = true;
      state_.image_index    = 0;
      break;
    case PRE_MARKING:
      state_.step          = FINAL_MARKING;
      break;
    case FINAL_MARKING:
      if (state_.image_index + 1 < source_images_.size()) {
        state_.step        = PRE_MARKING;
        state_.image_index++;
      } else if (!state_.centering_done) {
        state_.step        = CENTERING;
        state_.image_index = 0;
      } else {
        state_.step        = DONE;
        state_.image_index = 0;
      }
      break;
    case DONE:
      state_.step        = DONE;
      state_.image_index = (state_.image_index + 1) % source_images_.size();
      break;
    default:
      throw fawkes::Exception("Invalid Calibration State");
  }
  printf("Next step: %s\n", get_state_description());
}


/** Performs one step in the calibration process.
 * In each step, a filter is applied to the image. The resulting image of
 * the n-th step is stored in the stepResults-vector on the (n-1)-th position.
 */
void
MirrorCalibTool::next_step()
{
  printf("Performing step for image %u / %u, %s\n",
      (unsigned int)state_.image_index + 1,
      (unsigned int)source_images_.size(),
      get_state_description());
  if (state_.image_index >= source_images_.size()) {
    return;
  }
  Image& src_img = source_images_[state_.image_index];
  StepResult result(src_img.buflen(), src_img.width(), src_img.height());
  switch (state_.step) {
    case SHARPENING:
      {
        apply_sharpen(src_img.yuv_buffer(), result.yuv_buffer(),
                      src_img.width(), src_img.height());
        make_grayscale(result.yuv_buffer(), result.buflen());
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      goto_next_state();
      return;
    case EDGE_DETECTION:
      {
        StepResult& prev = src_img.result(0);
        const int offset = (src_img.results().size() - 1) % ORIENTATION_COUNT;
        const orientation_t ori = static_cast<orientation_t>(ORI_DEG_0+offset);
        apply_sobel(prev.yuv_buffer(), result.yuv_buffer(),
                    prev.width(), prev.height(), ori);
        make_grayscale(result.yuv_buffer(), result.buflen());
        make_contrast(result.yuv_buffer(), result.buflen());
      }
      printf("Edge detection in %d\n", src_img.results().size());
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      assert(!memcmp(result.yuv_buffer(),
                     get_last_yuv_buffer(),
                     result.buflen()));
      goto_next_state();
      return;
    case COMBINATION:
      {
        const int index1 = src_img.results().size() - ORIENTATION_COUNT == 1
                         ? src_img.results().size() - ORIENTATION_COUNT
                         : src_img.results().size() - 1;
        const int index2 = src_img.results().size() - ORIENTATION_COUNT + 1;
        assert(index1 != index2);
        printf("ORing: %d = or(%d, %d)\n", src_img.results().size(),
            index1, index2);
        StepResult& prev1 = src_img.result(index1);
        StepResult& prev2 = src_img.result(index2);
        assert(&prev1 != &prev2);
        assert(prev1.width() == prev2.width());
        assert(prev1.height() == prev2.height());
        apply_or(prev1.yuv_buffer(), prev2.yuv_buffer(), result.yuv_buffer(),
                 prev1.width(), prev1.height());
        make_grayscale(result.yuv_buffer(), result.buflen());
        assert(memcmp(prev1.yuv_buffer(),
                      prev2.yuv_buffer(),
                      result.buflen()));
        assert(memcmp(result.yuv_buffer(),
                      prev1.yuv_buffer(),
                      result.buflen()));
        assert(memcmp(result.yuv_buffer(),
                      prev1.yuv_buffer(),
                      result.buflen()));
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      assert(!memcmp(result.yuv_buffer(),
                     get_last_yuv_buffer(),
                     result.buflen()));
      goto_next_state();
      return;
    case CENTERING:
      {
        const StepResult& orig = src_img.result(0);
        memcpy(result.yuv_buffer(), orig.yuv_buffer(), result.buflen());
        const PixelPoint center = calculate_center(source_images_);
        img_center_x_ = center.x;
        img_center_y_ = center.y;
        std::cout << "Found center (" << center.x << ", "<< center.y << ")"
                  << std::endl;
        CartesianImage img(result, robotRelativeOrientationToImageRotation(0.0),
                           center);
        img.highlight_point(CartesianPoint(0, 0));
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      goto_next_state();
      return;
    case PRE_MARKING:
      {
        const StepResult& prev = src_img.result(2 * ORIENTATION_COUNT - 1);
        memcpy(result.yuv_buffer(), prev.yuv_buffer(), result.buflen());
        const unsigned char* mask = img_yuv_mask_;
        const PixelPoint center(img_center_x_, img_center_y_);
        MarkList premarks = premark(prev, mask, result, src_img.ori(), center);
        src_img.set_premarks(premarks);
        apply_sharpen(src_img.yuv_buffer(), result.yuv_buffer(),
                      src_img.width(), src_img.height());
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      goto_next_state();
      return;
    case FINAL_MARKING:
      {
        const StepResult& orig = src_img.result(0);
        memcpy(result.yuv_buffer(), orig.yuv_buffer(), result.buflen());
        const unsigned char* mask = img_yuv_mask_;
        const PixelPoint center(img_center_x_, img_center_y_);
        const MarkList marks = mark(src_img.premarks(), mask, result,
                                    src_img.ori(), center);
        src_img.set_marks(marks);
        const PolarAngle ori = src_img.ori();
        std::cout << "Marking done for orientation "
                  << rad2deg(ori)
                  << " = "
                  << rad2deg(imageRotationToRobotRelativeOrientation(ori))
                  << std::endl;
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      goto_next_state();
      return;
    case DONE:
      {
        for (ImageList::iterator it = source_images_.begin();
             it != source_images_.end(); it++)
        {
          // cleanup: except the first result (whose yuv_buffer is needed)
          // no results are needed any more
          StepResultList results = it->results();
          results.erase(results.begin() + 1, results.end());

          const Image& src_img = *it;
          const PolarAngle ori = src_img.ori();
          MarkList marks = src_img.marks();
          mark_map_[imageRotationToRobotRelativeOrientation(ori)] = marks;
        }

        const StepResult& prev = src_img.result(0);
        const PixelPoint center(img_center_x_, img_center_y_);
        CartesianImage img(result, robotRelativeOrientationToImageRotation(0.0),
                           center);
        memcpy(result.yuv_buffer(), prev.yuv_buffer(), result.buflen());
        img.highlight_pixel(center);
        img.highlight_point(CartesianPoint(0, 0)); // should be same as center
        for (MarkMap::const_iterator map_iter = mark_map_.begin();
             map_iter != mark_map_.end(); map_iter++)
        {
          const PolarAngle phi = map_iter->first;
          const MarkList& list = map_iter->second;
          printf("%3.f: ", rad2deg(phi));
          for (MarkList::const_iterator list_iter = list.begin();
               list_iter != list.end(); list_iter++)
          {
            const PolarAngle angle = phi;
            const PolarRadius radius = *list_iter;
            img.highlight_point(CartesianPoint(angle, radius));
            PixelPoint pp = img.to_pixel(CartesianPoint(angle, radius));
            printf("%3d (%d, %d)", radius, pp.x, pp.y);
          }
          printf("\n");
        }
      }
      src_img.add_result(result);
      set_last_yuv_buffer(result.yuv_buffer());
      goto_next_state();
      return;
  }
}


/**
 * Returns the center point determined by the mean of the first marks found in
 * the different directions.
 * The angle of the returned cartesian point is relative to the robots
 * orientation (and thus must be rotated by 90 degrees counter-clock-wise
 * to be displayed correctly).
 * @return the estimated center point relative to the robots orientation
 */
MirrorCalibTool::PixelPoint
MirrorCalibTool::calculate_center(const ImageList& images)
{
  int x = 0;
  int y = 0;
  for (ImageList::const_iterator it = images.begin();
       it != images.end(); it++)
  {
    const Image& image = *it;
    const PolarRadius radius = image.marks().at(0);
    const unsigned char* null_buf = 0;
    const CartesianImage cart_image(null_buf, image.width(), image.height(),
                                    image.ori());
    const CartesianPoint point(0.0, radius);
    const PixelPoint pixel = cart_image.to_pixel(point);
    x += pixel.x;
    y += pixel.y;
  }
  return PixelPoint(x / images.size(), y / images.size());

  /*
  return PixelPoint(500, 500);
  int x = 0;
  int y = 0;
  int count = 0;
  for (MarkMap::const_iterator it = mark_map.begin();
       it != mark_map.end(); it++) {
    PolarAngle angle = it->first;
    PolarRadius dist = *(it->second.begin());
    CartesianPoint p(angle, dist);
    x += p.x;
    y += p.y;
    ++count;
  }
  CartesianPoint center(x/count, y/count);
  return center;
  */
}


/**
 * Searches the two angles in the MarkMap mark_map that are nearest to angle.
 * @param angle The reference angle to which the nearest marks are searched.
 * @param mark_map The mark map.
 * @return The two angles nearest to angle contained in mark_map (and therefore
 * keys to MarkLists in mark_map).
 */
MirrorCalibTool::PolarAnglePair
MirrorCalibTool::find_nearest_neighbors(PolarAngle angle,
                                        const MarkMap& mark_map)
{
  typedef vector<PolarAngle> AngleList;
  AngleList diffs;
  for (MarkMap::const_iterator it = mark_map.begin();
       it != mark_map.end(); it++)
  {
    const PolarAngle mark_angle = it->first;
    const PolarAngle diff = normalize_mirror_rad(mark_angle - angle);
    diffs.push_back(diff);
#if 0
    std::cout << "Finding nearest neighbors: "
              << "ref="<<angle<<"="<<rad2deg(angle)<<" "
              << "to="<<mark_angle<<"="<<rad2deg(mark_angle)<<" "
              << "diff="<<diff<<"="<<rad2deg(diff) << std::endl;
#endif
  }
  bool min1_init = false;
  AngleList::size_type min1_index = 0;
  bool min2_init = false;
  AngleList::size_type min2_index = 0;
  for (int i = 0; static_cast<AngleList::size_type>(i) < diffs.size(); i++) {
    if (!min1_init || abs(diffs[min1_index]) >= abs(diffs[i])) {
      min2_index = min1_index;
      min2_init  = min1_init;
      min1_index = i;
      min1_init  = true;
    } else if (!min2_init || abs(diffs[min2_index]) >= abs(diffs[i])) {
      min2_index = i;
      min2_init  = true;
    }
  }
  if (!min1_init) {
    throw fawkes::Exception("First minimum diff. angle not found");
  }
  if (!min2_init) {
    throw fawkes::Exception("Second minimum diff. angle not found");
  }
  PolarAngle min1;
  PolarAngle min2;
  AngleList::size_type i = 0;
  for (MarkMap::const_iterator it = mark_map.begin();
       it != mark_map.end(); it++)
  {
    if (i == min1_index) {
      min1 = it->first;
    } else if (i == min2_index) {
      min2 = it->first;
    }
    i++;
  }
#if 0
  std::cout << "Found nearest neighbor 1: "
            << "ref="<<angle<<"="<<rad2deg(angle)<<" "
            << "to="<<min1<<"="<<rad2deg(min1) << std::endl;
  std::cout << "Found nearest neighbor 2: "
            << "ref="<<angle<<"="<<rad2deg(angle)<<" "
            << "to="<<min2<<"="<<rad2deg(min2) << std::endl;
#endif
  return PolarAnglePair(min1, min2);
}


/**
 * Calculates the real distance to the n-th mark.
 * @param n The index of the mark, starting at 0.
 * @return The distance in centimeters.
 */
MirrorCalibTool::RealDistance
MirrorCalibTool::calculate_real_distance(int n)
{
  return MARK_DISTANCE + (n + 1) * MARK_DISTANCE;
}


MirrorCalibTool::RealDistance
MirrorCalibTool::interpolate(PolarRadius radius,
                             const MarkList& marks)
{
  if (marks.size() < 2) {
    throw fawkes::Exception("Not enough marks for interpolation");
  }
  for (MarkList::size_type i = 0; i < marks.size(); i++)
  {
    // linear interpolation, x0, x1 = 'Stuetzstellen', f0, f1 = 'Stuetzwerte'
    if (i == 0 && radius < marks[i]) {
      const PolarRadius  x0 = 0;
      const PolarRadius  x1 = marks[i];
      const PolarRadius  x  = radius;
      const RealDistance f0 = 0;
      const RealDistance f1 = calculate_real_distance(i);
#if 0
      std::cout << "A_Interpolate " << x << " between "
                << x0 << "->" << f0 << " "
                << x1 << "->" << f1 << std::endl;
#endif
      return static_cast<RealDistance>(static_cast<double>(f0) +
                                       static_cast<double>(x - x0) *
                                       static_cast<double>(f1 - f0) /
                                       static_cast<double>(x1 - x0));
    } else if (i > 0 && marks[i-1] <= radius &&
        (radius < marks[i] || i+1 == marks.size())) {
      const PolarRadius  x0 = marks[i-1];
      const PolarRadius  x1 = marks[i];
      const PolarRadius  x  = radius;
      const RealDistance f0 = calculate_real_distance(i-1);
      const RealDistance f1 = calculate_real_distance(i);
#if 0
      std::cout << "B_Interpolate " << x << " between "
                << x0 << "->" << f0 << " "
                << x1 << "->" << f1 << std::endl;
#endif
      return static_cast<RealDistance>(static_cast<double>(f0) +
                                       static_cast<double>(x - x0) *
                                       static_cast<double>(f1 - f0) /
                                       static_cast<double>(x1 - x0));
    }
  }
  throw fawkes::Exception("Interpolation Error");
}


Bulb
MirrorCalibTool::generate(int width,
                          int height,
                          const PixelPoint& center,
                          const MarkMap& mark_map)
{
  const unsigned char* null_img_buf = 0;
  CartesianImage img(null_img_buf, width, height,
                     robotRelativeOrientationToImageRotation(0.0), center);
  Bulb bulb(width, height);
  bulb.setCenter(center.x, center.y);
  bulb.setOrientation(0.0);
  for (int x = 0; x < width; x++)
  {
    for (int y = 0; y < height; y++)
    {
      const PixelPoint pp(x, y);
      const CartesianPoint cp = img.to_cartesian(pp);
      const PolarAngle ori_to_robot = cp.atan();
      const PolarAnglePair nearest_neighbors =
        find_nearest_neighbors(ori_to_robot, mark_map);
      const PolarAngle diff1 =
        abs(normalize_mirror_rad(nearest_neighbors.first - ori_to_robot));
      const PolarAngle diff2 =
        abs(normalize_mirror_rad(nearest_neighbors.second - ori_to_robot));
      const PolarAngle norm = diff1 + diff2;
      assert(norm != 0.0);
      const double weight1 = 1.0 - diff1 / norm;
      const double weight2 = 1.0 - diff2 / norm;
#if 0
      std::cout << "PixelPoint("<< x <<", "<< y <<")"<< std::endl;
      std::cout << "CartesianPoint("<< cp.x <<", "<< cp.y <<")"<< std::endl;
      std::cout << "Radius, Angle: " << cp.length() << " "
                << ori_to_robot << " = "
                << rad2deg(ori_to_robot) << std::endl;
      std::cout << "Neighbor 1: "
                << normalize_rad(nearest_neighbors.first) << " = "
                << rad2deg(normalize_rad(nearest_neighbors.first))
                << std::endl;
      std::cout << "Neighbor 2: "
                << normalize_rad(nearest_neighbors.second) << " = "
                << rad2deg(normalize_rad(nearest_neighbors.second))
                << std::endl;
      std::cout << "Diff 1: " << diff1 << " = " << rad2deg(diff1) << std::endl;
      std::cout << "Diff 2: " << diff2 << " = " << rad2deg(diff2) << std::endl;
      std::cout << "Norm Factor: " << norm << " = " << rad2deg(norm)
                << std::endl;
      std::cout << "Weight 1: " << weight1 << " = " << rad2deg(weight1)
                << std::endl;
      std::cout << "Weight 2: " << weight2 << " = " << rad2deg(weight2)
                << std::endl;
#endif
      assert(0.0 <= weight1 && weight1 <= 1.0);
      assert(0.0 <= weight2 && weight2 <= 1.0);
      assert(1.0 - 10e-5 < weight1 + weight2 && weight1 + weight2 < 1.0 + 10e-5);
      const MarkList& marks1 = mark_map.at(nearest_neighbors.first);
      const MarkList& marks2 = mark_map.at(nearest_neighbors.second);
      const RealDistance dist1 = interpolate(cp.length(), marks1);
      const RealDistance dist2 = interpolate(cp.length(), marks2);
#if 0
      std::cout << "Real 1 " << dist1 << std::endl;
      std::cout << "Real 2 " << dist2 << std::endl;
#endif
      const RealDistance weighted_mean_dist = dist1 * weight1 + dist2 * weight2;
      const float world_dist_in_meters      = weighted_mean_dist / 100.0f;
      const float world_phi_rel_to_robot    = ori_to_robot;
#if 0
      std::cout << "Dist 1: " << dist1 << std::endl;
      std::cout << "Dist 2: " << dist2 << std::endl;
      std::cout << "World Dist: " << world_dist_in_meters << std::endl;
      std::cout << "World Phi:  " << ori_to_robot << " = "
                << rad2deg(ori_to_robot) << std::endl;
#endif
      if (world_dist_in_meters > 0) {
        bulb.setWorldPoint(x, y, world_dist_in_meters, world_phi_rel_to_robot);
      }
    }
  }
  return bulb;
}


void
MirrorCalibTool::set_last_yuv_buffer(const unsigned char* last_buf)
{
  last_yuv_buffer_ = last_buf;
}


const unsigned char*
MirrorCalibTool::get_last_yuv_buffer() const
{
  return last_yuv_buffer_;
}


void
MirrorCalibTool::eval(unsigned int x, unsigned int y,
                      float* dist_ret, float* ori_ret)
{
#ifdef HAVE_BULB_CREATOR
  if (!bulb_) {
    printf("No bulb loaded, cannot evaluate.\n");
    return;
  }
  polar_coord_2d_t coord;
  coord = bulb_->getWorldPointRelative(x, y);

  *dist_ret = coord.r;
  *ori_ret = coord.phi;
#endif
}


/** Loads a calibration file.
 * @param filename Name of the file containing the calibration data.
 */
void
MirrorCalibTool::load(const char* filename)
{
#ifdef HAVE_BULB_CREATOR
  bulb_ = new Bulb(filename);
#endif
}


/** Saves calibration data to a file.
 * @param filename The name of the file.
 */
void
MirrorCalibTool::save(const char* filename)
{
#ifdef HAVE_BULB_CREATOR
  if (state_.step == DONE) {
    const Image& src_img = source_images_[state_.image_index];
    const PixelPoint center(img_center_x_, img_center_y_);
    Bulb bulb = generate(src_img.width(), src_img.height(), center, mark_map_);
    bulb.save(filename);
  } else {
    cout << "Can't save in the middle of the calibration" << endl;
  }
#endif
}


/** Draws a line from the image center in the given angle.
 * @param yuv_buffer The in/out parameter for the image.
 * @param angle_deg Angle of line in degrees.
 * @param center_x X-coordinate of center point in image pixels.
 * @param center_y Y-coordinate of center point in image pixels.
 * @param width Width of image.
 * @param height Height of image.
 */
void
MirrorCalibTool::draw_line(unsigned char* yuv_buffer,
                           double angle_deg,
                           int center_x,
                           int center_y,
                           int width,
                           int height)
{
  const PolarAngle angle = normalize_rad(deg2rad(angle_deg));
  CartesianImage img(yuv_buffer, width, height,
                     robotRelativeOrientationToImageRotation(0.0),
                     PixelPoint(center_x, center_y));
  for (PolarRadius length = 0; length < img.max_radius(); length++)
  {
    const CartesianPoint p(angle, length);
    if (img.contains(p)) {
      img.set_color(p, MARK_LUMA, MARK_CHROMINANCE);
    }
  }
}


/** Draws a crosshair with the lines in the directions of the keys of 
 * the mark map.
 * @param yuv_buffer The in/out parameter for the image.
 */
void
MirrorCalibTool::draw_mark_lines(unsigned char* yuv_buffer)
{
  for (ImageList::const_iterator it = source_images_.begin();
       it != source_images_.end(); it++)
  {
    const Image& src_img = *it;
    CartesianImage img(yuv_buffer, src_img.width(), src_img.height(),
                       src_img.ori(), PixelPoint(img_center_x_, img_center_y_));
    for (PolarRadius length = 0; length < img.max_radius(); length++)
    {
      const PolarAngle angle = 0.0;
      const CartesianPoint p(angle, length);
      if (img.contains(p)) {
        img.set_color(p, MARK_LUMA, MARK_CHROMINANCE);
      }
    }
  }
}


/** Draws a crosshair in the YUV-buffer. The crosshair consists of three lines
 * from the origin (0, 0) with the angles 0 degree, 120 degrees, 240 degrees.
 * @param yuv_buffer The in/out parameter for the image.
 * @param center_x X-coordinate of center point in image pixels.
 * @param center_y Y-coordinate of center point in image pixels.
 * @param width Width of image.
 * @param height Height of image.
 */
void
MirrorCalibTool::draw_crosshair(unsigned char* yuv_buffer,
                                int center_x,
                                int center_y,
                                int width,
                                int height)
{
  const PolarAngle POSITIONS[] = { normalize_rad(deg2rad(   0.0f)),
                                   normalize_rad(deg2rad(-120.0f)),
                                   normalize_rad(deg2rad( 120.0f)) };
  const int POSITION_COUNT = sizeof POSITIONS / sizeof(double);
  CartesianImage img(yuv_buffer, width, height,
                     robotRelativeOrientationToImageRotation(0.0),
                     PixelPoint(center_x, center_y));
  for (int i = 0; i < POSITION_COUNT; i++)
  {
    const PolarAngle angle = POSITIONS[i];
    for (PolarRadius length = 0; length < img.max_radius(); length++)
    {
      const CartesianPoint p(angle, length);
      if (img.contains(p)) {
        img.set_color(p, MARK_LUMA, MARK_CHROMINANCE);
      }
    }
  }
}

