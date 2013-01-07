
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

#include <fvfilters/sobel.h>
#include <fvfilters/sharpen.h>
#include <fvfilters/median.h>
#include <fvfilters/or.h>
#include <fvfilters/laplace.h>
#include <fvfilters/min.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cassert>

/* If defined, the center point is re-calculated by averaging the first
 * marks. */
//#define RECALCULATE_CENTER_POINT

#define FILTER_HOLES

/* If defined, very small holes are ignored. These small holes occur because a
 * single mark is sometimes recognized as *two* different marks due to the edge
 * detection. */
#define FILTER_MINI_HOLES

using namespace fawkes;

namespace firevision {

/** @class MirrorCalibTool mirror_calib.h
 * This class encapsulates the routines necessary for interactive mirror
 * calibration.
 *
 * The input is N pairs (degree,image) and (optionally) a filter mask.
 * The calibration runs in multiple phases:
 * (1) Edge detection.
 * (2) Assume center point in the middle of the image unless it is set
 *     explicitly.
 * (3) Pre-marking: Find the edges that lie at `degree' wrt. Y axis in image.
 * (4) Final marking: Try to filter false-positive marks.
 * (5) Centering: Average the first marks in each direction to get the `real'
 *     center point of the mirror.
 * (6) Again do steps (3) and (5) with the new center point.
 * (7) Generate bulb files. The polar coordinates of each pixel are determined
 *     as follows: 
 *    (a) Determine the polar coordinates of the pixel.
 *    (b) Get the two mark streams that enclose the pixel.
 *    (c) Do linear interpolation in both mark streams to get two distances.
 *    (d) Add the weightened distances (the nearer the pixel is to the mark
 *        stream the higher is its weight).
 *    (e) Now we have an estimated distance. We need to multiply the angle
 *        with -1.0 (!!!) to consider the fact that the image is mirrored:
 *        what looks to left from the robot on the image, is on the right of
 *        the robot in reality!
 *        Note that all the time we worked on the mirrored image with angles
 *        that were correct from the perspective of the image. But to make them
 *        correct from the perspective of the robot, we have to mirror
 *        everything.
 */

namespace {
  const unsigned int  ORIENTATION_COUNT                 = (ORI_DEG_360 - ORI_DEG_0);
  const unsigned int  MARK_COUNT                        = 6;
  const float         MARK_DISTANCE                     = 29.7;
  const float         CARPET_DISTANCE_FROM_ROBOT_CENTER = 5.0;
  const unsigned char DARK                              = 0;
  const unsigned char BRIGHT                            = 255;
  const unsigned char MARK_LUMA                         = 128;
  const unsigned char MARK_CHROMINANCE                  = 255;
  const int           MIN_WIDTH_OF_BIGGEST_LINE         = 50;
  const float         APPROX_LINE_WIDTH_LOSS            = 0.20f;
  const float         MIN_BRIGHT_FRACTION               = 0.20f;
  const int           HIGHLIGHT_RADIUS                  = 2;
  const int           FALSE_POSITIVE_HOLE_SIZE          = 5;
}


/**
 * The result of a step contains a YUV buffer.
 * The different steps are descripted in the MirrorCalibTool's doc.
 */
class MirrorCalibTool::StepResult {
 private:
  unsigned char* buffer_;
  size_t buflen_;
  int width_;
  int height_;
  unsigned int* refcount_;

 public:
  /** Constructor.
   * @param buflen
   * @param width
   * @param height */
  StepResult(size_t buflen, int width, int height)
  : buffer_(new unsigned char[buflen]),
    buflen_(buflen),
    width_(width),
    height_(height),
    refcount_(new unsigned int)
  {
    *refcount_ = 1;
  }

  /** Constructor.
   * @param copy */
  StepResult(const StepResult& copy)
  : buffer_(copy.buffer_),
    buflen_(copy.buflen_),
    width_(copy.width_),
    height_(copy.height_),
    refcount_(copy.refcount_)
  {
    ++*refcount_;
  }

  /** Assignment.
   * @param copy result to assign
   * @return reference to this instance */
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

  /** Destructor. */
  ~StepResult()
  {
    if (--*refcount_ == 0) {
      delete[] buffer_;
      delete refcount_;
    }
  }

  /** YUV buffer.
   * @return YUV buffer */
  inline unsigned char* yuv_buffer() { return buffer_; }

  /** YUV buffer.
   * @return YUV buffer */
  inline const unsigned char* yuv_buffer() const { return buffer_; }

  /** YUV buffer's length.
   * @return buffer length */
  inline size_t buflen() const { return buflen_; }

  /** Width of YUV buffer.
   * @return height */
  inline int width() const { return width_; }

  /** Height of YUV buffer.
   * @return height */
  inline int height() const { return height_; }
}; // StepResult


/**
 * Abstract Point class.
 */
class MirrorCalibTool::Point {
 public:
  const int x; /**< X coordinate. */
  const int y; /**< Y coordinate. */

  /** Constructor.
   * @param x
   * @param y */
  Point(int x, int y)
  : x(x),
    y(y)
  {
  }

  /** Length of the vector the point.
   * @return length of the vector the point.
   * @see atan() for polar coordinates. */
  PolarRadius length() const
  {
    return static_cast<PolarRadius>(sqrt(x*x + y*y));
  }

  /** Atan(y, x) of the point.
   * @return Atan(y, x) of the point.
   * @see length() for polar coordinates. */
  inline PolarAngle atan() const
  {
    return normalize_rad(atan2(y, x));
  }

  /** Assignment.
   * @return A copy.
   * @param p */
  Point operator=(const Point& p)
  {
    if (&p == this) {
      return *this;
    }
    return Point(p.x, p.y);
  }
}; // Point


/**
 * A cartesian point is a 2d point which can have negative X and Y coords.
 */
class MirrorCalibTool::CartesianPoint
: public Point
{
 public:
  /** Constructor.
   * @param phi
   * @param length */
  CartesianPoint(PolarAngle phi, PolarRadius length)
  : Point(length * cos(phi),
          length * sin(phi))
  {
  }

  /** Constructor.
   * @param x
   * @param y */
  CartesianPoint(int x, int y)
  : Point(x, y)
  {
  }

  /** Rotates the vector to the point counter-clockwise and returns the vector
   * to the point.
   * @param rotate_phi
   * @return Counter-clockwise rotated point. */
  CartesianPoint rotate(PolarAngle rotate_phi) const
  {
    const PolarRadius len = length();
    const PolarAngle phi = atan() + rotate_phi;
    const int x = len * cos(phi);
    const int y = len * sin(phi);
    return CartesianPoint(x, y);
  }
}; // CartesianPoint


/**
 * A pixel point is a 2d point with positive X and Y coords.
 */
class MirrorCalibTool::PixelPoint
: public Point
{
 public:
  /** Constructor.
   * @param x
   * @param y */
  PixelPoint(int x, int y)
  : Point(x, y)
  {
  }

  /** Rotates the vector to the point counter-clockwise and returns the vector
   * to the point.
   * @param rotate_phi Angle.
   * @return Counter-clockwise rotated point. */
  PixelPoint rotate(PolarAngle rotate_phi) const
  {
    const PolarRadius len = length();
    const PolarAngle phi = atan() + rotate_phi;
    const int x = len * cos(phi);
    const int y = len * sin(phi);
    return PixelPoint(x, y);
  }
}; // PixelPoint


/**
 * Wraps an image so that access to (0, 0) is mapped to the middle of the
 * image and so on. The result is a cartesian coordinate system with X and
 * Y axis.
 */
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
  /** Constructor.
   * @param res The step result that contains the base image.
   * @param phi The angle by which the image is rotated counter-clockwise.
   * @param center The center of the image from the robot's point of view; i.e.
   *               the center of the cone mirror.
   * @param mask The mask that indicates which pixels should be ignored. */
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

  /** Constructor.
   * @param res The step result that contains the base image.
   * @param phi The angle by which the image is rotated counter-clockwise.
   * @param mask The mask that indicates which pixels should be ignored. */
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

  /** Constructor.
   * @param buf The base YUV buffer.
   * @param width Image width.
   * @param height Image height.
   * @param phi The angle by which the image is rotated counter-clockwise.
   * @param center The center of the image from the robot's point of view; i.e.
   *               the center of the cone mirror. */
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

  /** Constructor.
   * @param buf The base YUV buffer.
   * @param width Image width.
   * @param height Image height.
   * @param phi The angle by which the image is rotated counter-clockwise.
   * @param center The center of the image from the robot's point of view; i.e.
   *               the center of the cone mirror. */
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

  /** Constructor.
   * @param buf The base YUV buffer.
   * @param width Image width.
   * @param height Image height.
   * @param phi The angle by which the image is rotated counter-clockwise. */
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

  /** Constructor.
   * @param buf The base YUV buffer.
   * @param width Image width.
   * @param height Image height.
   * @param phi The angle by which the image is rotated counter-clockwise. */
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

  /** Get Buffer.
   * @return buffer
   */
  inline unsigned char* buf() { return buf_; }
  /** Mask.
   * @return mask
   */
  inline const unsigned char* mask() const { return mask_; }
  /** Buffer.
   * @return const buffer
   */
  inline const unsigned char* buf() const { return buf_; }
  /** Center of buffer.
   * @return center point of buffer
   */
  inline const PixelPoint& center() const { return center_; }
  /** Width of buffer.
   * @return width of buffer
   */
  inline const int width() const { return width_; }
  /** Height of buffer.
   * @return height of buffer
   */
  inline const int height() const { return height_; }
  /** Phi angle.
   * @return Phi angle
   */
  inline const PolarAngle phi() const { return phi_; }

  /** Converts a cartesian point to a pixel point.
   * @param p The point.
   * @return converted point
   */
  inline PixelPoint
  to_pixel(const CartesianPoint& p) const
  {
    const CartesianPoint rp = p.rotate(-1.0 * phi());
    return PixelPoint(center().x + rp.x, center().y - rp.y);
  }

  /** Converts a pixel point to a cartesian point.
   * @param p The point.
   * @return converted point
   */
  inline CartesianPoint
  to_cartesian(const PixelPoint& p) const
  {
    const CartesianPoint cp(p.x - center().x, center().y - p.y);
    return cp.rotate(phi());
  }

  /** Indicates whether the image contains a pixel point.
   * @param p The point.
   * @return true if pixel point is in image, false otherwise
   */
  inline bool
  contains(const PixelPoint& p) const
  {
    return 0 <= p.x && p.x <= width()-1 && 0 <= p.y && p.y <= height()-1;
  }

  /** Indicates whether the image contains a cartesian point.
   * @param p The point.
   * @return true if cartesian point is in image, false otherwise
   */
  inline bool
  contains(const CartesianPoint& p) const
  {
    return contains(to_pixel(p));
  }

  /** Get luminance at a given point.
   * @param p The point.
   * @return luminance at given point
   */
  inline unsigned char
  get(const CartesianPoint& p) const
  {
    if (!contains(p)) {
      throw fawkes::Exception("Point p is out of image");
    }
    PixelPoint pp = to_pixel(p);
    const YUV_t ignr(0);
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

  /** Get Maximum cartesian X coordinate of the image.
   * @return Maximum cartesian X coordinate of the image.
   */
  inline int max_x() const { return std::max(center().x, width() - center().x); }
  /** Get Maximum cartesian Y coordinate of the image.
   * @return Maximum cartesian Y coordinate of the image.
   */
  inline int max_y() const { return std::max(center().y, height() - center().y); }
  /** Maximum polar radius of the image.
   * @return Maximum polar radius of the image.
   */
  inline PolarRadius max_radius() const {
    return static_cast<PolarRadius>(sqrt(max_x()*max_x() + max_y()*max_y()));
  }

  /** Sets the luminance Y and chrominance U at a given pixel point.
   * @param p The point whose luminance Y and chrominance U is changed.
   * @param luma The luminance Y.
   * @param chrominance The chrominance U. */
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

  /** Sets the luminance Y and chrominance U at a given cartesian point.
   * @param p The point whose luminance Y and chrominance U is changed.
   * @param luma The luminance Y.
   * @param chrominance The chrominance U. */
  void
  set_color(const CartesianPoint& p,
            unsigned char luma,
            unsigned char chrominance)
  {
    set_color(to_pixel(p), luma, chrominance);
  }

  /** Get relative amount of bright pixels.
   * @param from One point of the rectangle.
   * @param to Opposite point of the rectangle.
   * @return relative amount of BRIGHT pixels in the rectangle denoted
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

  /** Indicates whether at pixel point p there is a highlighted line.
   * @param p The assumed center point of the line.
   * @param length The length of the line
   * @return true if pixel belongs to line, false otherwise
   */
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

  /** Highlightes a line at pixel point p.
   * @param p The center point of the line.
   * @param length The length of the line. */
  void
  highlight_line(const CartesianPoint& p,
                 int length)
  {
    const CartesianPoint vec(p.atan() + deg2rad(90.0), length / 2);
    const CartesianPoint from(p.x - vec.x, p.y - vec.y);
    const CartesianPoint to(p.x + vec.x, p.y + vec.y);
    draw_line(from, to);
  }

  /** Draws a line from p to q.
   * @param p First point of the line.
   * @param q First point of the line. */
  void
  draw_line(const PixelPoint& p,
            const PixelPoint& q)
  {
    draw_line(to_cartesian(p), to_cartesian(q));
  }

  /** Draws a line from p to q.
   * @param p First point of the line.
   * @param q First point of the line. */
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

  /** Highlights a pixel, i.e. sets the luminance and chrominance to
   * MARK_LUMA and MARK_CHROMINANCE.
   * @param p The pixel. */
  void
  highlight_pixel(const PixelPoint& p)
  {
    const int R = HIGHLIGHT_RADIUS;
    for (int xx = p.x-R; xx <= p.x+R; xx++) {
      for (int yy = p.y-R; yy <= p.y+R; yy++) {
        const PixelPoint pp(xx, yy);
        if (contains(pp)) {
          set_color(pp, MARK_LUMA, MARK_CHROMINANCE);
        }
      }
    }
  }

  /** Highlights a point, i.e. sets the luminance and chrominance to
   * MARK_LUMA and MARK_CHROMINANCE.
   * @param p The point. */
  void
  highlight_point(const CartesianPoint& p)
  {
    const int R = HIGHLIGHT_RADIUS;
    for (int xx = p.x-R; xx <= p.x+R; xx++) {
      for (int yy = p.y-R; yy <= p.y+R; yy++) {
        const CartesianPoint hp(xx, yy);
        if (contains(hp)) {
          set_color(hp, MARK_LUMA, MARK_CHROMINANCE);
        }
      }
    }
  }
}; // CartesianImage


/** @class MirrorCalibTool::ConvexPolygon libs/fvmodels/mirror/mirror_calib.h
 * Represents a convex polygon. It is defined by a sequence of points in
 * clock-wise-order.
 */

/** Check if point is inside convex polygon.
 * The point r is converted to PixelPoint wrt img.
 * @param img image in which to check
 * @param r cartesian point to check
 * @return true if the point is inside the convex polygon
 */
bool
MirrorCalibTool::ConvexPolygon::contains(const CartesianImage& img,
                                         const CartesianPoint& r) const
{
  return contains(img.to_pixel(r));
}


/** Check if pixel point is inside convex polygon.
 * This is the case if for all points p, q in the polygon p_1, ..., p_n
 * where p = p_i, q = p_{i+1} for some i or p = p_n, q = p_1 it holds
 * (p, q, r) does not form a left turn (if they do, they are
 * counter-clock-wise).
 * @param r point to check
 * @return true if the point is inside the convex polygon
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
  int index; /**< The index of the hole. */
  PolarRadius from_length; /**< Beginning of the hole. */
  PolarRadius to_length; /**< Ending of the hole. */

  /** Constructor.
   * @param index The index of the hole.
   * @param from_length The beginning of the hole in pixels.
   * @param to_length The ending of the hole in pixels. */
  Hole(int index,
       PolarRadius from_length,
       PolarRadius to_length)
  : index(index),
    from_length(from_length),
    to_length(to_length)
  {
  }

  /** The size of the hole in pixels.
   * @return radius of hole
   */
  inline PolarRadius size() const { return to_length - from_length; }
}; // Hole


/** A container for a YUV-buffer etc. */
class MirrorCalibTool::Image
{
 public:
  /** Constructor.
   * @param yuv_buffer The YUV buffer.
   * @param buflen The buffer's size.
   * @param width The width.
   * @param height The height.
   * @param ori The orientation. */
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

  /** Constructor.
   * @param copy */
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

  /** Assignment.
   * @param copy image to copy
   * @return this image
   */
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

  /** Destructor. */
  ~Image()
  {
    if (--*refcount_ == 0) {
      delete[] yuv_buffer_;
      delete refcount_;
    }
  }

  /** YUV buffer.
   * @return YUV buffer
   */
  inline unsigned char* yuv_buffer() { return yuv_buffer_; }
  /** YUV buffer.
   * @return 
   */
  inline const unsigned char* yuv_buffer() const { return yuv_buffer_; }
  /** YUV buffer's length.
   * @return YUV buffer's length
   */
  inline size_t buflen() const { return buflen_; }
  /** YUV buffer's width.
   * @return YUV buffer's width
   */
  inline int width() const { return width_; }
  /** YUV buffer's height.
   * @return YUV buffer's height
   */
  inline int height() const { return height_; }
  /** Angle of marks wrt X axis.
   * @return Angle of marks wrt X axis
   */
  inline PolarAngle ori() const { return ori_; }
  /** List of results.
   * @return List of results
   */
  inline StepResultList& results() { return results_; }
  /** List of results.
   * @return List of results
   */
  inline const StepResultList& results() const { return results_; }
  /** The premarks.
   * @return The premarks
   */
  inline const MarkList& premarks() { return premarks_; }
  /** The (final) marks.
   * @return 
   */
  inline MarkList& marks() { return marks_; }
  /** The (final) marks.
   * @return The (final) marks
   */
  inline const MarkList& marks() const { return marks_; }

  /** Appends a result.
   * @param r The new result. */
  inline void add_result(const StepResult& r) { results_.push_back(r); }
  /** Returns the i-th result.
   * @param i The index of the result starting with 0.
   * @return result
   */
  inline StepResult& result(int i) { return results_[i]; }
  /** Returns the i-th result.
   * @param i The index of the result starting with 0.
   * @return result
   */
  inline const StepResult& result(int i) const { return results_[i]; }
  /** The premarks.
   * @param premarks The list of premarks. */
  inline void set_premarks(const MarkList& premarks) { premarks_ = premarks; }
  /** The (final) marks.
   * @param marks The list of marks. */
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
    img_center_x_(-1),
    img_center_y_(-1),
    img_yuv_mask_(0),
    state_(CalibrationState())
  , bulb_(0)
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
  if (bulb_) {
    delete bulb_;
  }
}


/**
 * Converts an angle wrt. the Y axis (!) to the robots view to the needed image
 * rotation so that the things, which lie at angle `ori' wrt. the Y axis, lie
 * on the X axis of the rotated image.
 * For example: if the marks are 120 degrees counter-clock-wise from the robot,
 * the image needs to be rotated 120 degrees clock-wise (then the marks are 
 * in front of the robot, i.e. Y axis) and then 90 degrees clock-wise
 * (to get the marks from the Y axis to the X axis).
 *
 * The reason that we want to have the marks on the X axis is that calculating
 * with them is then a bit easier (because then their polar angle is always
 * 0.0).
 * @param ori The orientation.
 * @return angle
 */
MirrorCalibTool::PolarAngle
MirrorCalibTool::relativeOrientationToImageRotation(PolarAngle ori)
{
  return normalize_rad(-1.0 * ori + deg2rad(-90.0));
}


/**
 * Converts the rotation of the image to the orientation relative to the Y axis.
 * See the documentation of relativeOrientationToImageRotation() of which this
 * is the inverse.
 * @param ori The orientation.
 * @return angle
 */
MirrorCalibTool::PolarAngle
MirrorCalibTool::imageRotationToRelativeOrientation(PolarAngle ori)
{
  return normalize_rad(-1.0 * (ori + deg2rad(90.0)));
}


/**
 * Loads a PNM mask file for the robot's bars.
 * The mask is used to ignore those pixels that are part of the robot's bar.
 * @param mask_file_name The mask file name.
 */
void
MirrorCalibTool::load_mask(const char* mask_file_name)
{
  if (img_yuv_mask_) {
    delete[] img_yuv_mask_;
  }
  PNMReader reader(mask_file_name);
  size_t size = colorspace_buffer_size(reader.colorspace(),
                                       reader.pixel_width(),
                                       reader.pixel_height());
  img_yuv_mask_ = new unsigned char[size];
  if (img_center_x_ == -1) {
    img_center_x_ = reader.pixel_width() / 2;
  }
  if (img_center_y_ == -1) {
    img_center_y_ = reader.pixel_height() / 2;
  }
  reader.set_buffer(img_yuv_mask_);
  reader.read();
}


/** Store image for calibration process.
 * @param yuv_buffer The image's yuv_buffer. It may be freeed by the caller
 *                   immediately, the MirrorCalibTool works with a copy of it.
 * @param buflen     The length of yuv_buffer.
 * @param width      The width of the image.
 * @param height     The height of the image.
 * @param ori        The polar angle in degrees (!) relative to the Y axis of
 *                   the image (!) where the marks are in the image (!) (not in
 *                   in reality from the robot's perspective).
 */
void
MirrorCalibTool::push_back(const unsigned char* yuv_buffer,
                           size_t buflen,
                           int width,
                           int height,
                           double ori)
{
  ori = relativeOrientationToImageRotation(ori);
  Image src_img(yuv_buffer, buflen, width, height, ori);
   if (img_center_x_ == -1) {
    img_center_x_ = width / 2;
  }
  if (img_center_y_ == -1) {
    img_center_y_ = height / 2;
  }
  source_images_.push_back(src_img);
}


/** Aborts the calibration process. */
void
MirrorCalibTool::abort()
{
  img_center_x_       = -1;
  img_center_y_       = -1;
  state_              = CalibrationState();
  source_images_.clear();
  premarks_.clear();
  mark_map_.clear();
}


/** Applies a sobel filter for edge detection in some direction.
 * @param src The source YUV buffer.
 * @param dst The destination YUV buffer.
 * @param width The width.
 * @param height The height.
 * @param ori The orientation. */
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


/** Applies a sharpening filter.
 * @param src The source YUV buffer.
 * @param dst The destination YUV buffer.
 * @param width The width.
 * @param height The height. */
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


/** Applies a median filter.
 * @param src The source YUV buffer.
 * @param dst The destination YUV buffer.
 * @param width The width.
 * @param height The height.
 * @param i The median parameter. */
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


/** Applies a minimum filter.
 * @param src The source YUV buffer.
 * @param dst The destination YUV buffer.
 * @param width The width.
 * @param height The height. */
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


/** Applies a disjunction filter.
 * @param src1 The first source YUV buffer.
 * @param src2 The second source YUV buffer.
 * @param dst The destination YUV buffer.
 * @param width The width.
 * @param height The height. */
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


/** Sets all pixels to black or white (i.e. maximum contrast).
 * @param buf The YUV buffer.
 * @param buflen Buffer's length. */
void
MirrorCalibTool::make_contrast(unsigned char* buf,
                               size_t buflen)
{
  for (unsigned int i = 0; i < buflen/2; i++) {
    buf[i] = (buf[i] >= 75) ? BRIGHT : DARK;
  }
}


/** Sets all chrominance values to 128.
 * @param buf The YUV buffer.
 * @param buflen Buffer's length. */
void
MirrorCalibTool::make_grayscale(unsigned char* buf,
                                size_t buflen)
{
  memset(buf + buflen/2, 128, buflen - buflen/2);
}


/** Get description of next step.
 * @return string that describes what's done in next_step().
 */
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


/** Finds the first marks. This is the first step in finding marks.
 * @return mark list
 */
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


/** Finds the first marks. This is the first step in finding marks.
 * @return mark list
 */
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


/** Searches for holes between the pre-marks. Helper for mark().
 * @return holes
 */
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
    if (it->size() <= FALSE_POSITIVE_HOLE_SIZE) {
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
    if (hole.size() < FALSE_POSITIVE_HOLE_SIZE) {
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
#ifdef RECALCULATE_CENTER_POINT
      } else if (!state_.centering_done) {
        state_.step        = CENTERING;
        state_.image_index = 0;
#endif
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
      printf("Edge detection in %u\n", (unsigned) src_img.results().size());
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
        printf("ORing: %d = or(%d, %d)\n", (unsigned) src_img.results().size(),
            index1, index2);
        StepResult& prev1 = src_img.result(index1);
        StepResult& prev2 = src_img.result(index2);
        assert(&prev1 != &prev2);
        assert(prev1.width() == prev2.width());
        assert(prev1.height() == prev2.height());
        apply_or(prev1.yuv_buffer(), prev2.yuv_buffer(), result.yuv_buffer(),
                 prev1.width(), prev1.height());
        make_grayscale(result.yuv_buffer(), result.buflen());
        /*assert(memcmp(prev1.yuv_buffer(),
                      prev2.yuv_buffer(),
                      result.buflen()));
        assert(memcmp(result.yuv_buffer(),
                      prev1.yuv_buffer(),
                      result.buflen()));
        assert(memcmp(result.yuv_buffer(),
                      prev1.yuv_buffer(),
                      result.buflen()));*/
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
        CartesianImage img(result, relativeOrientationToImageRotation(0.0),
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
                  << rad2deg(imageRotationToRelativeOrientation(ori))
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
          mark_map_[imageRotationToRelativeOrientation(ori)] = marks;
        }

        const StepResult& prev = src_img.result(0);
        const PixelPoint center(img_center_x_, img_center_y_);
        CartesianImage img(result, relativeOrientationToImageRotation(0.0),
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
  typedef std::vector<PolarAngle> AngleList;
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
    if (!min1_init || fabs(diffs[min1_index]) >= fabs(diffs[i])) {
      min2_index = min1_index;
      min2_init  = min1_init;
      min1_index = i;
      min1_init  = true;
    } else if (!min2_init || fabs(diffs[min2_index]) >= fabs(diffs[i])) {
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
  PolarAngle min1 = 0.0;
  PolarAngle min2 = 0.0;
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
            << "to="<<min1<<"="<<rad2deg(min1) <<" "
            << "index="<<min1_index << std::endl;
  std::cout << "Found nearest neighbor 2: "
            << "ref="<<angle<<"="<<rad2deg(angle)<<" "
            << "to="<<min2<<"="<<rad2deg(min2) <<" "
            << "index="<<min2_index << std::endl;
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
  return static_cast<int>(CARPET_DISTANCE_FROM_ROBOT_CENTER +
                          static_cast<float>(n + 1) * MARK_DISTANCE);
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


/**
 * Generates a new bulb based on the given data.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param center The center pixel point of the image.
 * @param mark_map The map of marks where the key is the orientation of the
 * marks and the value is a sequence of distances to the marks.
 */
Bulb
MirrorCalibTool::generate(int width,
                          int height,
                          const PixelPoint& center,
                          const MarkMap& mark_map)
{
  const unsigned char* null_img_buf = 0;
  CartesianImage img(null_img_buf, width, height,
                     relativeOrientationToImageRotation(0.0), center);
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
        fabs(normalize_mirror_rad(nearest_neighbors.first - ori_to_robot));
      const PolarAngle diff2 =
        fabs(normalize_mirror_rad(nearest_neighbors.second - ori_to_robot));
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
      const float world_phi_rel_to_robot    = -1.0f * ori_to_robot;
      // world_phi_rel_to_robot must be multiplied with -1 because the image is
      // mirrored: what's on the right in the image, is on the left of the robot
      // in reality
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


/** Get last created YUV buffer.
 * This is the result of the most recent step. Memory management is
 * done by MirrorCalibTool.
 * @return most recent result's YUV buffer
 */
const unsigned char*
MirrorCalibTool::get_last_yuv_buffer() const
{
  return last_yuv_buffer_;
}


/** Get the assumed distance and orientation of a pixel point.
 * @param x The X coordinate of the pixel that is to be evaluated.
 * @param y The Y coordinate of the pixel that is to be evaluated.
 * @param dist_ret The distance in the real world to the pixel.
 * @param ori_ret The orientation in the real world to the pixel.
 */
void
MirrorCalibTool::eval(unsigned int x, unsigned int y,
                      float* dist_ret, float* ori_ret)
{
  if (!bulb_) {
    printf("No bulb loaded, cannot evaluate.\n");
    return;
  }
  polar_coord_2d_t coord;
  coord = bulb_->getWorldPointRelative(x, y);

  *dist_ret = coord.r;
  *ori_ret = coord.phi;
}


/** Loads a calibration file.
 * @param filename Name of the file containing the calibration data.
 */
void
MirrorCalibTool::load(const char* filename)
{
  bulb_ = new Bulb(filename);
}


/** Saves calibration data to a file.
 * @param filename The name of the file.
 */
void
MirrorCalibTool::save(const char* filename)
{
  if (state_.step == DONE) {
    const Image& src_img = source_images_[state_.image_index];
    const PixelPoint center(img_center_x_, img_center_y_);
    Bulb bulb = generate(src_img.width(), src_img.height(), center, mark_map_);
    bulb.save(filename);
  } else {
    std::cout << "Can't save in the middle of the calibration" << std::endl;
  }
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
                     relativeOrientationToImageRotation(0.0),
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
                     relativeOrientationToImageRotation(0.0),
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

}

