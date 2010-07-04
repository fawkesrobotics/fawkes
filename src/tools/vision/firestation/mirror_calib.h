
/***************************************************************************
 *  mirror_calib.h - Mirror calibration tool
 *
 *  Created: Fri Dec 07 18:34:50 2007
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

#ifndef __FIREVISION_TOOLS_FIRESTATION_MIRROR_CALIB_H_
#define __FIREVISION_TOOLS_FIRESTATION_MIRROR_CALIB_H_

#include <geometry/hom_point.h>
#include <utils/math/angle.h>

#ifdef HAVE_BULB_CREATOR
#include <models/mirror/bulb.h>
#include <bulb_calib/bulb_generator.h>
#endif

#include <iostream>
#include <vector>
#include <map>
#include <cassert>

class MirrorCalibTool
#ifdef HAVE_BULB_CREATOR
 : public firevision::BulbGeneratorProgressHandler
#endif
{
 public:
  static void draw_line(unsigned char* yuv_buffer, double angle_deg,
                        int center_x, int center_y, int width, int height);
  void draw_mark_lines(unsigned char* yuv_buffer);
  static void draw_crosshair(unsigned char* yuv_buffer, int center_x,
                             int center_y, int width, int height);

  MirrorCalibTool();
  ~MirrorCalibTool();

  void load_mask(const char* mask_file_name);
  void push_back(const unsigned char* yuv_buffer,
                 size_t buflen,
                 int width,
                 int height,
                 double ori);
  void abort();
  void next_step();
  const unsigned char* get_last_yuv_buffer() const;
  const char* get_state_description() const;

  inline int center_x() const { return img_center_x_; } /**< Center X accessor. */
  inline int center_y() const { return img_center_y_; } /**< Center Y accessor. */

  void eval(unsigned int x,
            unsigned int y,
            float* x_ret,
            float* y_ret);
  /** Nothing.
   * @param total_steps */
  virtual void setTotalSteps(unsigned int total_steps) {};
  /** Nothing.
   * @param progress */
  virtual void setProgress(unsigned int progress) {};
  /** Nothing. */
  virtual void finished() {};
  
  void load(const char* filename);
  void save(const char* filename);

 private:
  class ConvexPolygon;
  class StepResult;
  typedef std::vector<StepResult> StepResultList;
  class Point;
  class PixelPoint;
  class CartesianPoint;
  class CartesianImage;
  class Hole;
  class HoleList;
  class Image;
  typedef std::vector<Hole> HoleList;
  typedef double PolarAngle;
  typedef int PolarRadius;
  typedef int RealDistance;
  typedef std::vector<PolarRadius> MarkList;
  typedef std::map<PolarAngle, MarkList> MarkMap;
  typedef std::pair<PolarAngle, PolarAngle> PolarAnglePair;
  typedef std::vector<Image> ImageList;

  class ConvexPolygon : public std::vector<PixelPoint> {
   public:
    bool contains(const CartesianImage& img, const CartesianPoint& r) const;
    bool contains(const PixelPoint& r) const;
  };

  enum StepName { SHARPENING, EDGE_DETECTION, COMBINATION, CENTERING,
                  PRE_MARKING, FINAL_MARKING, DONE };
  struct CalibrationState {
    StepName              step;
    ImageList::size_type  image_index;
    bool                  centering_done;
    CalibrationState()
    : step(SHARPENING), image_index(0), centering_done(false) {};
  };

  void goto_next_state();
  void set_last_yuv_buffer(const unsigned char* last_buf);
  void draw_center(StepResult& result);


  static PolarAngle relativeOrientationToImageRotation(PolarAngle ori);
  static PolarAngle imageRotationToRelativeOrientation(PolarAngle ori);

  static void apply_sobel(unsigned char* src, unsigned char* dst,
                          int widt, int height,
                          firevision::orientation_t ori);
  static void apply_sharpen(unsigned char* src, unsigned char* dst,
                            int widt, int height);
  static void apply_median(unsigned char* src, unsigned char* dst,
                           int widt, int height, int i);
  static void apply_min(unsigned char* src, unsigned char* dst,
                        int widt, int height);
  static void apply_or(unsigned char* src1, unsigned char* src2,
                       unsigned char* dst, int widt, int height);
  static void make_contrast(unsigned char* buf, size_t buflen);
  static void make_grayscale(unsigned char* buf, size_t buflen);
  static MarkList premark(const StepResult& prev, const unsigned char* yuv_mask,
                          StepResult& result, PolarAngle phi,
                          const PixelPoint& center);
  static MarkList premark(const ConvexPolygon& polygon, const StepResult& prev,
                          const unsigned char* yuv_mask, StepResult& result,
                          PolarAngle phi, const PixelPoint& center);
  static HoleList search_holes(const MarkList& premarks);
  static HoleList filter_biggest_holes(const HoleList& holes, unsigned int n);
  static MarkList determine_marks(const HoleList& holes);
  static MarkList mark(const MarkList& premarks, const unsigned char* yuv_mask,
                       StepResult& result, PolarAngle phi,
                       const PixelPoint& center);

  static PixelPoint calculate_center(const ImageList& images);
  static RealDistance calculate_real_distance(int n);
  static PolarAnglePair find_nearest_neighbors(PolarAngle angle,
                                               const MarkMap& mark_map);
  static RealDistance interpolate(PolarRadius radius, const MarkList& marks);
  static firevision::Bulb generate(int width, int height,
                                   const PixelPoint& center,
                                   const MarkMap& mark_map);

  unsigned char*   img_yuv_buffer_;
  int              img_center_x_;
  int              img_center_y_;
  unsigned char*   img_yuv_mask_;

  ImageList        source_images_;
  CalibrationState state_;
  MarkList         premarks_;
  MarkMap          mark_map_; /** orientations wrt robot (i.e. Y axis) */
  
  const unsigned char* last_yuv_buffer_;

#ifdef HAVE_BULB_CREATOR
  firevision::Bulb* bulb_;
  firevision::BulbGenerator* m_generator;
#endif
};

#endif /*  __FIREVISION_TOOLS_IMAGE_VIEWER_MIRROR_CALIB_H_ */

