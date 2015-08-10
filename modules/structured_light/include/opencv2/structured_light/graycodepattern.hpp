/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2009, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifndef __OPENCV_GRAY_CODE_PATTERN_HPP__
#define __OPENCV_GRAY_CODE_PATTERN_HPP__

#include "opencv2/core.hpp"

namespace cv {
namespace structured_light {
//! @addtogroup structured_light
//! @{

/** @brief Class implementing the gray code pattern
 */
class CV_EXPORTS_W GrayCodePattern : public StructuredLightPattern
{
 public:

  /** @brief Parameters of StructuredLightPattern constructor.
   *  @param width Projector's width.
   *  @param height Projector's height.
   */
  struct CV_EXPORTS_W_SIMPLE Params
  {
    CV_WRAP
    Params();CV_PROP_RW
    int width;CV_PROP_RW
    int height;
  };

  /** @brief Constructor
   @param parameters GrayCodePattern parameters GrayCodePattern::Params: the width and the height of the projector.
   */
  CV_WRAP
  static Ptr<GrayCodePattern> create(const GrayCodePattern::Params &parameters = GrayCodePattern::Params());

  /** @brief Sets the value for set the value for light threshold, needed for decoding.

   @param value The desired light threshold value.
   */
  CV_WRAP
  virtual void setLightThreshold(size_t value) = 0;

  /** @brief Sets the value for dark threshold, needed for decoding.

   @param value The desired dark threshold value.
   */
  CV_WRAP
  virtual void setDarkThreshold(size_t value) = 0;

  /** @brief Generates The all-dark and all-light images needed for shadowMasks computation.
   *
   *  @param darkImage The generated all-dark image.
   *  @param lightImage The generated all-light image.
   */
  CV_WRAP
  virtual void getImagesForShadowMasks(InputOutputArray darkImage, InputOutputArray lightImage) const = 0;

  /** @brief For a (x,y) pixel of the camera returns the corresponding projector pixel.
   *
   *  @param patternImages The acquired pattern images.
   *  @param x x coordinate of the image pixel.
   *  @param y y coordinate of the image pixel.
   *  @param p_out Projectors pixel corresponding to the camera's pixel.
   */
  CV_WRAP
  virtual bool getProjPixel(InputArrayOfArrays patternImages, int x, int y, Point &p_out) const = 0;
};

//! @}
}
}
#endif
