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
 // Copyright (C) 2013, OpenCV Foundation, all rights reserved.
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

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/viz.hpp>


using namespace std;
using namespace cv;

static const char* keys =
{ "{@images_list | | Image list where the captured pattern images are saved}"
    "{@calib_param_path     | | Calibration_parameters            }"
    "{@proj_width      | | The projector width used to acquire the pattern          }"
    "{@proj_height     | | The projector height used to acquire the pattern}" };

static void help()
{
  cout
      << "\nThis example shows how to use the \"Structured Light module\" to decode a previously acquired gray code pattern"
      "\nCall:\n"
      "./example_structured_light_pointcloud <images_list>  <calib_param_path> <proj_width> <proj_height> \n"
      << endl;
}

static bool readStringList(const string& filename, vector<string>& l)
{
  l.resize(0);
  FileStorage fs(filename, FileStorage::READ);
  if( !fs.isOpened() )
    {
      cerr << "failed to open " << filename << endl;

      return -1;
    }
  FileNode n = fs.getFirstTopLevelNode();
  if( n.type() != FileNode::SEQ )
    {
      cerr << "cam 1 images are not a sequence! FAIL" << endl;
      return -1;
    }

  FileNodeIterator it = n.begin(), it_end = n.end();
  for( ; it != it_end; ++it )
    {
      l.push_back((string) *it);
    }

  n = fs["cam2"];
  if( n.type() != FileNode::SEQ )
    {
      cerr << "cam 2 images are not a sequence! FAIL" << endl;
      return -1;
    }

  it = n.begin(), it_end = n.end();
  for( ; it != it_end; ++it )
    {
      cout << (string) *it << "\n";
      l.push_back((string) *it);
    }

  if( l.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        return -1;
    }
  return true;
}

int main(int argc, char** argv)
{
  structured_light::GrayCodePattern::Params params;
  CommandLineParser parser(argc, argv, keys);
  String images_file = parser.get<String>(0);
  String calib_file = parser.get<String>(1);


  if( images_file.empty() || calib_file.empty() || params.width < 1 || params.height < 1 )
    {
      help();
      return -1;
    }

  params.width = parser.get<int>(2);
  params.height = parser.get<int>(3);

  std::vector<std::vector<Mat> > captured_pattern;

  captured_pattern.resize(2);
  captured_pattern[0].resize(42);
  captured_pattern[1].resize(42);

  vector<string> imagelist;
  bool ok = readStringList(images_file, imagelist);
  if( !ok || imagelist.empty() )
    {
      cout << "can not open " << images_file << " or the string list is empty" << endl;

    }

  FileStorage fs(calib_file, FileStorage::READ);
  if( !fs.isOpened() )
    {
      std::cout << "Failed to open Calibration Data File." << std::endl;
    }

  Mat cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, R, T;

  fs["cam1_intrinsics"] >> cam1intrinsics;
  fs["cam2_intrinsics"] >> cam2intrinsics;
  fs["cam1_distorsion"] >> cam1distCoeffs;
  fs["cam2_distorsion"] >> cam2distCoeffs;
  fs["R"] >> R;
  fs["T"] >> T;

  Mat color = imread(imagelist[captured_pattern[0].size()]);
  Size imagesSize = color.size();

  cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
  cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;
  cout << "cam2intrinsics" << endl << cam2intrinsics << endl;
  cout << "cam2distCoeffs" << endl << cam2distCoeffs << endl;
  cout << "T" << endl << T << endl << "R" << endl << R << endl;

  // Stereo rectify
  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];
  stereoRectify(cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, imagesSize, R, T, R1, R2, P1, P2, Q,
                    0, -1, imagesSize, &validRoi[0], &validRoi[1]);

  Mat map1x, map1y, map2x, map2y;
  initUndistortRectifyMap(cam1intrinsics, cam1distCoeffs, R1, P1, imagesSize, CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(cam2intrinsics, cam2distCoeffs, R2, P2, imagesSize, CV_32FC1, map2x, map2y);

  for( size_t i = 0; i < captured_pattern[1].size(); i++ )
    {

      captured_pattern[0][i] = imread(imagelist[i], 0);
      captured_pattern[1][i] = imread(imagelist[i + captured_pattern[1].size() + 2], 0);

      remap(captured_pattern[1][i], captured_pattern[1][i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
      remap(captured_pattern[0][i], captured_pattern[0][i], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());


      /*Mat tmp;

      resize(captured_pattern[0][i], tmp, Size(640,480));
       imshow("cam1 (left) rect", tmp);

       resize(captured_pattern[1][i], tmp, Size(640,480));
       imshow("cam2 (right) rect",tmp);
       waitKey();*/
    }

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);

  vector<Mat> darkImages;
  vector<Mat> lightImages;

  darkImages.resize(2);
  lightImages.resize(2);

  cvtColor(color, lightImages[0], COLOR_RGB2GRAY);

  lightImages[1] = imread(imagelist[2 * captured_pattern[1].size() + 2], 0);
  darkImages[0] = imread(imagelist[captured_pattern[0].size() + 1], 0);
  darkImages[1] = imread(imagelist[2 * captured_pattern[1].size() + 2 + 1], 0);

  remap(color, color, map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

  remap(lightImages[0], lightImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
  remap(lightImages[1], lightImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

  remap(darkImages[0], darkImages[0], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
  remap(darkImages[1], darkImages[1], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

  /*graycode->setDarkThreshold(55);
  graycode->setLightThreshold(10);*/

  graycode->setDarkThreshold(22);
  graycode->setLightThreshold(5);

  cout << endl << "Decoding pattern ..." << endl;
  Mat disparityMap;
  bool decoded = graycode->decode(captured_pattern, disparityMap, darkImages, lightImages,
                            structured_light::DECODE_3D_UNDERWORLD);
  if( decoded )
    {
      cout << "pattern decoded" << endl;

      Mat cm_disp, scaledDisparityMap;
      double min;
      double max;
      minMaxIdx(disparityMap, &min, &max);
      cout << "disp min " << min << endl << "disp max " << max << endl;

      convertScaleAbs(disparityMap, scaledDisparityMap, 255 / (max - min));
      applyColorMap(scaledDisparityMap, cm_disp, COLORMAP_JET);
      // Show the result
      resize(cm_disp, cm_disp, Size(640, 480));
      imshow("cm disparity m", cm_disp);

      // Computing the mask to remove background
      Mat dst, thresholded_disp;
      threshold(scaledDisparityMap, thresholded_disp, 0, 255, THRESH_OTSU + THRESH_BINARY);
      resize(thresholded_disp, dst, Size(640, 480));
      imshow("threshold disp otsu", dst);

      // Computing the point cloud
      Mat pointcloud;
      disparityMap.convertTo(disparityMap, CV_32FC1);
      reprojectImageTo3D(disparityMap, pointcloud, Q, true, -1);

      // Apply the mask
      Mat pointcloud_tresh, color_tresh;
      pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
      color.copyTo(color_tresh, thresholded_disp);

      // Showing the point cloud on viz
      viz::Viz3d myWindow("Point cloud with color");
      myWindow.setBackgroundMeshLab();
      myWindow.showWidget("coosys", viz::WCoordinateSystem());
      myWindow.showWidget("pointcloud", viz::WCloud(pointcloud_tresh / 1000, color_tresh));
      myWindow.showWidget("text2d", viz::WText("Point cloud", Point(20, 20), 20, viz::Color::green()));
      myWindow.spin();

    }

  waitKey();
  return 0;
}
