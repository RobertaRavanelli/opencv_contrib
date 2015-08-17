//To build from terminal g++ ./rect_and_decode.cpp `pkg-config --cflags --libs opencv` -o ./rect_and_decode && ./rect_and_decode
//To build from terminal g++ ./rect_and_decode.cpp `pkg-config --cflags --libs opencv` -g -o ./rect_and_decode && gdb ./rect_and_decode

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/viz.hpp>
#include "opencv2/rgbd.hpp"

using namespace std;
using namespace cv;


int main()
{
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  cv::FileStorage fs(path + "11_08/calibrationParameters.yml", FileStorage::READ);
  if( !fs.isOpened() )
    {
      std::cout << "Failed to open Calibration Data File. " << std::endl;
    }

  Size calib_camImageSize;
  Mat cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, R, T;

  fs["cam1_size"] >> calib_camImageSize;
  fs["cam1_intrinsics"] >> cam1intrinsics;
  fs["cam2_intrinsics"] >> cam2intrinsics;
  fs["cam1_distorsion"] >> cam1distCoeffs;
  fs["cam2_distorsion"] >> cam2distCoeffs;
  fs["R"] >> R;
  fs["T"] >> T;

  cout << calib_camImageSize << endl;


  Mat color = cv::imread(path + "11_08/bassa_risoluzione/pattern_cam1_white.png");

  Size FRimageSize = color.size();//(5184, 3456);  // full resolution cam image size
  double rap_w = (double) FRimageSize.width / calib_camImageSize.width;
  double rap_h = (double) FRimageSize.height / calib_camImageSize.height;
  cout << rap_w << "\t" << rap_h << endl;  // they are the same

  cam1intrinsics = cam1intrinsics * rap_w;
  cam2intrinsics = cam2intrinsics * rap_h;

  cam1intrinsics.at<double>(2, 2) = 1.0;
  cam2intrinsics.at<double>(2, 2) = 1.0;

  cout << "prior cx " << FRimageSize.width / 2.0 << endl;
  cout << "prior cy " << FRimageSize.height / 2.0 << endl;

  cout << "cam1intrinsics" << endl << cam1intrinsics << endl;
  cout << "cam1distCoeffs" << endl << cam1distCoeffs << endl;
  cout << "cam2intrinsics" << endl << cam2intrinsics << endl;
  cout << "cam2distCoeffs" << endl << cam2distCoeffs << endl;
  cout << "T" << endl << T << endl << "R" << endl << R << endl;

  // Stereo rectify
  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];
  cv::stereoRectify(cam1intrinsics, cam1distCoeffs, cam2intrinsics, cam2distCoeffs, FRimageSize, R, T, R1, R2, P1, P2,
                    Q, 0, -1, FRimageSize, &validRoi[0], &validRoi[1]);
  //Q, CALIB_ZERO_DISPARITY, -1, FRimageSize, &validRoi[0], &validRoi[1]);

  Mat map1x, map1y, map2x, map2y;
  Mat imgU1, imgU2;

  // in pattern acquisition, cam 1 is right and cam 2 is left, while during calibration cam 1 was left and cam2 was right
  initUndistortRectifyMap(cam1intrinsics, cam1distCoeffs, R1, P1, FRimageSize, CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(cam2intrinsics, cam2distCoeffs, R2, P2, FRimageSize, CV_32FC1, map2x, map2y);

  std::vector<std::vector<cv::Mat> > captured_pattern;

  captured_pattern.resize(2);
  captured_pattern[0].resize(42);
  captured_pattern[1].resize(42);



  for( size_t i = 0; i < captured_pattern[1].size(); i++ )
    {
      std::ostringstream name1;

      //name1 << "30_07/MacGrande/pattern_cam1_im" << i + 1 << ".png";
      name1 << "11_08/bassa_risoluzione/pattern_cam1_im" << i + 1 << ".png";
      //name1 << "30_07/prova_scat_macbook/pattern_cam1_im" << i + 1 << ".png";

      cout << name1.str() << endl;
      captured_pattern[0][i] = cv::imread(path + name1.str(), 0);
      Mat tmp;

      remap(captured_pattern[0][i], captured_pattern[0][i], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

      std::ostringstream name2;

      //name2 << "30_07/MacGrande/pattern_cam2_im" << i + 1 << ".png";
      name2 << "11_08/bassa_risoluzione/pattern_cam2_im" << i + 1 << ".png";
      //name2 << "30_07/prova_scat_macbook/pattern_cam2_im" << i + 1 << ".png";

      captured_pattern[1][i] = cv::imread(path + name2.str(), 0);
      remap(captured_pattern[1][i], captured_pattern[1][i], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
      cout << name2.str() << endl;

      cv::resize(captured_pattern[0][i], tmp, Size(640,480));
      cv::imshow("ret cam 1", tmp);
      cv::resize(captured_pattern[1][i], tmp, Size(640,480));

       cv::imshow("ret cam 2",tmp);
       cv::waitKey();
    }

  cv::structured_light::GrayCodePattern::Params params;

  // Change projector resolution
  params.width = 1280;  // 1024;
  params.height = 800;  //768;

  // Set up GraycodePattern with params
  cv::Ptr<cv::structured_light::GrayCodePattern> SL = cv::structured_light::GrayCodePattern::create(params);

  Mat disparityMap;
  vector<Mat> camerasMatrix;
  vector<Mat> camerasDistCoeffs;
  vector<Mat> camerasRotationMatrix;
  vector<Mat> camerasTranslationVector;
  vector<Mat> darkImages;
  vector<Mat> lightImages;

  darkImages.resize(2);
  lightImages.resize(2);
  /*Mat color = cv::imread(path + "30_07/MacGrande/pattern_cam1_white.png");
  lightImages[0] = cv::imread(path + "30_07/MacGrande/pattern_cam1_white.png", 0);
  lightImages[1] = cv::imread(path + "30_07/MacGrande/pattern_cam2_white.png", 0);

   darkImages[0] = cv::imread(path + "30_07/MacGrande/pattern_cam1_black.png", 0);
   darkImages[1] = cv::imread(path + "30_07/MacGrande/pattern_cam2_black.png", 0);*/


     lightImages[0] = cv::imread(path + "11_08/bassa_risoluzione/pattern_cam1_white.png", 0);
     lightImages[1] = cv::imread(path + "11_08/bassa_risoluzione/pattern_cam2_white.png", 0);

      darkImages[0] = cv::imread(path + "11_08/bassa_risoluzione/pattern_cam1_black.png", 0);
      darkImages[1] = cv::imread(path + "11_08/bassa_risoluzione/pattern_cam2_black.png", 0);

  /*lightImages[0] = cv::imread(path + "30_07/prova_scat_macbook/pattern_cam1_white.png", 0);
  lightImages[1] = cv::imread(path + "30_07/prova_scat_macbook/pattern_cam2_white.png", 0);

  darkImages[0] = cv::imread(path + "30_07/prova_scat_macbook/pattern_cam1_black.png", 0);
  darkImages[1] = cv::imread(path + "30_07/prova_scat_macbook/pattern_cam2_black.png", 0);*/

  remap(color, color, map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
  imwrite(path+"color.png", color);
  //cv::waitKey();

  remap(lightImages[0], lightImages[0], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
  remap(lightImages[1], lightImages[1], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

  remap(darkImages[0], darkImages[0], map1x, map1y, INTER_NEAREST, BORDER_CONSTANT, Scalar());
  remap(darkImages[1], darkImages[1], map2x, map2y, INTER_NEAREST, BORDER_CONSTANT, Scalar());

  //macbook grande
  SL->setBlackThreshold(20);
  SL->setWhiteThreshold(5);

  /*SL->setDarkThreshold(8);
    SL->setLightThreshold(5);*/
  Mat thresholded_disp;
  bool decoded = SL->decode(captured_pattern, disparityMap, darkImages, lightImages,
                            cv::structured_light::DECODE_3D_UNDERWORLD);
if (decoded)
    {
      cout << "pattern decoded" << endl;
      // SAVING COMPUTED DISPARITY
      cv::FileStorage fs1(path + "/11_08/disparity.yml", FileStorage::WRITE);
      fs1 << "disparity" << disparityMap;
      fs1.release();
      Mat tmp,cm_disp;
      double min;
      double max;
      cv::minMaxIdx(disparityMap, &min, &max);
      cout << "disp min " << min << endl << "disp max " << max << endl;

        cv::convertScaleAbs(disparityMap, tmp, 255 / (max - min));
        applyColorMap(tmp, cm_disp, COLORMAP_JET);
        // Show the result
        cv::resize(cm_disp, cm_disp, Size(640, 480));
        imshow("cm disparity m", cm_disp);
        cv::waitKey();

     /* threshold(tmp, thresholded_disp, 50, max, 0);
      Mat dst;
      cv::resize(thresholded_disp, dst, Size(640, 480));

      imshow("threshold disp", dst);
      cv::waitKey();*/



     /* cv::Mat cm_img0;
      //applyColorMap(tmp, cm_img0, COLORMAP_JET);
      // Show the result
      cv::resize(tmp, tmp, Size(640, 480));
      imshow("cm disparity", tmp);*/


    }

  // Computing the point cloud
  Mat pointcloud;
  //

  cv::FileStorage fs4(path + "11_08/Q.yml", FileStorage::WRITE);
    fs4 << "Q" << Q;
    fs4.release();
  cout << "Q" << endl << Q << endl;
  double rap_w_pi = (double) params.width / FRimageSize.width;
  double rap_h_pi = (double) params.height / FRimageSize.height;
  cout << rap_w_pi << "\t" << rap_h_pi << endl;  // they are the same

  disparityMap.convertTo(disparityMap, CV_32FC1);
  cv::reprojectImageTo3D(disparityMap, pointcloud, Q, true, -1);

  // Saving the computed point cloud as a file (it can be opened in meshlab)
  ofstream myfile;

  //float Zmax = 0;
  // http://choorucode.com/2011/08/18/ply-file-format/
  /*myfile.open("/home/roberta/Sviluppo/DemoOpencv/pointcloud.ply");
  myfile << "ply" << endl;
  myfile << "format ascii 1.0" << endl;
  myfile << "element vertex " << pointcloud.rows * pointcloud.cols << endl;
  myfile << "property float x" << endl;
  myfile << "property float y" << endl;
  myfile << "property float z" << endl;
  myfile << "property uchar red" << endl;
  myfile << "property uchar green" << endl;
  myfile << "property uchar blue" << endl;
  myfile << "element face " << 0 << endl;
  myfile << "property list uchar int vertex_index" << endl;
  myfile << "end_header" << endl;

  cout << pointcloud.rows << endl << pointcloud.cols << endl;

  for( int j = 0; j < pointcloud.rows; j++ )
    {
      for( int i = 0; i < pointcloud.cols; i++ )
        {
          if( thresholded_disp.at<uchar>(j, i) != 0 )
                      {
          Vec3f intensity = pointcloud.at<Vec3f>(j, i);
          float X = intensity.val[0];
          float Y = intensity.val[1];
          float Z = intensity.val[2];
          Vec3b col = color.at<Vec3b>(j, i);
         uchar Red = col.val[0];
         uchar Green = col.val[1];
         uchar Blue = col.val[2];

          myfile << X << ' ' << Y << ' ' << Z << ' ' << (int) Red << ' ' << (int) Green << ' ' << (int)Blue << endl;}
          else

                       myfile << 0 << ' ' << 0 << ' ' << 0 << ' ' << (int) 0 << ' ' << (int) 0 << ' ' << (int) 0
                                           << endl;
          //myfile << X << ' ' << Y << ' ' << Z << ' ' << Red << endl;
        }
    }
  myfile.close();*/

  Mat tmp;
  //cvtColor(disparityMap, tmp, CV_RGBA2GRAY );
  /*cv::resize(tmp, tmp, Size(640, 480));
  imshow("bynary disp", tmp);

  threshold( tmp, dst, threshold_value, max_BINARY_value,threshold_type );

   imshow( thesh, dst );*/

  cout << "end of saving .ply file" << endl;

  

  cv::waitKey();
  return 0;
}
