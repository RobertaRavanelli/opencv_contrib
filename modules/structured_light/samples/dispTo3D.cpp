// To build and run: g++ ./dispTo3D.cpp `pkg-config --cflags --libs opencv` -o ./dispTo3D && ./dispTo3D
// From the disparity previously comuted, using the calibration parameters,
// the program conputes the corresponding 3D point cloud

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/rgbd.hpp>
//#include "opencv2/rgbd.hpp"

using namespace cv;
using namespace std;



int main()
{
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  // Load disparity previously computed
  cv::Mat disp, tmp, cm_disp;
  cv::FileStorage fs_disp(path + "30_07/disparity.yml", FileStorage::READ);
  //cv::FileStorage fs_disp(path + "11_08/disparity.yml", FileStorage::READ);
  fs_disp["disparity"] >> disp;
  fs_disp.release();

  // Loading the Q matrix
  cv::Mat Q;
  cv::FileStorage fs_extr(path + "30_07/Q.yml", FileStorage::READ);
  //cv::FileStorage fs_extr(path + "11_08/Q.yml", FileStorage::READ);
  fs_extr["Q"] >> Q;

  // Loading the color image
  Mat color = cv::imread(path + "color.png", cv::IMREAD_COLOR);

  // Visualize the disparity
  double min, max;
  cv::minMaxIdx(disp, &min, &max);
  cv::convertScaleAbs(disp, tmp, 255 / (max - min));
  applyColorMap(tmp, cm_disp, COLORMAP_JET);
  // Show the result
  cv::resize(cm_disp, cm_disp, Size(640, 480));
  imshow("cm disparity m", cm_disp);
  cv::waitKey();

  // Computing the mask
  Mat thresholded_disp, dst;
  threshold(tmp, thresholded_disp, 0, 255, cv::THRESH_OTSU + cv::THRESH_BINARY);
  cv::resize(thresholded_disp, dst, Size(640, 480));
  imshow("threshold disp otsu", dst);
  cv::waitKey();

  // Computing the point cloud
  Mat pointcloud;
  disp.convertTo(disp, CV_32FC1);
  cv::reprojectImageTo3D(disp, pointcloud, Q, true, -1);
  //Vec3f intensity = pointcloud.at<Vec3f>(j, i);

  // Apply the mask
  Mat pointcloud_tresh, color_tresh;
  pointcloud.copyTo(pointcloud_tresh, thresholded_disp);
  color.copyTo(color_tresh, thresholded_disp);
  /*pointcloud.copyTo(pointcloud_tresh);//;, thresholded_disp);
  color.copyTo(color_tresh);//, thresholded_disp);*/

  // Visualize the point cloud with viz
/*  viz::Viz3d myWindow("show_cloud_with_color");
  myWindow.setBackgroundMeshLab();
  myWindow.showWidget("coosys", viz::WCoordinateSystem());
  myWindow.showWidget("pointcloud", viz::WCloud(pointcloud_tresh / 1000, color_tresh));
  myWindow.showWidget("text2d", viz::WText("Point cloud", Point(20, 20), 20, viz::Color::green()));
  myWindow.spin();

  /*cv::Mat mask;
  Mat_ < Vec3f > points3d;
  std::vector<Vec4f> plane_coefficients;
  //std::vector<Plane> planes;
  const Mat_<unsigned char> plane_mask;
   //ground_normals;
  rgbd::RgbdPlane plane_computer;
  plane_computer(points3d, plane_mask, plane_coefficients);*/
  //plane_computer(points3d, plane_mask, plane_coefficients);



  Mat plane_mask;
  std::vector<Vec4f> plane_coefficients;
   vector<float> coeff;
   rgbd::RgbdPlane plane_computer;
   plane_computer(thresholded_disp, plane_mask, plane_coefficients);

   cout << plane_coefficients[0] << endl;
   imshow ("plane_mask", plane_mask);
  cv::waitKey(0);
 return 0;
}

