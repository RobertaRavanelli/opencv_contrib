// To compile: g++ ./dispTo3D.cpp `pkg-config --cflags --libs opencv` -o ./dispTo3D && ./dispTo3D
// From the disparity previously comuted, using the calibration parameters,
// the program conputes the corresponding 3D point cloud

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  // Load disparity previously computed
  cv::Mat disp, tmp, cm_disp;
  cv::FileStorage fs_disp(path + "disparity_atzec5.yml", FileStorage::READ);
  fs_disp["disparity"] >> disp;
  fs_disp.release();

  // Visualize the disparity
  double min, max;
  cv::minMaxIdx(disp, &min, &max);
  cv::convertScaleAbs(disp, tmp, 255 / (max - min));
  applyColorMap(tmp, cm_disp, COLORMAP_JET);
  // Show the result
  imshow("cm disparity", cm_disp);

  // Loading the Q matrix
  cv::Mat Q;
  cv::FileStorage fs_extr(path + "extrinsics.yml", FileStorage::READ);
  fs_extr["Q"] >> Q;

  // Computing the point cloud
  Mat pointcloud;
  disp.convertTo(disp, CV_32FC1);
  cv::reprojectImageTo3D(disp, pointcloud, Q, false, -1);
  cv::imshow("point cloud", pointcloud);

  // Saving the computed point cloud as a file (it can be opened in meshlab)
  ofstream myfile;
  myfile.open("/home/roberta/Sviluppo/DemoOpencv/pointcloud.xyz");
  for( int j = 0; j < pointcloud.rows; j++ )
    {
      for( int i = 0; i < pointcloud.cols; i++ )
      //for( int j = 0; j < pointcloud.rows; j++ )
        {
          //couut << pointcloud.at<float>(j, i) << endl;
          Vec3f intensity = pointcloud.at<Vec3f>(j, i);
          float X = intensity.val[0];
          float Y = intensity.val[1];
          float Z = intensity.val[2];
          myfile << X << '\t' << Y << '\t' << Z << endl;
        }
    }
  myfile.close();

  FileStorage fsout;
  fsout.open(path + "pointcloud.yml", CV_STORAGE_WRITE);
  if( fsout.isOpened() )
    {
      fsout << "pointcloud" << pointcloud;
      fsout.release();
    } else
    cout << "Error: can not save the point cloud\n";
  cv::waitKey(0);
  return 0;
}