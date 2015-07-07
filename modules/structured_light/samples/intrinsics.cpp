// To compile: g++ ./intrinsics.cpp `pkg-config --cflags --libs opencv` -o ./intrinsics && ./intrinsics

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;

// The chessboard is considered the origin of the world.
// So, it is the camera that is moving around, taking different shots of the camera.
// So, you can set the chessboard on some place (like the XY plane).
/*static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
 {
 corners.resize(0);

 for( int i = 0; i < boardSize.height; i++ )
 for( int j = 0; j < boardSize.width; j++ )
 corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
 }*/

void LoadData(string path, Size& camImageSize, vector<vector<cv::Point2f> >& imgBoardCornersCam,
              vector<vector<cv::Point3f> >& objBoardCornersCam)
{
  cv::FileStorage fs(path, FileStorage::READ);
  if( !fs.isOpened() )
    {
      std::cout << "Failed to open Calibration Data File. " << std::endl;
    }
  cv::FileNode node = fs["Camera"];

  // The 3D underworld valus for atzec dataset: they are our reference
  cv::Mat cameraMatrix1ref, distCoeffs1ref;

  node["Height"] >> camImageSize.height;
  node["Width"] >> camImageSize.width;
  node["Matrix"] >> cameraMatrix1ref;
  node["Distortion"] >> distCoeffs1ref;

  cout << "cam matrix (ref)" << endl << cameraMatrix1ref << endl;
  cout << "cam dist (ref)" << endl << distCoeffs1ref << endl;

  cv::FileNode features = fs["ExtractedFeatures"];
  cv::FileNode images = features["CameraImages"];
  int calibration_images = images["NumberOfImgs"];

  for( int i = 0; i < calibration_images; i++ )
    {
      std::stringstream name;
      name << "Image" << i + 1;

      cv::FileNode image = images[name.str()];

      std::vector<cv::Point2f> in2;
      std::vector<cv::Point3f> in3;

      image["BoardCorners"] >> in2;
      imgBoardCornersCam.push_back(in2);

      //cout << "Intrinsics corners image " << i + 1 << endl << in2 << endl;
      image["ObjBoardCorners"] >> in3;
      objBoardCornersCam.push_back(in3);
    }
  fs.release();
}

int main()
{
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  Size camImageSize;
  //location of the corners in the selected image (in 2D space)
  vector<vector<cv::Point2f> > imgBoardCornersCam;

  //physical position of the corners (in 3D space).
  vector<vector<cv::Point3f> > objBoardCornersCam;

  /*LoadData( path+ "atzec/PHOTOS/Dataset1/L/input/calib.xml",
   camImageSize,  imgBoardCornersCam,  objBoardCornersCam);*/

  LoadData(path + "output1/calib.xml", camImageSize, imgBoardCornersCam, objBoardCornersCam);
  cv::Mat intrinsics1 = Mat(3, 3, CV_64FC1);
  cv::Mat distCoeffs1;
  std::vector<cv::Mat> rvecs1, tvecs1;
  cout << endl << "Camera 1 intrinsics" << endl;
  double err1 = calibrateCamera(objBoardCornersCam, imgBoardCornersCam, camImageSize, intrinsics1, distCoeffs1, rvecs1,
                                tvecs1);

  cout << "calibration RMS reprojection error: " << err1 << endl;
  cout << "M_intrinsic_1 " << endl << " " << intrinsics1 << endl;
  cout << "M_distCoeffs_1 " << endl << " " << distCoeffs1 << endl << endl;

  //location of the corners in the selected image (in 2D space)
  vector<vector<cv::Point2f> > imgBoardCornersCam2;

  //physical position of the corners (in 3D space).
  vector<vector<cv::Point3f> > objBoardCornersCam2;
  LoadData(path + "output2/calib.xml", camImageSize, imgBoardCornersCam2, objBoardCornersCam2);
  cv::Mat intrinsics2 = Mat(3, 3, CV_64FC1);
  cv::Mat distCoeffs2;
  std::vector<cv::Mat> rvecs2, tvecs2;
  cout << endl << "Camera 2 intrinsics" << endl;

  double err2 = calibrateCamera(objBoardCornersCam2, imgBoardCornersCam2, camImageSize, intrinsics2, distCoeffs2,
                                rvecs2, tvecs2);

  cout << "calibration RMS reprojection error: " << err2 << endl;
  cout << "M_intrinsic_2 " << endl << " " << intrinsics2 << endl;
  cout << "M_distCoeffs_2 " << endl << " " << distCoeffs2 << endl;

  // SAVING FOUND CALIBRATION INTRINSICS PARAMETERS
  cv::FileStorage fs(path + "intrinsics.yml", FileStorage::WRITE);
  fs << "cam1_intrinsics" << intrinsics1;
  fs << "cam1_distorsion" << distCoeffs1;
  fs << "cam2_intrinsics" << intrinsics2;
  fs << "cam2_distorsion" << distCoeffs2;
  fs.release();

  return 0;
}