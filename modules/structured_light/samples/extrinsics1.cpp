// To compile: g++ ./extrinsics1.cpp `pkg-config --cflags --libs opencv` -o ./extrinsics1 && ./extrinsics1
// Find the extrinsics parameters using the values stored in atzec dataset

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

// The chessboard is considered the origin of the world.
//So, it is the camera that is moving around, taking different shots of the camera.
//So, you can set the chessboard on some place (like the XY plane).
static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
  corners.resize(0);

  for( int i = 0; i < boardSize.height; i++ )
    for( int j = 0; j < boardSize.width; j++ )
      corners.push_back(Point3f(float(j * squareSize), float(i * squareSize), 0));
}

int main()
{
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  Mat distCoeffs1, cameraMatrix1, distCoeffs2, cameraMatrix2;

  cv::FileStorage fs(path + "intrinsics.yml", FileStorage::READ);
  if( !fs.isOpened() )
    {
      std::cout << "Failed to open intrinsics Data File. " << std::endl;
    } else
    {
      fs["cam1_intrinsics"] >> cameraMatrix1;
      cout << "cameraMatrix1" << endl << cameraMatrix1 << endl;
      fs["cam1_distorsion"] >> distCoeffs1;
      cout << "distCoeffs1" << endl << distCoeffs1 << endl;

      fs["cam2_intrinsics"] >> cameraMatrix2;
      cout << "cameraMatrix2" << endl << cameraMatrix2 << endl;
      fs["cam2_distorsion"] >> distCoeffs2;
      cout << "distCoeffs2" << endl << distCoeffs2 << endl;
    }

  // Extrinsics calibration
  Size boardSize = Size(5, 4);
  Size ns = Size(640, 480);
  vector<Mat> extrinsics;
  extrinsics.resize(2);

  extrinsics[0] = cv::imread(path + "atzec/CALIBRATION/L/extr_1.png", 0);
  extrinsics[1] = cv::imread(path + "atzec/CALIBRATION/R/extr.JPG", 0);

  vector<vector<Point2f> > camCorners1;
  vector<vector<Point2f> > camCorners2;
  for( int i = 0; i < 2; i++ )
    {
      std::vector<cv::Point2f> extr_corners;
      bool found = findChessboardCorners(extrinsics[i], boardSize, extr_corners, CALIB_CB_ADAPTIVE_THRESH);  //+ CALIB_CB_FILTER_QUADS + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

      if( found )
        {
          cornerSubPix(extrinsics[i], extr_corners, Size(11, 11), Size(-1, -1),
                       TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 0));
          //drawChessboardCorners(extrinsics[i], boardSize, Mat(extr_corners), found);
          cout << "cam " << i + 1 << " corners" << endl << extr_corners << endl;
          if( i == 0 )
            {
              camCorners1.push_back(extr_corners);
            }
          if( i == 1 )
            {
              camCorners2.push_back(extr_corners);
            }
        }
    }

  //physical position of the corners (in 3D space).
  vector<vector<Point3f> > objectPoints(1);
  calcChessboardCorners(boardSize, 35.0, objectPoints[0]);

  cout << "objectPoints" << endl << objectPoints[0] << endl;

  cout << "Solve PnP results" << endl;
  objectPoints.resize(camCorners1.size(), objectPoints[0]);
  cv::Mat rVec, translation1, rotation1;
  bool r1 = cv::solvePnP(objectPoints[0], camCorners1[0], cameraMatrix1, distCoeffs1, rVec, translation1);
  cv::Rodrigues(rVec, rotation1);

  cout << "cam 1 T" << endl << translation1 << endl;
  cout << "cam 1 R" << endl << rotation1 << endl;

  cv::Mat translation2, rotation2;
  bool r2 = cv::solvePnP(objectPoints[0], camCorners2[0], cameraMatrix2, distCoeffs2, rVec, translation2);
  cv::Rodrigues(rVec, rotation2);

  cout << "cam 2 T" << endl << translation2 << endl;
  cout << "cam 2 R" << endl << rotation2 << endl;

  cv::Mat R, T, E, F;
  Size camImageSize = Size(4896, 3264);
  double errSTEREO = stereoCalibrate(objectPoints, camCorners1, camCorners2, cameraMatrix1, distCoeffs1, cameraMatrix2,
                                     distCoeffs2, camImageSize, R, T, E, F, CALIB_FIX_INTRINSIC,
                                     /*CALIB_FIX_ASPECT_RATIO +
                                      CALIB_ZERO_TANGENT_DIST +
                                      CALIB_SAME_FOCAL_LENGTH +
                                      CALIB_RATIONAL_MODEL +
                                      CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5*/
                                     TermCriteria(TermCriteria::COUNT, 30, 0));

  cout << "Stereo results: err " << errSTEREO << endl;
  cout << "R" << endl << R << endl;
  cout << "T" << endl << T << endl;

  Mat R1, R2, P1, P2, Q;
  Rect validRoi[2];
  cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, camImageSize, R, T, R1, R2, P1, P2, Q, 0,
                    -1, Size(1024, 768), &validRoi[0], &validRoi[1]);  // size del proiettore, come quella della mia mappa di profondità
  //-1, camImageSize, &validRoi[0], &validRoi[1]);

  cout << "StereoRectify results" << endl;
  cout << "Q" << endl << Q << endl;

  // vari tentativi
  Mat map1x, map1y, map2x, map2y;
  Mat imgU1, imgU2;
  initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, camImageSize, CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, camImageSize, CV_32FC1, map2x, map2y);
  /*initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, Size(1024, 768), CV_32FC1, map1x, map1y);// così sarebbe concettualmente corretto, ma i risultati sono peggiori
   initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, Size(1024, 768), CV_32FC1, map2x, map2y);*/
  remap(extrinsics[0], extrinsics[0], map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
  remap(extrinsics[1], extrinsics[1], map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

  // Saves computed calibration parameters
  FileStorage fsout;
  fsout.open(path + "extrinsics.yml", CV_STORAGE_WRITE);
  if( fsout.isOpened() )
    {
      fsout << "rotation1PnP" << rotation1 << "translation1_PnP" << translation1 << "rotation2PnP" << rotation2
            << "translation2_PnP" << translation2 << "Rstereo" << R << "Tstereo" << T << "R1" << R1 << "R2" << R2
            << "P1" << P1 << "P2" << P2 << "Q" << Q;
      fsout.release();
    } else
    cout << "Error: can not save the extrinsics parameters\n";

  //  visualization
  cv::resize(extrinsics[0], extrinsics[0], ns);
  cv::imshow("extrinscs cam 1", extrinsics[0]);
  cv::resize(extrinsics[1], extrinsics[1], ns);
  cv::imshow("extrinscs cam 2", extrinsics[1]);
  cv::waitKey(0);
  return 0;
}