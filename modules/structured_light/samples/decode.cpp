// To compile: g++ ./decode.cpp `pkg-config --cflags --libs opencv` -o ./decode && ./decode

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/structured_light.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

int main()
{
  // Set up GraycodePattern using default 3D Underworld parameters
  cv::Ptr<cv::structured_light::GrayCodePattern> SL = cv::structured_light::GrayCodePattern::create();

  std::vector<std::vector<cv::Mat> > captured_pattern;
  captured_pattern.resize(2);
  captured_pattern[0].resize(40);
  captured_pattern[1].resize(40);

  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  for( int i = 0; i < 40; i++ )
    {
      std::ostringstream name1;
      //name1 << "alexander/left_cam/dataset1/IMG_0000" << 922+i << ".JPG";//942 -961 colonne //922-941
      //name1 << "diana/diana/LEFT/DATASET01/AI4X" << 7344 + i << ".JPG";
      //name1 << "atzec/PHOTOS/Dataset2/L/AI4X" << i + 7187 << ".JPG";// atzec 2nd dataset
      //name1 << "atzec/PHOTOS/Dataset3/L/AI4X" << i + 7229 << ".JPG";// atzec 3rd dataset
      //name1 << "atzec/PHOTOS/Dataset4/L/AI4X" << i + 7271 << ".JPG";// atzec 4th dataset
      name1 << "atzec/PHOTOS/Dataset5/L/AI4X" << i + 7313 << ".JPG";   // atzec 5th dataset

      cout << name1.str() << endl;
      captured_pattern[0][i] = cv::imread(path + name1.str(), 0);

      std::ostringstream name2;
      //name2 << "diana/diana/RIGHT/DATASET01/AI2X" << 1062+i << ".JPG";
      //name2 << "atzec/PHOTOS/Dataset2/R/AI2X0" << 891 + i << ".JPG";// atzec 2nd dataset
      //name2 << "atzec/PHOTOS/Dataset3/R/AI2X0" << 933 + i << ".JPG";// atzec 3rd dataset
      //name2 << "atzec/PHOTOS/Dataset4/R/AI2X" << 975 + i << ".JPG";// atzec 4th dataset
      name2 << "atzec/PHOTOS/Dataset5/R/AI2X" << 1017 + i << ".JPG";   // atzec 5th dataset
      captured_pattern[1][i] = cv::imread(path + name2.str(), 0);
      cout << name2.str() << endl;
    }

  Mat disparityMap;
  vector<Mat> darkImages;
  vector<Mat> lightImages;

  darkImages.resize(2);
  lightImages.resize(2);

  /*SL->setDarkThreshold(8);
   SL->setLightThreshold(5);*/
  /* SL->setDarkThreshold(40);
   SL->setLightThreshold(5);*/
  //load camera images
  /*lightImages[0] = cv::imread(path +"alexander/left_cam/dataset1/IMG_0000920.JPG",0);
   lightImages[1] = cv::imread(path +"alexander/right_cam/dataset1/IMG_0000829.JPG",0);

   darkImages[0] = cv::imread(path +"alexander/left_cam/dataset1/IMG_0000921.JPG",0);
   darkImages[1] = cv::imread(path +"alexander/right_cam/dataset1/IMG_0000830.JPG",0);*/

  // diana dataset
  /*lightImages[0] = cv::imread(path +"diana/diana/LEFT/DATASET01/AI4X7342.JPG",0);
   lightImages[1] = cv::imread(path +"diana/diana/RIGHT/DATASET01/AI2X1060.JPG",0);

   darkImages[0] = cv::imread(path +"diana/diana/LEFT/DATASET01/AI4X7343.JPG",0);
   darkImages[1] = cv::imread(path +"diana/diana/RIGHT/DATASET01/AI2X1061.JPG",0);*/

  // atzec 2nd dataset
  /*lightImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset2/L/AI4X7185.JPG", 0);
   lightImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset2/R/AI2X0889.JPG", 0);

   darkImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset2/L/AI4X7186.JPG", 0);
   darkImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset2/R/AI2X0890.JPG", 0);*/

  // atzec 3rd dataset
  /*lightImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset3/L/AI4X7227.JPG", 0);
   lightImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset3/R/AI2X0931.JPG", 0);

   darkImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset3/L/AI4X7228.JPG", 0);
   darkImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset3/R/AI2X0932.JPG", 0);*/

  // atzec 4th dataset
  /*lightImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset4/L/AI4X7269.JPG", 0);
   lightImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset4/R/AI2X0973.JPG", 0);

   darkImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset4/L/AI4X7270.JPG", 0);
   darkImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset4/R/AI2X0974.JPG", 0);*/

  // atzec 5th dataset
  lightImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset5/L/AI4X7311.JPG", 0);
  lightImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset5/R/AI2X1015.JPG", 0);

  darkImages[0] = cv::imread(path + "atzec/PHOTOS/Dataset5/L/AI4X7312.JPG", 0);
  darkImages[1] = cv::imread(path + "atzec/PHOTOS/Dataset5/R/AI2X1016.JPG", 0);

  bool decoded = SL->decode(captured_pattern, disparityMap, darkImages, lightImages,
                            cv::structured_light::DECODE_3D_UNDERWORLD);

  if( decoded )
    {
      // SAVING COMPUTED DISPARITY
      cv::FileStorage fs(path + "disparity.yml", FileStorage::WRITE);
      fs << "disparity" << disparityMap;
      fs.release();
      double min;
      double max;
      cv::minMaxIdx(disparityMap, &min, &max);
      cv::convertScaleAbs(disparityMap, disparityMap, 255 / max);
      cv::Mat cm_img0;
      applyColorMap(disparityMap, cm_img0, COLORMAP_JET);
      // Show the result
      cv::resize(cm_img0, cm_img0, Size(640,480));
      imshow("cm disparity", cm_img0);
    }

  // ******************** Visualize the captured pattern and resize in order to visualize ********
  /*Size ns = Size(640, 480);
   cout << "light 1" << endl;
   resize(lightImages[0], lightImages[0], ns);
   cout << "light 2" << endl;
   resize(lightImages[1], lightImages[1], ns);
   cout << "dark 1" << endl;
   resize(darkImages[0], darkImages[0], ns);
   cout << "dark 2" << endl;
   resize(darkImages[1], darkImages[1], ns);

   imshow("light cam 1", lightImages[0]);
   imshow("light cam 2", lightImages[1]);
   imshow("dark cam 1", darkImages[0]);
   imshow("dark cam 2", darkImages[1]);

   for( int i = 0; i < 2; i++ )
   {
   for( int k = 0; k < pattern.size(); k++ )
   {
   std::ostringstream name;
   name << "captured pattern cam" << i + 1 << " " << k << ".JPG";
   cout << name.str() << endl;
   cv::resize(captured_pattern[i][k], captured_pattern[i][k], ns);
   cv::imshow(name.str(), captured_pattern[i][k]);
   }
   }*/

  cv::waitKey(0);
  return 0;
}