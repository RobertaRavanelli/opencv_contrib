#include <iostream>
#include <opencv2/structured_light.hpp>
#include <opencv2/highgui.hpp>
#include <bitset>
#include <iostream>


using namespace cv;
using namespace std;

int main()
{
  // Setup GraycodePattern parameters
  structured_light::GrayCodePattern::Params params;

  const int num_images = 4;
  for (int i = 0; i < pow(2,num_images); i++){
    std::bitset<num_images> bin_x(i);
    cout << bin_x << endl;
  }

  // Change projector resolution
  params.width = 1280;//16;
  params.height = 800;

  // Set up GraycodePattern with params
  Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);

  // Storage for pattern
  std::vector<Mat> pattern;

  // Set dark color
  //Scalar dark = Scalar(0, 0, 255);  // red

  // Set light color
  //Scalar light = Scalar(0, 255, 255);  // yellow

  // Generate the pattern
  graycode->generate(pattern);//, dark, light);

  // Generate the all-white and all-black images needed for shadows mask computation
  Mat all_light;
  Mat all_dark;
  graycode->getImagesForShadowMasks(all_dark, all_light);
  imshow("light", all_light);
  imshow("dark", all_dark);

  // Show and save the pattern
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  for( int i = 0; i < (int)pattern.size(); i++ )
    {
      std::ostringstream ostr;
      ostr << "Pattern " << i << ".png";
      imshow(ostr.str(), pattern[i]);
      imwrite(path + "pattern/" + ostr.str(), pattern[i]);
    }

  waitKey(0);
  return 0;
}