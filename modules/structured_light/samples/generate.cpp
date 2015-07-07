#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/structured_light/structured_light.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main()
{
  // Setup GraycodePattern parameters
  cv::structured_light::GrayCodePattern::Params params;

  // Change projector resolution
  params.width = 1024;
  params.height = 768;

  // Set up GraycodePattern with params
  cv::Ptr<cv::structured_light::GrayCodePattern> SL = cv::structured_light::GrayCodePattern::create(params);

  // Storage for pattern
  std::vector<cv::Mat> pattern;

  // Set dark color
  Scalar dark = Scalar(0, 0, 255);  // red

  // Set light color
  Scalar light = Scalar(0, 255, 255);  // yellow

  // Generate the pattern
  //SL->generate(pattern);
  SL->generate(pattern, dark, light);

  // Generate the all-white and all-black images needed for shadows mask computation
  cv::Mat all_light;
  cv::Mat all_dark;
  SL->getImagesForShadowMasks(all_dark, all_light);
  cv::imshow("light", all_light);
  cv::imshow("dark", all_dark);

  // Show and save the pattern
  std::string path = "/home/roberta/Sviluppo/DemoOpencv/";
  for( int i = 0; i < pattern.size(); i++ )
    {
      std::ostringstream ostr;
      ostr << "Pattern " << i << ".png";
      cv::imshow(ostr.str(), pattern[i]);
      cv::imwrite(path + "pattern/" + ostr.str(), pattern[i]);
    }

  cv::waitKey(0);
  return 0;
}