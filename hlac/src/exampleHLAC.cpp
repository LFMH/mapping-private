// Activate normalization of HLAC by the number of pixels
//#define ENABLE_NORMALIZATION

#include <hlac/HLAC.h>
#include <highgui.h>

int main( int argc, char** argv ){
  if( argc != 2 ){
    std::cerr << "Need one parameter! Syntax is: " << argv[0] << " {input_image_filename}" << std::endl; 
    return(-1);
  }
  std::vector<float> feature, feature_gray;

  // read
  cv::Mat img, img_gray;
  img      = cv::imread( argv[1] );
  img_gray = cv::imread( argv[1], 0 );

  // hlac
  HLAC hlac;
  hlac.extractColor( feature, img );
  hlac.extractGray( feature_gray, img_gray );

  // output
  for( int i=0; i<feature.size(); i++ )
    std::cout << feature[i] << std::endl;
  std::cout << std::endl;
  for( int i=0; i<feature_gray.size(); i++ )
    std::cout << feature_gray[i] << std::endl;

  return(0);
}
