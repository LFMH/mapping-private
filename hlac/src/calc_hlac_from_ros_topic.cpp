#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <hlac/HLAC.h>
#include <sensor_msgs/image_encodings.h>

class ViewHist {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  std::vector<float> color_hlac;
  int hist_width;
  int hist_height;
  int max_hist_height;
  HLAC hlac;
public:
  ViewHist( const int _max_hist_height ) : 
    max_hist_height( _max_hist_height ) {
    hist_width = 400; //1000;
    hist_height = 100; //200;
    _sub = _nh.subscribe ("input", 1,  &ViewHist::color_hlac_cb, this);
    ROS_INFO ("Listening for incoming data on topic input" );
  }

  ~ViewHist() {}

  void color_hlac_cb( const sensor_msgs::ImageConstPtr& msg ){
    namespace enc = sensor_msgs::image_encodings;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    //* show color img
    cv::imshow("color_img", cv_ptr->image);
    cv::waitKey(1);

    //* extract colorHLAC
    hlac.extractColor( color_hlac, cv_ptr->image );

    const int bin_num = color_hlac.size();
    if( bin_num != 0 ){
      const int bin_size = hist_width / bin_num;
      if( bin_size < 1 ){
    	std::cerr << "Numer of bins is too large for histogram image. " << bin_num << " > " << hist_width << std::endl;
    	exit(1);
      }
      cv::Mat hist_img(hist_height, hist_width, CV_8UC1, 255);

      // draw histogram
      int h_height;
      for( int h=0; h<bin_num; h++ ){
    	h_height = (int) (color_hlac[ h ] * hist_height / max_hist_height);
    	if( h_height >= hist_height ) h_height = hist_height - 1;
    	cv::rectangle( hist_img, cv::Point( bin_size*h, hist_height-1 ), cv::Point( bin_size*(h+1), hist_height-h_height-1 ), 0, CV_FILLED );
      }
      
      cv::imshow("color_hlac", hist_img);
      const char key = cv::waitKey(3);
      if( key == 'q' ) exit(1);
    }
  }

};

int main( int argc, char** argv ){
  if( argc != 3 ){
    std::cerr << "usage: rosrun hlac calc_hlac_from_ros_topic <max_hist_height> input:=<topic name>" << std::endl;
    std::cerr << "  e.g. rosrun hlac calc_hlac_from_ros_topic 100000 input:=/image_color" << std::endl;
    exit(1);
  }

  ros::init(argc,argv,"calc_hlac_from_ros_topic");
  if( !ros::master::check() )  return 1;

  const int max_hist_height = atoi( argv[ 1 ] );

  ViewHist vh( max_hist_height );
  ros::spin();

  return(0);
}
