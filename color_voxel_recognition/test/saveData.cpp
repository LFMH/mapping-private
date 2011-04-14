#include <visualization_msgs/Marker.h>
#include <float.h>
#include "color_voxel_recognition/FILE_MODE"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

//**************************************************************************************************//
//* save point clouds                                                                              *//
//*   press any key to save, "Ctrl+C" to quit.                                                     *//
//*   if you give "distance_th" as command-line arg,                                               *//
//*    the point cloud will be limited by "distance_th" depth from the closest point to viewpoint. *//
//*   (this limitation is used for segmenting the target object in training process)               *//
//**************************************************************************************************//

using namespace std;

float DISTANCE_TH = 5.0;
bool save_flg = false;

int kbhit(){
  struct timeval tv = { 0L, 0L };
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

//******************************************
//* limit points by certain depth threshold
template <typename T>
void limitPoint( const pcl::PointCloud<T> input_cloud, pcl::PointCloud<T> &output_cloud, const float dis_th ){
  output_cloud.width = input_cloud.width;
  output_cloud.height = input_cloud.height;
  const int v_num = input_cloud.points.size();
  output_cloud.points.resize( v_num );

  int idx = 0;
  for( int i=0; i<v_num; i++ ){
    if( input_cloud.points[ i ].z < dis_th ){
      output_cloud.points[ idx ] = input_cloud.points[ i ];
      idx++;
    }
  }
  output_cloud.width = idx;
  output_cloud.height = 1;
  output_cloud.points.resize( idx );
}

//**************
//* set marker
visualization_msgs::Marker setMarker( const int id_, const float pos_x, const float pos_y, const float pos_z, const float scale_x, const float scale_y, const float scale_z, const float ca, const float cr, const float cg, const float cb ){
  visualization_msgs::Marker marker_;
  marker_.header.frame_id = "/openni_rgb_optical_frame";
  //std::cout << frame_id << std::endl;
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "BoxEstimation";
  marker_.id = id_;
  marker_.type = visualization_msgs::Marker::CUBE;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.pose.position.x = pos_x;
  marker_.pose.position.y = pos_y;
  marker_.pose.position.z = pos_z;
  marker_.pose.orientation.x = 0;
  marker_.pose.orientation.y = 0;
  marker_.pose.orientation.z = 0;
  marker_.pose.orientation.w = 1;
  marker_.scale.x = scale_x;
  marker_.scale.y = scale_y;
  marker_.scale.z = scale_z;
  marker_.color.a = ca;
  marker_.color.r = cr;
  marker_.color.g = cg;
  marker_.color.b = cb;
  marker_.lifetime = ros::Duration();
  return marker_;
}

//*******************
//* save point cloud
class SaveData {
protected:
  ros::NodeHandle nh_;
  sensor_msgs::PointCloud2 cloud_msg;
private:
  bool relative_mode; // flag for RELATIVE MODE (depth limitation)
  int captureNum;     // number of saved files
  char filename[ 2048 ];
  const char *save_base_dir;
public:
  void activateRelativeMode(){ relative_mode = true; }
  string cloud_topic_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher marker_pub_;

  SaveData( const char* _save_base_dir ) :
    relative_mode(false),
    captureNum(0),
    save_base_dir(_save_base_dir) {
    make_directories();
  }

  //*******************
  //* make directories
  void make_directories() {
    mkdir(save_base_dir, 0777);
    sprintf(filename,"%s/Points", save_base_dir );
    mkdir(filename, 0777);
  }

  //*******************
  //* save point cloud
  void save_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
      if ((cloud->width * cloud->height) == 0)
        return;
      pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
      pcl::fromROSMsg (*cloud, cloud_xyzrgb_);

      //* limit depth
      if( relative_mode ){
      	float dis_min = FLT_MAX;
      	for (int i=0; i<(int)cloud_xyzrgb_.points.size(); i++)
	  if( dis_min > cloud_xyzrgb_.points[ i ].z ) dis_min = cloud_xyzrgb_.points[ i ].z;
	limitPoint( cloud_xyzrgb_, cloud_xyzrgb, dis_min + DISTANCE_TH );
      }
      else
	limitPoint( cloud_xyzrgb_, cloud_xyzrgb, DISTANCE_TH );

      const int pnum = cloud_xyzrgb.points.size();
      float x_min = 10000000, y_min = 10000000, z_min = 10000000;
      float x_max = -10000000, y_max = -10000000, z_max = -10000000;
      for( int p=0; p<pnum; p++ ){
	if( cloud_xyzrgb.points[ p ].x < x_min ) x_min = cloud_xyzrgb.points[ p ].x;
	if( cloud_xyzrgb.points[ p ].y < y_min ) y_min = cloud_xyzrgb.points[ p ].y;
	if( cloud_xyzrgb.points[ p ].z < z_min ) z_min = cloud_xyzrgb.points[ p ].z;
	if( cloud_xyzrgb.points[ p ].x > x_max ) x_max = cloud_xyzrgb.points[ p ].x;
	if( cloud_xyzrgb.points[ p ].y > y_max ) y_max = cloud_xyzrgb.points[ p ].y;
	if( cloud_xyzrgb.points[ p ].z > z_max ) z_max = cloud_xyzrgb.points[ p ].z;
      }
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker_range", 1); 

      //* show the limited space
      if( relative_mode )
	marker_pub_.publish( setMarker( -1, (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2, x_max-x_min, y_max-y_min, z_max-z_min, 0.5, 0.0, 1.0, 0.0 ) );
      else
	marker_pub_.publish( setMarker( -1, (x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2, x_max-x_min, y_max-y_min, z_max-z_min, 0.1, 1.0, 0.0, 0.0 ) );
      
      //* publish
      pcl::toROSMsg (cloud_xyzrgb, cloud_msg);
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("save_points", 1);
      pub_.publish( cloud_msg );

      //* save data
      if( save_flg ){
	if( cloud_xyzrgb.points.size() != 0 ){
	  sprintf(filename,"%s/Points/%05d.pcd", save_base_dir, captureNum );
	  pcl::io::savePCDFile (filename, cloud_xyzrgb, true);
	  cout << "    captured. " << filename << endl;
	  cout << "Press any key to save." << endl;
	  captureNum++;
	}
	save_flg = false;
      }

  }

  void loop(){
      cloud_topic_ = "input";
      sub_ = nh_.subscribe ("input", 1,  &SaveData::save_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

//********************************
//* main
int main(int argc, char* argv[]) {
    cerr << argc << endl;
    cerr << argv[0] << " " << argv[1]  << " " << argv[2]  << " " << argv[3]  << " " << argv[4] << endl;
  if((argc!=3)&&(argc!=4)&&(argc!=5)&&(argc!=6)){
    cerr << "usage: " << argv[0] << " [save_dir_name] /input:=/camera/depth/points2" << endl;
    cerr << " or" << endl;
    cerr << "usage: " << argv[0] << " [save_dir_name] <distance_th(m)> /input:=/camera/depth/points2" << endl;
    cerr << "       (for relative-mode)" << endl;
    exit( EXIT_FAILURE );
  }
  ros::init (argc, argv, "saveData", ros::init_options::AnonymousName);

  SaveData saveData( argv[1] );
  if( (argc==3) || (argc==5) ){ // note that argc=(argc-1) after ros::init
    saveData.activateRelativeMode();
    DISTANCE_TH = atof( argv[2] );
  }

  saveData.loop();
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  char c;
  cout << "Press any key to save." << endl;

  while(ros::ok()){
    if( kbhit() ){
      c = getchar();    
      //if( c=='q' || c=='Q' ) break;
      save_flg = true;
    }
    else usleep(100000);
  }
  return 0;
}
