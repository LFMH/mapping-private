//***************************************************************************************//
//* sliding-box object detection and show the most similar regions to the target object *//
//***************************************************************************************//

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <float.h>
#include <color_voxel_recognition/pca.h>
#include <color_voxel_recognition/param.h>
#include <color_voxel_recognition_2/search_new.h>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/FILE_MODE>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

float distance_th;

SearchGRSD search_obj;
const int dim = 20; // for GRSD
int box_size;
float voxel_size;
float region_size;
float sliding_box_size_x, sliding_box_size_y, sliding_box_size_z;
bool exit_flg = false;
bool start_flg = true;
float detect_th = 0;
int rank_num;
//const double normals_radius_search = 0.02;

//******************************************
//* limit points by certain depth threshold
template <typename T>
int limitPoint( const pcl::PointCloud<T> input_cloud, pcl::PointCloud<T> &output_cloud, const float dis_th ){
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
  std::cout << "from " << input_cloud.points.size() << " to " << idx << " points" << std::endl;
  return idx;
}

//**********************************************************
//* voxelize, extract features and detect the target object
class VoxelizeAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d; 
  KdTreePtr normals_tree;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz_, cloud_xyz;
  pcl::PointCloud<pcl::Normal> cloud_normal;
  pcl::PointCloud<pcl::PointNormal> cloud_xyz_normal, cloud_downsampled;
  pcl::VoxelGrid<pcl::PointNormal> grid;
  double t1, t1_2, t2, t0, t0_2, tAll;
  int process_count;
public:
  std::string cloud_topic_;
  ros::Subscriber sub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  VoxelizeAndDetect() :
    tAll(0), process_count(0) {
  }

  void vad_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
      if ((cloud->width * cloud->height) == 0)
        return;
      //ROS_INFO ("Received %d data points in frame %s with the following fields: %s", (int)cloud->width * cloud->height, cloud->header.frame_id.c_str (), pcl::getFieldsList (*cloud).c_str ());
      //std::cout << "fromROSMsg?" << std::endl;
      pcl::fromROSMsg (*cloud, cloud_xyz_);
      //std::cout << "  fromROSMsg done." << std::endl;
      t0 = my_clock();

      if( limitPoint( cloud_xyz_, cloud_xyz, distance_th ) > 10 ){
	//std::cout << "  limit done." << std::endl;
	std::cout << "compute normals and voxelize...." << std::endl;

	//****************************************
	//* compute normals
	n3d.setInputCloud (cloud_xyz.makeShared());
	n3d.setRadiusSearch (normals_radius_search);
	normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
	n3d.setSearchMethod (normals_tree);
	n3d.compute (cloud_normal);
	pcl::concatenateFields (cloud_xyz, cloud_normal, cloud_xyz_normal);

	t0_2 = my_clock();
	
	//* voxelize
	getVoxelGrid( grid, cloud_xyz_normal, cloud_downsampled, voxel_size );
	std::cout << "     ...done.." << std::endl;
	
	const int pnum = cloud_downsampled.points.size();
	float x_min = 10000000, y_min = 10000000, z_min = 10000000;
	float x_max = -10000000, y_max = -10000000, z_max = -10000000;
	for( int p=0; p<pnum; p++ ){
	  if( cloud_downsampled.points[ p ].x < x_min ) x_min = cloud_downsampled.points[ p ].x;
	  if( cloud_downsampled.points[ p ].y < y_min ) y_min = cloud_downsampled.points[ p ].y;
	  if( cloud_downsampled.points[ p ].z < z_min ) z_min = cloud_downsampled.points[ p ].z;
	  if( cloud_downsampled.points[ p ].x > x_max ) x_max = cloud_downsampled.points[ p ].x;
	  if( cloud_downsampled.points[ p ].y > y_max ) y_max = cloud_downsampled.points[ p ].y;
	  if( cloud_downsampled.points[ p ].z > z_max ) z_max = cloud_downsampled.points[ p ].z;
	}
	//std::cout << x_min << " " << y_min << " " << z_min << std::endl;
	//std::cout << x_max << " " << y_max << " " << z_max << std::endl;
	//std::cout << grid.getMinBoxCoordinates() << std::endl;

	std::cout << "search start..." << std::endl;
	//****************************************
	//* object detection start
	t1 = my_clock();
	search_obj.cleanData();
	search_obj.setGRSD( dim, grid, cloud_xyz_normal, cloud_downsampled, voxel_size, box_size );
	t1_2 = my_clock();
	if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	  search_obj.searchWithoutRotation();
	t2 = my_clock();
	//* object detection end
	//****************************************
	std::cout << "  ...search done." << std::endl;
	
	tAll += t2 - t0;
	process_count++;
	std::cout << "normal estimation  :"<< t0_2 - t0 << " sec" << std::endl;
	std::cout << "voxelize           :"<< t1 - t0_2 << " sec" << std::endl;
	std::cout << "feature extraction : "<< t1_2 - t1 << " sec" <<std::endl;
	std::cout << "search             : "<< t2 - t1_2 << " sec" <<std::endl;
	std::cout << "all processes      : "<< t2 - t0 << " sec" << std::endl;
	std::cout << "AVERAGE            : "<< tAll / process_count << " sec" << std::endl;
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker_range", 1); 
	marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
	visualization_msgs::MarkerArray marker_array_msg_;
	
	//* show the limited space
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = "openni_rgb_optical_frame";
	marker_.header.stamp = ros::Time::now();
	marker_.ns = "BoxEstimation";
	marker_.id = -1;
	marker_.type = visualization_msgs::Marker::CUBE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.pose.position.x = (x_max+x_min)/2;
	marker_.pose.position.y = (y_max+y_min)/2;
	marker_.pose.position.z = (z_max+z_min)/2;
	marker_.pose.orientation.x = 0;
	marker_.pose.orientation.y = 0;
	marker_.pose.orientation.z = 0;
	marker_.pose.orientation.w = 1;
	marker_.scale.x = x_max-x_min;
	marker_.scale.y = x_max-x_min;
	marker_.scale.z = x_max-x_min;
	marker_.color.a = 0.1;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.lifetime = ros::Duration();
	marker_pub_.publish(marker_);
	
	for( int q=0; q<rank_num; q++ ){
	  if( search_obj.maxDot( q ) < detect_th ) break;
	  std::cout << search_obj.maxX( q ) << " " << search_obj.maxY( q ) << " " << search_obj.maxZ( q ) << std::endl;
	  std::cout << "dot " << search_obj.maxDot( q ) << std::endl;
	  //if( (search_obj.maxX( q )!=0)||(search_obj.maxY( q )!=0)||(search_obj.maxZ( q )!=0) ){
	  //* publish marker
	  visualization_msgs::Marker marker_;
	  //marker_.header.frame_id = "base_link";
	  marker_.header.frame_id = "openni_rgb_optical_frame";
	  marker_.header.stamp = ros::Time::now();
	  marker_.ns = "BoxEstimation";
	  marker_.id = q;
	  marker_.type = visualization_msgs::Marker::CUBE;
	  marker_.action = visualization_msgs::Marker::ADD;
	  marker_.pose.position.x = search_obj.maxX( q ) * region_size + sliding_box_size_x/2 + x_min;
	  marker_.pose.position.y = search_obj.maxY( q ) * region_size + sliding_box_size_y/2 + y_min;
	  marker_.pose.position.z = search_obj.maxZ( q ) * region_size + sliding_box_size_z/2 + z_min;
	  marker_.pose.orientation.x = 0;
	  marker_.pose.orientation.y = 0;
	  marker_.pose.orientation.z = 0;
	  marker_.pose.orientation.w = 1;
	  marker_.scale.x = sliding_box_size_x;
	  marker_.scale.y = sliding_box_size_y;
	  marker_.scale.z = sliding_box_size_z;
	  marker_.color.a = 0.5;
	  marker_.color.r = 0.0;
	  marker_.color.g = 1.0;
	  marker_.color.b = 0.0;
	  marker_.lifetime = ros::Duration();
	  // std::cerr << "BOX MARKER COMPUTED, WITH FRAME " << marker_.header.frame_id << " POSITION: " 
	  // 	    << marker_.pose.position.x << " " << marker_.pose.position.y << " " 
	  // 	    << marker_.pose.position.z << std::endl;
	  //   marker_pub_.publish (marker_);    
	  // }
	  marker_array_msg_.markers.push_back(marker_);
	}
	//std::cerr << "MARKER ARRAY published with size: " << marker_array_msg_.markers.size() << std::endl; 
	marker_array_pub_.publish(marker_array_msg_);
      }
      std::cout << "Waiting msg..." << std::endl;
  }
  
  void loop(){
      cloud_topic_ = "input";
      sub_ = nh_.subscribe ("input", 1,  &VoxelizeAndDetect::vad_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

//********************************
//* main
int main(int argc, char* argv[]) {
  if( (argc != 12) && (argc != 14) ){
    std::cerr << "usage: " << argv[0] << " [path] <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> <distance_th> /input:=/camera/rgb/points" << std::endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  ros::init (argc, argv, "detect_object_grsd", ros::init_options::AnonymousName);

  // read the length of voxel side
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  voxel_size = Param::readVoxelSize( tmpname );

  detect_th = atof( argv[9] );
  distance_th = atof( argv[10] );
  rank_num = atoi( argv[2] );

  // read the number of voxels in each subdivision's side of scene
  box_size = Param::readBoxSizeScene( tmpname );

  // set the dimension of the target object's subspace
  const int dim_model = atoi(argv[5]);
  if( dim <= dim_model ){
    std::cerr << "ERR: dim_model should be less than 20" << std::endl;
    exit( EXIT_FAILURE );
  }

  // determine the size of sliding box
  region_size = box_size * voxel_size;
  float tmp_val = atof(argv[6]) / region_size;
  int size1 = (int)tmp_val;
  if( ( ( tmp_val - size1 ) >= 0.5 ) || ( size1 == 0 ) ) size1++;
  tmp_val = atof(argv[7]) / region_size;
  int size2 = (int)tmp_val;
  if( ( ( tmp_val - size2 ) >= 0.5 ) || ( size2 == 0 ) ) size2++;
  tmp_val = atof(argv[8]) / region_size;
  int size3 = (int)tmp_val;
  if( ( ( tmp_val - size3 ) >= 0.5 ) || ( size3 == 0 ) ) size3++;
  sliding_box_size_x = size1 * region_size;
  sliding_box_size_y = size2 * region_size;
  sliding_box_size_z = size3 * region_size;

  // set variables
  sprintf( tmpname, "%s/param/max_r.txt", argv[1] );
  search_obj.setNormalizeVal( tmpname );
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[3]) );
  search_obj.readAxis( argv[4], dim, dim_model, ASCII_MODE_P, MULTIPLE_SIMILARITY );

  // object detection
  VoxelizeAndDetect vad;
  vad.loop();
  ros::spin();

  return 0;
}
