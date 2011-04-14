#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <float.h>
#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition_2/Search_new.hpp>
#include "c3_hlac/c3_hlac_tools.h"
#include "color_voxel_recognition/FILE_MODE"
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

/****************************************************************************************************************************/
/* 物体検出を行い、類似度が閾値以上の領域を表示する                                                                                  */
/*  ./detectObj <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> */
/*  例 ./detectObj 10 30 models/phone1/pca_result 40 0.1 0.1 0.1 20                                                         */
/*  スペースキーかEnterキーを押すと、attentionモード（検出領域内のみを表示）と全体表示モードが切り替わる                                     */
/*  注） プレビューはMESH_VIEWがtrueならメッシュ、falseならボクセルで表示。                                                           */
/****************************************************************************************************************************/

typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
using namespace std;
using namespace Eigen;

float distance_th;

//************************
//* その他のグローバル変数など
Search_GRSD search_obj;
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
  cout << "from " << input_cloud.points.size() << " to " << idx << " points" << endl;
  return idx;
}

/////////////////////////////////////////////
//* ボクセル化、データの保存のためのクラス
class ViewAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d; 
  KdTreePtr normals_tree;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz_, cloud_xyz;
  pcl::PointCloud<pcl::Normal> cloud_normal;
  pcl::PointCloud<pcl::PointNormal> cloud_xyz_normal, cloud_downsampled;
  pcl::VoxelGrid<pcl::PointNormal> grid;
  bool relative_mode; // RELATIVE MODEにするか否か
  double t1, t1_2, t2, t0, t0_2, tAll;
  int process_count;
public:
  void activateRelativeMode(){ relative_mode = true; }
  string cloud_topic_;
  //pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;
  ros::Subscriber sub_;
  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;
  //***************
  //* コンストラクタ
  ViewAndDetect() :
    relative_mode(false), tAll(0), process_count(0) {
  }

  //*******************************
  //* ボクセル化、データの保存
  void vad_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
      if ((cloud->width * cloud->height) == 0)
        return;
      //ROS_INFO ("Received %d data points in frame %s with the following fields: %s", (int)cloud->width * cloud->height, cloud->header.frame_id.c_str (), pcl::getFieldsList (*cloud).c_str ());
      //cout << "fromROSMsg?" << endl;
      pcl::fromROSMsg (*cloud, cloud_xyz_);
      //cout << "  fromROSMsg done." << endl;
      cout << "CREATE" << endl;
      t0 = my_clock();

      if( limitPoint( cloud_xyz_, cloud_xyz, distance_th ) > 10 ){
	//cout << "  limit done." << endl;
	cout << "compute normals and voxelize...." << endl;

	//****************************************
	//* compute normals
	n3d.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud_xyz));
	n3d.setRadiusSearch (normals_radius_search);
	normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
	n3d.setSearchMethod (normals_tree);
	n3d.compute (cloud_normal);
	pcl::concatenateFields (cloud_xyz, cloud_normal, cloud_xyz_normal);

	t0_2 = my_clock();
	
	//* voxelize
	getVoxelGrid( grid, cloud_xyz_normal, cloud_downsampled, voxel_size );
	cout << "     ...done.." << endl;
	
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
	//cout << x_min << " " << y_min << " " << z_min << endl;
	//cout << x_max << " " << y_max << " " << z_max << endl;
	//cout << grid.getMinBoxCoordinates() << endl;

	cout << "search start..." << endl;
	t1 = my_clock();
	//****************************************
	//* 物体検出
	search_obj.cleanData();
	search_obj.setGRSD( dim, grid, cloud_xyz_normal, cloud_downsampled, voxel_size, box_size );
	t1_2 = my_clock();
	if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	  search_obj.search_withoutRotation();
	
	//* 物体検出 ここまで
	//****************************************
	t2 = my_clock();
	cout << "  ...search done." << endl;
	
	tAll += t2 - t0;
	process_count++;
	cout << "normal estimation  :"<< t0_2 - t0 << " sec" << endl;
	cout << "voxelize           :"<< t1 - t0_2 << " sec" << endl;
	cout << "feature extraction : "<< t1_2 - t1 << " sec" <<endl;
	cout << "search             : "<< t2 - t1_2 << " sec" <<endl;
	cout << "all processes      : "<< t2 - t0 << " sec" << endl;
	cout << "AVERAGE            : "<< tAll / process_count << " sec" << endl;
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
	  cout << search_obj.maxX( q ) << " " << search_obj.maxY( q ) << " " << search_obj.maxZ( q ) << endl;
	  cout << "dot " << search_obj.maxDot( q ) << endl;
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
      cout << "Waiting msg..." << endl;
  }
  
  void loop(){
      cloud_topic_ = "input";
      sub_ = nh_.subscribe ("input", 1,  &ViewAndDetect::vad_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  if( (argc != 12) && (argc != 14) ){
    cerr << "usage: " << argv[0] << " [path] <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> <distance_th> /input:=/camera/rgb/points" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  ros::init (argc, argv, "detectObj_GRSD", ros::init_options::AnonymousName);

  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  voxel_size = Param::readVoxelSize( tmpname );  //* ボクセルの一辺の長さ（mm）の読み込み

  detect_th = atof( argv[9] );
  distance_th = atof( argv[10] );
  rank_num = atoi( argv[2] );

  //****************************************
  //* 物体検出のための準備
  box_size = Param::readBoxSize_scene( tmpname );  //* 分割領域の大きさの読み込み

  const int dim_model = atoi(argv[5]);  //* 検出対象物体の部分空間の基底の次元数
  if( dim <= dim_model ){
    cerr << "ERR: dim_model should be less than 20" << endl; // 注 特徴量の次元数よりも多くなくてはいけません
    exit( EXIT_FAILURE );
  }
  //* 検出ボックスの大きさを決定する
  region_size = box_size * voxel_size;
  float tmp_val = atof(argv[6]) / region_size;
  int size1 = (int)tmp_val;
  if( ( ( tmp_val - size1 ) >= 0.5 ) || ( size1 == 0 ) ) size1++; // 四捨五入
  tmp_val = atof(argv[7]) / region_size;
  int size2 = (int)tmp_val;
  if( ( ( tmp_val - size2 ) >= 0.5 ) || ( size2 == 0 ) ) size2++; // 四捨五入
  tmp_val = atof(argv[8]) / region_size;
  int size3 = (int)tmp_val;
  if( ( ( tmp_val - size3 ) >= 0.5 ) || ( size3 == 0 ) ) size3++; // 四捨五入
  sliding_box_size_x = size1 * region_size;
  sliding_box_size_y = size2 * region_size;
  sliding_box_size_z = size3 * region_size;

  //* 変数をセット
  sprintf( tmpname, "%s/param/max_r.txt", argv[1] );
  search_obj.setNormalizeVal( tmpname );
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[3]) );
  search_obj.readAxis( argv[4], dim, dim_model, ASCII_MODE_P, MULTIPLE_SIMILARITY );  //* 検出対象物体の部分空間の基底軸の読み込み

  // 物体検出のための準備 ここまで
  //****************************************

  // ループのスタート
  ViewAndDetect vad;
  vad.loop();
  ros::spin();

  return 0;
}
