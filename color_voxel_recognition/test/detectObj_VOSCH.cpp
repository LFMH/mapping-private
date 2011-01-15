#include <visualization_msgs/Marker.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <float.h>
#include <color_voxel_recognition/glView.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA_eigen.hpp>
#include <color_voxel_recognition/Param.hpp>
//#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Search_VOSCH.hpp>
#include "color_chlac/grsd_colorCHLAC_tools.h"
#include "../param/FILE_MODE"
#include "../param/CAM_SIZE"
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl_ros/subscriber.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

/****************************************************************************************************************************/
/* ʪ�θ��Ф�Ԥ�������٤����Ͱʾ���ΰ��ɽ������                                                                                  */
/*  ./detectObj <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> */
/*  �� ./detectObj 10 30 models/phone1/pca_result 40 0.1 0.1 0.1 20                                                         */
/*  ���ڡ���������Enter�����򲡤��ȡ�attention�⡼�ɡʸ����ΰ���Τߤ�ɽ���ˤ�����ɽ���⡼�ɤ��ڤ��ؤ��                                     */
/*  ��� �ץ�ӥ塼��MESH_VIEW��true�ʤ��å��塢false�ʤ�ܥ������ɽ����                                                           */
/****************************************************************************************************************************/

using namespace std;
using namespace Eigen3;

float DISTANCE_TH = 1.0;

//************************
//* ����¾�Υ����Х��ѿ��ʤ�
Voxel voxel;

SearchObjVOSCH search_obj;
int color_threshold_r, color_threshold_g, color_threshold_b;
int dim;
int box_size;
float voxel_size;
bool exit_flg = false;
bool start_flg = true;
float detect_th = 0;
int rank_num;

//* Note that x and y values are inverted.
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
      output_cloud.points[ idx ].x = - input_cloud.points[ i ].x; // inverted.
      output_cloud.points[ idx ].y = - input_cloud.points[ i ].y; // inverted.
      idx++;
    }
  }
  output_cloud.width = idx;
  output_cloud.height = 1;
  output_cloud.points.resize( idx );
}

/////////////////////////////////////////////
//* �ܥ����벽���ǡ�������¸�Τ���Υ��饹
class ViewAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_normal, cloud_downsampled;
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
  bool relative_mode; // RELATIVE MODE�ˤ��뤫�ݤ�
  double t1, t2;
public:
  void activateRelativeMode(){ relative_mode = true; }
    string cloud_topic_;
    pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;

  //***************
  //* ���󥹥ȥ饯��
  ViewAndDetect() :
    relative_mode(false) {
  }

  //*******************************
  //* �ܥ����벽���ǡ�������¸
  void vad_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
      cout << "CREATE" << endl;
      t1 = my_clock();

      if ((cloud->width * cloud->height) == 0)
        return;
      //ROS_INFO ("Received %d data points in frame %s with the following fields: %s", (int)cloud->width * cloud->height, cloud->header.frame_id.c_str (), pcl::getFieldsList (*cloud).c_str ());
      //cout << "fromROSMsg?" << endl;
      pcl::fromROSMsg (*cloud, cloud_xyzrgb_);
      //cout << "  fromROSMsg done." << endl;

      //* limit distance
      //* Note that x and y values are inverted.
      if( relative_mode ){
      	float dis_min = FLT_MAX;
      	for (int i=0; i<(int)cloud_xyzrgb_.points.size(); i++)
	  if( dis_min > cloud_xyzrgb_.points[ i ].z ) dis_min = cloud_xyzrgb_.points[ i ].z;
	limitPoint( cloud_xyzrgb_, cloud_xyzrgb, dis_min + DISTANCE_TH );
      }
      else
	limitPoint( cloud_xyzrgb_, cloud_xyzrgb, DISTANCE_TH );
      
      cout << "  limit done." << endl;

      //****************************************
      //* compute normals
      computeNormal( cloud_xyzrgb, cloud_normal );
      
      //* voxelize
      getVoxelGrid( grid, cloud_normal, cloud_downsampled, voxel_size );
      voxel.points2voxel( cloud_downsampled, SIMPLE_REVERSE );
    
      int pnum = cloud_downsampled.points.size();
      float x_min = 10000000, y_min = 10000000, z_min = 10000000;
      for( int p=0; p<pnum; p++ ){
      	if( cloud_downsampled.points[ p ].x < x_min ) x_min = cloud_downsampled.points[ p ].x;
      	if( cloud_downsampled.points[ p ].y < y_min ) y_min = cloud_downsampled.points[ p ].y;
      	if( cloud_downsampled.points[ p ].z < z_min ) z_min = cloud_downsampled.points[ p ].z;
      }

      //****************************************
      //* ʪ�θ���
      search_obj.cleanData();
      search_obj.setData( dim, color_threshold_r, color_threshold_g, color_threshold_b, grid, cloud_normal, cloud_downsampled, voxel_size, box_size, false );
      if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	search_obj.search();
      //* ʪ�θ��� �����ޤ�
      //****************************************
      cout << "  search done." << endl;

      // if( ( voxel.Xsize() != 0 ) && ( voxel.Ysize() != 0 ) && ( voxel.Zsize() != 0 ) )
      // 	v_state = DISP;
      t2 = my_clock();
      cout << "Time for all processes: "<< t2 - t1 << endl;

      //* publish marker
      visualization_msgs::Marker marker_;
      marker_.header.frame_id = "openni_depth_optical_frame";
      marker_.header.stamp = ros::Time::now();
      marker_.ns = "BoxEstimation";
      marker_.id = 0;
      marker_.type = visualization_msgs::Marker::CUBE;
      marker_.action = visualization_msgs::Marker::ADD;
      //marker_.pose.position.x = x_min + search_obj.maxX( q ) * region_size;
      //marker_.pose.position.y = y_min + search_obj.maxY( q ) * region_size;
      //marker_.pose.position.z = z_min + search_obj.maxZ( q ) * region_size;
      marker_.pose.orientation.x = 0;
      marker_.pose.orientation.y = 0;
      marker_.pose.orientation.z = 0;
      marker_.pose.orientation.w = 1;
      //marker_.scale.x = sliding_box_size;
      //marker_.scale.y = sliding_box_size;
      //marker_.scale.z = sliding_box_size;
      marker_.color.a = 0.1;
      marker_.color.r = 0.0;
      marker_.color.g = 1.0;
      marker_.color.b = 0.0;
      std::cerr << "BOX MARKER COMPUTED, WITH FRAME " << marker_.header.frame_id << std::endl;
      
      ros::Publisher marker_pub_;
      marker_pub_.publish (marker_);      
  }

  void loop(){
      cloud_topic_ = "input";
      sub_.subscribe (nh_, "input", 1000,  boost::bind (&ViewAndDetect::vad_cb, this, _1));
      //ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  if( argc != 10 ){
    cerr << "usage: " << argv[0] << " <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> /input:=/camera/depth/points2" << endl;
    exit( EXIT_FAILURE );
  }
  ros::init (argc, argv, "detectObj", ros::init_options::AnonymousName);

  voxel_size = Param::readVoxelSize();  //* �ܥ�����ΰ��դ�Ĺ����mm�ˤ��ɤ߹���
  voxel.setVoxelSize( voxel_size );

  detect_th = atof( argv[8] );
  rank_num = atoi( argv[1] );

  //****************************************
  //* ʪ�θ��ФΤ���ν���
  box_size = Param::readBoxSize_scene();  //* ʬ���ΰ���礭�����ɤ߹���

  //* �������ʤ�
  dim = Param::readDim();  //* ���̤�����ħ�٥��ȥ�μ��������ɤ߹���
  const int dim_model = atoi(argv[4]);  //* �����о�ʪ�Τ���ʬ���֤δ���μ�����
  if( dim <= dim_model ){
    cerr << "ERR: dim_model should be less than dim(in dim.txt)" << endl; // �� ��ħ�̤μ���������¿���ʤ��ƤϤ����ޤ���
    exit( EXIT_FAILURE );
  }
  //* ���Хܥå������礭������ꤹ��
  const float region_size = box_size * voxel_size;
  float tmp_val = atof(argv[5]) / region_size;
  int size1 = (int)tmp_val;
  if( ( ( tmp_val - size1 ) >= 0.5 ) || ( size1 == 0 ) ) size1++; // �ͼθ���
  tmp_val = atof(argv[6]) / region_size;
  int size2 = (int)tmp_val;
  if( ( ( tmp_val - size2 ) >= 0.5 ) || ( size2 == 0 ) ) size2++; // �ͼθ���
  tmp_val = atof(argv[7]) / region_size;
  int size3 = (int)tmp_val;
  if( ( ( tmp_val - size3 ) >= 0.5 ) || ( size3 == 0 ) ) size3++; // �ͼθ���

  //* �ѿ��򥻥å�
  search_obj.setNormalizeVal( "param/minmax_r.txt" );
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[2]) );
  search_obj.readAxis( argv[3], dim, dim_model, ASCII_MODE_P, false );  //* �����о�ʪ�Τ���ʬ���֤δ��켴���ɤ߹���

  //* RGB���Ͳ������ͤ��ɤ߹���
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* ��ħ�򰵽̤���ݤ˻��Ѥ������ʬ�����ɤ߹���
  PCA_eigen pca;
  pca.read("models/compress_axis", ASCII_MODE_P );
  MatrixXf tmpaxis = pca.Axis();
  MatrixXf axis = tmpaxis.block( 0,0,tmpaxis.rows(),dim );
  MatrixXf axis_t = axis.transpose();
  VectorXf variance = pca.Variance();
  search_obj.setSceneAxis( axis_t, variance, dim );  //* ������ʬ���֤δ��켴�Υ��å�

  // ʪ�θ��ФΤ���ν��� �����ޤ�
  //****************************************

  //*****************************************//
  //* �ʲ��ϡ�saveData.cpp��main�ؿ���Ʊ�� *//
  //*****************************************//

  // �ܥ�����ʤ���å���ˤΥץ�ӥ塼�ȥǡ��������ν���
  ViewAndDetect vad;

  // �롼�פΥ�������
  vad.loop();
  ros::spin();

  return 0;
}
