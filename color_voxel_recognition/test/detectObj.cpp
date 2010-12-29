#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <float.h>
#include <color_voxel_recognition/glView.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Search.hpp>
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

float DISTANCE_TH = 5.0;
const int DOWNSIZE_RATE = 1; //3;

//************************
//* ����¾�Υ����Х��ѿ��ʤ�
enum V_STATE { DISP, CREATE } v_state;
static int GLUTwindow = 0;
Voxel voxel;
Voxel voxel_bin;
Voxel voxel_forView; // downsized voxel for view

SearchObj search_obj;
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

//* ���ַ�¬��
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

///////////////////////////////////////////////////////////////////////////////
//* �ץ�ӥ塼�Τ���Υ��饹
class TestView : public GlView {
public:
  bool attention_mode; // ���Ф��줿�ΰ�Τߤ�ɽ������⡼��
  TestView() : attention_mode(false) { me = this; }
  ~TestView(){ if( me!=NULL ) delete[] me; }
  void initView( Voxel &voxel );

private:
  void display( void );
  void keyboard (unsigned char key, int x, int y);
};

//* �ܥ������ɽ��������
void TestView::initView( Voxel &voxel ){
  int x_min,x_max,y_min,y_max,z_min,z_max;
  voxel.getMinMax( x_min, x_max, y_min, y_max, z_min, z_max );
  viewpoint_init( x_min/DOWNSIZE_RATE, x_max/DOWNSIZE_RATE, y_min/DOWNSIZE_RATE, y_max/DOWNSIZE_RATE, z_min/DOWNSIZE_RATE, z_max/DOWNSIZE_RATE );
}

void TestView::display(void)
{
  if( v_state == DISP ){
    cout << "DISP" << endl;
    display_init( 2 );
    if( DOWNSIZE_RATE == 1 )
      initView( voxel );
    else
      initView( voxel_forView );

    //* ����٤����Ͱʾ���ΰ�򤹤٤�ɽ��������֤��ܥå����ǰϤ��
    for( int q=0; q<rank_num; q++ ){
      if( search_obj.maxDot( q ) < detect_th ) break;
      if( (search_obj.maxX( q )!=0)||(search_obj.maxY( q )!=0)||(search_obj.maxZ( q )!=0) ){
	const float sx = search_obj.maxX( q ) * box_size + 1;
	const float sy = search_obj.maxY( q ) * box_size + 1;
	const float sz = search_obj.maxZ( q ) * box_size + 1;
	const float gx = sx + search_obj.maxXrange( q ) * box_size;
	const float gy = sy + search_obj.maxYrange( q ) * box_size;
	const float gz = sz + search_obj.maxZrange( q ) * box_size;
	cout << "dot " << search_obj.maxDot( q ) << endl;	
	dispBox( -gx/DOWNSIZE_RATE, -sx/DOWNSIZE_RATE, sy/DOWNSIZE_RATE, gy/DOWNSIZE_RATE, -gz/DOWNSIZE_RATE, -sz/DOWNSIZE_RATE, 3 );
	if( attention_mode ){
	  if( DOWNSIZE_RATE == 1 )
	    dispVoxel( voxel, -gx, -sx, sy, gy, -gz, -sz );
	  else
	    dispVoxel( voxel_forView, -gx/DOWNSIZE_RATE, -sx/DOWNSIZE_RATE, sy/DOWNSIZE_RATE, gy/DOWNSIZE_RATE, -gz/DOWNSIZE_RATE, -sz/DOWNSIZE_RATE );
	}
      }
    }
    cout << "drawing..." << endl;
    if(!attention_mode){
      if( DOWNSIZE_RATE == 1 )
	dispVoxel( voxel );
      else
	dispVoxel( voxel_forView );
    }

    v_state = CREATE;
    glutSwapBuffers();
    cout << "       ...done" << endl;
  }
  else usleep(100000);
}    

void TestView::keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case ' ':
  case 0x0D : // Enter
    if( attention_mode ) attention_mode = false;
    else attention_mode = true;
    break;

  case 'Q':
  case 'q':
  case 27: // ESCAPE
    glutDestroyWindow(GLUTwindow);
    exit(0);
    break;

  case 'p':
    capture("gomi/tmp.png");
    break;
  }
}
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//* �ܥ����벽���ǡ�������¸�Τ���Υ��饹
class ViewAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
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
    if( v_state == DISP ) usleep( 100000 );
    else{
      cout << "CREATE" << endl;
      t1 = my_clock();

      if ((cloud->width * cloud->height) == 0)
        return;
      ROS_INFO ("Received %d data points in frame %s with the following fields: %s", (int)cloud->width * cloud->height, cloud->header.frame_id.c_str (), pcl::getFieldsList (*cloud).c_str ());
      cout << "fromROSMsg?" << endl;
      pcl::fromROSMsg (*cloud, cloud_xyzrgb_);
      cout << "  fromROSMsg done." << endl;

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
      //* �ܥ����벽
      voxel.points2voxel( cloud_xyzrgb, SIMPLE_REVERSE ); // REVERSEMODE�ϴط��ʤ��Τǲ��Ǥ��ɤ�
      cout << "  voxelize..." << endl;
      if( DOWNSIZE_RATE != 1 )
	voxel_forView.points2voxel( cloud_xyzrgb, SIMPLE_REVERSE ); // REVERSEMODE�ϴط��ʤ��Τǲ��Ǥ��ɤ�
      cout << "  voxelize............." << endl;
      voxel_bin = voxel;
      voxel_bin.binarize( color_threshold_r, color_threshold_g, color_threshold_b );
      cout << "  voxelize done." << endl;
    
      //****************************************
      //* ʪ�θ���
      search_obj.cleanData();
      search_obj.setData( voxel, voxel_bin, dim, box_size ); //* scene�Ρ�ʬ���ΰ���Ρ���ħ�̤ȥܥ�������κ��� 
      if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	search_obj.search();
      //* ʪ�θ��� �����ޤ�
      //****************************************
      cout << "  search done." << endl;

      // if( ( voxel.Xsize() != 0 ) && ( voxel.Ysize() != 0 ) && ( voxel.Zsize() != 0 ) )
      // 	v_state = DISP;
      t2 = my_clock();
      cout << t2 - t1 << endl;
    
      v_state = DISP;
    }
  }

  void loop(){
      cloud_topic_ = "input";
      sub_.subscribe (nh_, "input", 1000,  boost::bind (&ViewAndDetect::vad_cb, this, _1));
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
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
  voxel_bin.setVoxelSize( voxel_size );
  voxel_forView.setVoxelSize( voxel_size * DOWNSIZE_RATE );

  detect_th = atof( argv[8] );
  rank_num = atoi( argv[1] );
  v_state = CREATE;

  //****************************************
  //* ʪ�θ��ФΤ���ν���
  box_size = Param::readBoxSize_scene();  //* ʬ���ΰ���礭�����ɤ߹���

  //* �������ʤ�
  dim = Param::readDim();  //* ���̤���CCHLAC��ħ�٥��ȥ�μ��������ɤ߹���
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
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[2]) );
  search_obj.readAxis( argv[3], dim, dim_model, ASCII_MODE_P, true );  //* �����о�ʪ�Τ���ʬ���֤δ��켴���ɤ߹���

  //* RGB���Ͳ������ͤ��ɤ߹���
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* CCHLAC��ħ�򰵽̤���ݤ˻��Ѥ������ʬ�����ɤ߹���
  PCA pca;
  pca.read("scene/pca_result", ASCII_MODE_P );
  Matrix axis = pca.Axis();
  axis.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3, dim );
  Matrix axis_t = axis.transpose();
  ColumnVector variance = pca.Variance();
  search_obj.setSceneAxis( axis_t, variance, dim );  //* ������ʬ���֤δ��켴�Υ��å�

  // ʪ�θ��ФΤ���ν��� �����ޤ�
  //****************************************

  //*****************************************//
  //* �ʲ��ϡ�saveData.cpp��main�ؿ���Ʊ�� *//
  //*****************************************//

  // �ܥ�����ʤ���å���ˤΥץ�ӥ塼�ȥǡ��������ν���
  ViewAndDetect vad;

  // OpenGL�ˤ������
  glutInit(&argc, argv);
  TestView testView;
  GLUTwindow = testView.createWindow( 640, 480, 400, 100, argv[0] );

  // �롼�פΥ�������
  vad.loop();
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  //ros::waitForShutdown();
  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();
  testView.start();

  return 0;
}
