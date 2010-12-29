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
/* 物体検出を行い、類似度が閾値以上の領域を表示する                                                                                  */
/*  ./detectObj <rank_num> <exist_voxel_num_threshold> [model_pca_filename] <dim_model> <size1> <size2> <size3> <detect_th> */
/*  例 ./detectObj 10 30 models/phone1/pca_result 40 0.1 0.1 0.1 20                                                         */
/*  スペースキーかEnterキーを押すと、attentionモード（検出領域内のみを表示）と全体表示モードが切り替わる                                     */
/*  注） プレビューはMESH_VIEWがtrueならメッシュ、falseならボクセルで表示。                                                           */
/****************************************************************************************************************************/

using namespace std;

float DISTANCE_TH = 5.0;
const int DOWNSIZE_RATE = 1; //3;

//************************
//* その他のグローバル変数など
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

//* 時間計測用
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

///////////////////////////////////////////////////////////////////////////////
//* プレビューのためのクラス
class TestView : public GlView {
public:
  bool attention_mode; // 検出された領域のみを表示するモード
  TestView() : attention_mode(false) { me = this; }
  ~TestView(){ if( me!=NULL ) delete[] me; }
  void initView( Voxel &voxel );

private:
  void display( void );
  void keyboard (unsigned char key, int x, int y);
};

//* ボクセルを表示する場合
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

    //* 類似度が閾値以上の領域をすべて表示する（赤いボックスで囲む）
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
//* ボクセル化、データの保存のためのクラス
class ViewAndDetect {
protected:
  ros::NodeHandle nh_;
private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
  bool relative_mode; // RELATIVE MODEにするか否か
  double t1, t2;
public:
  void activateRelativeMode(){ relative_mode = true; }
    string cloud_topic_;
    pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;

  //***************
  //* コンストラクタ
  ViewAndDetect() :
    relative_mode(false) {
  }

  //*******************************
  //* ボクセル化、データの保存
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
      //* ボクセル化
      voxel.points2voxel( cloud_xyzrgb, SIMPLE_REVERSE ); // REVERSEMODEは関係ないので何でも良い
      cout << "  voxelize..." << endl;
      if( DOWNSIZE_RATE != 1 )
	voxel_forView.points2voxel( cloud_xyzrgb, SIMPLE_REVERSE ); // REVERSEMODEは関係ないので何でも良い
      cout << "  voxelize............." << endl;
      voxel_bin = voxel;
      voxel_bin.binarize( color_threshold_r, color_threshold_g, color_threshold_b );
      cout << "  voxelize done." << endl;
    
      //****************************************
      //* 物体検出
      search_obj.cleanData();
      search_obj.setData( voxel, voxel_bin, dim, box_size ); //* sceneの（分割領域毎の）特徴量とボクセル数の作成 
      if( ( search_obj.XYnum() != 0 ) && ( search_obj.Znum() != 0 ) )
	search_obj.search();
      //* 物体検出 ここまで
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

  voxel_size = Param::readVoxelSize();  //* ボクセルの一辺の長さ（mm）の読み込み
  voxel.setVoxelSize( voxel_size );
  voxel_bin.setVoxelSize( voxel_size );
  voxel_forView.setVoxelSize( voxel_size * DOWNSIZE_RATE );

  detect_th = atof( argv[8] );
  rank_num = atoi( argv[1] );
  v_state = CREATE;

  //****************************************
  //* 物体検出のための準備
  box_size = Param::readBoxSize_scene();  //* 分割領域の大きさの読み込み

  //* 次元数など
  dim = Param::readDim();  //* 圧縮したCCHLAC特徴ベクトルの次元数の読み込み
  const int dim_model = atoi(argv[4]);  //* 検出対象物体の部分空間の基底の次元数
  if( dim <= dim_model ){
    cerr << "ERR: dim_model should be less than dim(in dim.txt)" << endl; // 注 特徴量の次元数よりも多くなくてはいけません
    exit( EXIT_FAILURE );
  }
  //* 検出ボックスの大きさを決定する
  const float region_size = box_size * voxel_size;
  float tmp_val = atof(argv[5]) / region_size;
  int size1 = (int)tmp_val;
  if( ( ( tmp_val - size1 ) >= 0.5 ) || ( size1 == 0 ) ) size1++; // 四捨五入
  tmp_val = atof(argv[6]) / region_size;
  int size2 = (int)tmp_val;
  if( ( ( tmp_val - size2 ) >= 0.5 ) || ( size2 == 0 ) ) size2++; // 四捨五入
  tmp_val = atof(argv[7]) / region_size;
  int size3 = (int)tmp_val;
  if( ( ( tmp_val - size3 ) >= 0.5 ) || ( size3 == 0 ) ) size3++; // 四捨五入

  //* 変数をセット
  search_obj.setRange( size1, size2, size3 );
  search_obj.setRank( rank_num );
  search_obj.setThreshold( atoi(argv[2]) );
  search_obj.readAxis( argv[3], dim, dim_model, ASCII_MODE_P, true );  //* 検出対象物体の部分空間の基底軸の読み込み

  //* RGB二値化の閾値の読み込み
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* CCHLAC特徴を圧縮する際に使用する主成分軸の読み込み
  PCA pca;
  pca.read("scene/pca_result", ASCII_MODE_P );
  Matrix axis = pca.Axis();
  axis.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3, dim );
  Matrix axis_t = axis.transpose();
  ColumnVector variance = pca.Variance();
  search_obj.setSceneAxis( axis_t, variance, dim );  //* 圧縮部分空間の基底軸のセット

  // 物体検出のための準備 ここまで
  //****************************************

  //*****************************************//
  //* 以下は、saveData.cppのmain関数と同じ *//
  //*****************************************//

  // ボクセル（かメッシュ）のプレビューとデータ取得の準備
  ViewAndDetect vad;

  // OpenGLによる描画
  glutInit(&argc, argv);
  TestView testView;
  GLUTwindow = testView.createWindow( 640, 480, 400, 100, argv[0] );

  // ループのスタート
  vad.loop();
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  //ros::waitForShutdown();
  // ros::MultiThreadedSpinner spinner(2);
  // spinner.spin();
  testView.start();

  return 0;
}
