#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <color_voxel_recognition/glView.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <float.h>
#include "./FILE_MODE"
//#include "../param/CAM_SIZE"
#include <ros/ros.h>
#include "pcl/io/pcd_io.h"
#include "pcl_ros/subscriber.h"
#include <sys/types.h>
#include <sys/stat.h>

/**************************************************************************/
/* Enterキーか's'を入力することで、カメラ画像とSRデータを保存する                    */
/* 同じフォルダに、プレビューに用いたメッシュ化のパラメータファイルparam.txtを保存する   */
/* 注） RELATIVE MODE: 末尾のコマンドライン引数でDISTANCE_TH(m)を指定すると、      */
/*       最も近い点からDISTANCE_TH(m)の奥行きまでのみをメッシュ化する              */
/*     （対象物体を見せて学習させる際の自動セグメンテーション方法として使用する）        */
/**************************************************************************/

using namespace std;

float DISTANCE_TH = 5.0;

//************************
//* グローバル変数など
enum V_STATE { DISP, CREATE } v_state;
static int GLUTwindow = 0;
Voxel voxel;
bool save_flg = false;

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

///////////////////////////////////////////////////////////////////////////////
//* プレビューのためのクラス
class TestView : public GlView {
public:
  TestView(){ me = this; }
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
  viewpoint_init( x_min, x_max, y_min, y_max, z_min, z_max );
  cout << "x_min: "<< x_min << "  x_max: " << x_max << endl;
  cout << "y_min: "<< y_min << "  y_max: " << y_max << endl;
  cout << "z_min: "<< z_min << "  z_max: " << z_max << endl;
}

void TestView::display(void)
{
  if( v_state == DISP ){
    cout << "DISP" << endl;
    display_init( (float)2.0, true );
    initView( voxel );
    dispVoxel( voxel );
    cout << voxel.Xsize() << " " << voxel.Ysize() << " " << endl;
    glutSwapBuffers();
    v_state = CREATE;
  }
}    

void TestView::keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 's':
  case 0x0D : // Enter
    save_flg = true;
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

/////////////////////////////////////////////
//* ボクセル化、データの保存のためのクラス
class ViewAndSave {
protected:
  ros::NodeHandle nh_;
private:
  bool relative_mode; // RELATIVE MODEにするか否か
  int captureNum; // セーブしたファイル数
  char filename[ 2048 ];
  const char *save_base_dir;
public:
  void activateRelativeMode(){ relative_mode = true; }
    string cloud_topic_;
    pcl_ros::Subscriber<sensor_msgs::PointCloud2> sub_;

  //***************
  //* コンストラクタ
  ViewAndSave( const char* _save_base_dir ) :
    relative_mode(false),
    captureNum(0),
    save_base_dir(_save_base_dir) {
    make_directories();
  }

  //**********************
  //* セーブするフォルダの生成  
  void make_directories() {
    mkdir(save_base_dir, 0777);
    sprintf(filename,"%s/Points", save_base_dir );
    mkdir(filename, 0777);
  }

  //*******************************
  //* ボクセル化、データの保存
  void vas_cb(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    if( v_state == DISP ) usleep( 1000 );
    else{
      cout << "CREATE" << endl;
      if ((cloud->width * cloud->height) == 0)
        return;
      ROS_INFO ("Received %d data points in frame %s with the following fields: %s", (int)cloud->width * cloud->height, cloud->header.frame_id.c_str (), pcl::getFieldsList (*cloud).c_str ());
      pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb_, cloud_xyzrgb;
      pcl::fromROSMsg (*cloud, cloud_xyzrgb_);

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
      
      //* ボクセル化
      voxel.points2voxel( cloud_xyzrgb, SIMPLE_REVERSE ); // REVERSEMODEは関係ないので何でも良い

      //* データの保存
      if( save_flg ){
	save_flg = false;
	if( ( voxel.Xsize() != 0 ) && ( voxel.Ysize() != 0 ) && ( voxel.Zsize() != 0 ) ){

	  sprintf(filename,"%s/Points/%03d.pcd", save_base_dir, captureNum );
	  //pcl::io::savePCDFile (filename, *cloud, Eigen3::Vector4f::Zero (), Eigen3::Quaternionf::Identity (), true);
	  pcl::io::savePCDFile (filename, cloud_xyzrgb, true);
	  cout << "captured. " << captureNum << endl;
	  captureNum++;
	}
      }

      v_state = DISP;
    }
  }

  void loop(){
      cloud_topic_ = "input";

      sub_.subscribe (nh_, "input", 1,  boost::bind (&ViewAndSave::vas_cb, this, _1));
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
  }
};

///////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  if((argc!=3)&&(argc!=4)){
    cerr << "usage: " << argv[0] << " [save_dir_name] /input:=/camera/depth/points2" << endl;
    cerr << " or" << endl;
    cerr << "usage: " << argv[0] << " [save_dir_name] <distance_th(m)> /input:=/camera/depth/points2" << endl;
    cerr << "       (for relative-mode)" << endl;
    exit( EXIT_FAILURE );
  }
  ros::init (argc, argv, "saveData", ros::init_options::AnonymousName);

  voxel.setVoxelSize( Param::readVoxelSize() );
  v_state = CREATE;

  // ボクセル（かメッシュ）のプレビューとデータ取得の準備
  ViewAndSave vas( argv[1] );
  if( argc==3 ){ // 注！！ ros::init の後なのでコマンドライン引数の数が１こ少なくなってる
    vas.activateRelativeMode();
    DISTANCE_TH = atof( argv[2] );
  }
  //cout << argc << " " << DISTANCE_TH <<  endl;

  // OpenGLによる描画
  glutInit(&argc, argv);
  TestView testView;
  GLUTwindow = testView.createWindow( 640, 480, 400, 100, argv[0] );

  // ループのスタート
  //boost::thread thread_create( boost::bind(&testView_start, testView) );
  vas.loop();
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  testView.start();

  return 0;
}
